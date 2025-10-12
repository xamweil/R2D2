"""A Ros wrapper for Daly BMS via dalybms library"""
from dalybms import DalyBMS
import serial
from sensor_msgs.msg import BatteryState

class BMS:
    def __init__(self, device, address=8):
        self.bms = DalyBMS(address=address)   # 8 is for UART
        self.bms.connect(device)

        self.current_state = BatteryState()
        

    def update_all(self, stamp):
        msg = BatteryState()

        soc = self.bms.get_soc()  # {"total_voltage","current","soc_percent"}
        if soc:
            msg.voltage = float(soc.get("total_voltage", math.nan))
            msg.current = float(soc.get("current", math.nan))  # + = discharging, Daly definition
            msg.percentage = float(soc.get("soc_percent", math.nan)) / 100.0
            soc_pct = soc.get("soc_percent", None)
        else:
            msg.voltage = math.nan
            msg.current = math.nan
            msg.percentage = math.nan
            soc_pct = None
        
        # Status
        status = self.bms.get_status()  # caches self.bms.status internally
        cell_count = status.get("cells") if status else None

        # cell voltages
        cell_voltages = self.bms.get_cell_voltages()  # {1: V, 2: V, ...} or False
        msg.cell_voltage = self._ordered_values(cell_voltages if cell_voltages else {})

        # Temperatures
        temps = self.bms.get_temperatures()            # {1: C, 2: C, ...} or False
        if temps:
            msg.cell_temperature = self._ordered_values(temps)
            msg.temperature = sum(temps.values()) / len(temps)  # average
        else:
            msg.cell_temperature = []
            msg.temperature = math.nan

        # MOSFET / charge capacity
        mosfet = self.bms.get_mosfet_status()  # includes capacity_ah (kinda "rated"/learned")
        msg.power_supply_status = self._map_status(mosfet, soc_pct)

        # Calculate capacity
        if mosfet and isinstance(mosfet.get("capacity_ah"), (int, float)):
            capacity_ah = float(mosfet["capacity_ah"])
            msg.capacity = capacity_ah
            if soc_pct is not None:
                msg.charge = capacity_ah * (float(soc_pct) / 100.0)
            else:
                msg.charge = math.nan
        else:
            msg.capacity = math.nan
            msg.charge = math.nan

        # NA
        msg.design_capacity = math.nan

        # Health/Errors
        errors = self.bms.get_errors()
        msg.power_supply_health = self._map_health(errors, temp_range)

        # Battery tech
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        msg.present = True
        
        msg.header.stamp = stamp
        self.current_state = msg


    def _ordered_values(self, mapping=None):
        """Convert {index: value} into a 1..N ordered list."""
        if not isinstance(mapping, dict) or not mapping:
            return []
        return [mapping[k] for k in sorted(mapping.keys())]

    def _map_status(self, mosfet_status=None, soc_pct=None) -> int:
        """
        Map Daly 'mode' + MOSFET booleans + SoC to BatteryState.power_supply_status.
        """
        if not mosfet_status:
            return BatteryState.POWER_SUPPLY_STATUS_UNKNOWN

        mode = mosfet_status.get("mode")
        chg = bool(mosfet_status.get("charging_mosfet"))
        dch = bool(mosfet_status.get("discharging_mosfet"))

        if mode == "charging" or chg:
            return BatteryState.POWER_SUPPLY_STATUS_CHARGING
        if mode == "discharging" or dch:
            return BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        # Decide between NOT_CHARGING vs FULL if SoC available
        if soc_pct is not None and soc_pct >= 99.5:
            return BatteryState.POWER_SUPPLY_STATUS_FULL
        return BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        
    def _map_health(self, errors=None, temp_range=None) -> int:
        """
        Priority (worst first): DEAD > OVERHEAT > OVERVOLTAGE > COLD > UNSPEC_FAILURE > GOOD
        """
        if errors:
            errs = [str(e).lower() for e in errors]

            def has_any(subs):
                return any(any(s in e for s in subs) for e in errs)

            # DEAD (severe undervoltage / no-charging)
            if has_any((
                "total voltage is too low level two alarm",
                "low voltage no charging",
            )):
                return BatteryState.POWER_SUPPLY_HEALTH_DEAD

            # OVERHEAT
            if has_any((
                "overtemperature",
                "temperature too high",
                "discharge temperature is too high",
            )):
                return BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT

            # OVERVOLTAGE
            if has_any((
                "over voltage",
                "total voltage is too high",
            )):
                return BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE

            # COLD
            if has_any((
                "temperature too low",
            )):
                return BatteryState.POWER_SUPPLY_HEALTH_COLD

            # Everything else ⇒ UNSPEC_FAILURE
            if has_any((
                "over current",
                "short circuit",
                "sensor fault", "sensor failure",
                "malfunction", "failure", "fault",
                "afe acquisition chip", "eeprom", "rtc clock",
                "precharge failure",
                "communications malfunction", "communication module malfunction",
                "mos adhesion failure", "mos breaker failure", "temperature detection sensor failure",
                "excessive differential pressure", "excessive temperature difference",
                "monomer collect drop off",
                "main pressure detection module",
                "soc is too high", "soc is too low",
            )):
                return BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE

            # Unknown error string but still an error present
            return BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE

        # No explicit errors: temperature-based fallback
        if isinstance(temp_range, dict):
            hi = temp_range.get("highest_temperature")
            lo = temp_range.get("lowest_temperature")
            if isinstance(hi, (int, float)) and hi >= 60.0:
                return BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT
            if isinstance(lo, (int, float)) and lo <= -20.0:
                return BatteryState.POWER_SUPPLY_HEALTH_COLD

        return BatteryState.POWER_SUPPLY_HEALTH_GOOD