import threading

# Dictionary to hold a Lock for each port
_port_locks = {}


def get_port_lock(port: str) -> threading.Lock:
    """
    Retrieve a threading.Lock for the given serial port.
    If one does not exist yet, create it.

    :param port: Serial port identifier, e.g. '/dev/serial/by-id/...'
    :return: threading.Lock object for exclusive access to the port.
    """
    if port not in _port_locks:
        _port_locks[port] = threading.Lock()
    return _port_locks[port]