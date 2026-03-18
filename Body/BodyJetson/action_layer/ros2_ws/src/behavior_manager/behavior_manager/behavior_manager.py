from __future__ import annotations

from typing import Any, Dict, List, Optional, Set

from behavior_manager.behavior_base import BehaviorBase


class BehaviorManager:
    """
    Generic scheduler/orchestrator for skills.

    Responsibilities:
    - register available skills
    - activate/deactivate skills
    - enforce resource conflicts
    - run active skills in a periodic loop
    - expose active skill names for status publishing
    """

    def __init__(self) -> None:
        self._skills: Dict[str, BehaviorBase] = {}
        self._active_skills: Dict[str, BehaviorBase] = {}
        self._last_terminal_output: Dict[str, Dict[str, Any]] = {}

    # ---------------------------------------------------------------------
    # Registration
    # ---------------------------------------------------------------------

    def register_skill(self, skill: BehaviorBase) -> None:
        if skill.name in self._skills:
            raise ValueError(f"Skill '{skill.name}' already registered")
        self._skills[skill.name] = skill

    def registered_skill_names(self) -> List[str]:
        return sorted(self._skills.keys())

    # ---------------------------------------------------------------------
    # Query helpers
    # ---------------------------------------------------------------------

    def active_skill_names(self) -> List[str]:
        return sorted(self._active_skills.keys())

    def has_active_skill(self, name: str) -> bool:
        return name in self._active_skills

    def has_any_active_skill(self) -> bool:
        return len(self._active_skills) > 0

    def active_resources(self) -> Set[str]:
        resources: Set[str] = set()
        for skill in self._active_skills.values():
            resources |= set(skill.required_resources())
        return resources

    # ---------------------------------------------------------------------
    # Lifecycle / scheduling
    # ---------------------------------------------------------------------

    def can_start_skill(self, skill_name: str) -> tuple[bool, str]:
        if skill_name not in self._skills:
            return False, f"unknown skill '{skill_name}'"

        if skill_name in self._active_skills:
            return False, f"skill '{skill_name}' already active"

        requested_resources = set(self._skills[skill_name].required_resources())
        conflicting_resources = requested_resources & self.active_resources()

        if conflicting_resources:
            return False, (
                f"resource conflict: {sorted(conflicting_resources)}"
            )

        return True, "accepted"

    def start_skill(self, skill_name: str, **kwargs: Any) -> None:
        if skill_name not in self._skills:
            raise ValueError(f"unknown skill '{skill_name}'")

        skill = self._skills[skill_name]
        skill.start(**kwargs)
        self._active_skills[skill_name] = skill

    def stop_skill(self, skill_name: str) -> None:
        skill = self._active_skills.pop(skill_name, None)
        if skill is not None:
            skill.stop()

    def stop_all_skills(self) -> None:
        for skill_name in list(self._active_skills.keys()):
            self.stop_skill(skill_name)

    # ---------------------------------------------------------------------
    # Periodic update
    # ---------------------------------------------------------------------
    def pop_terminal_output(self, skill_name: str) -> Optional[Dict[str, Any]]:
        return self._last_terminal_output.pop(skill_name, None)

    def update(self, inputs: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Run one manager tick.

        Returns a list of skill outputs, one per active skill.
        Skills that report done=True are stopped after this update.
        """
        outputs: List[Dict[str, Any]] = []
        skills_to_stop: List[str] = []

        for skill_name, skill in list(self._active_skills.items()):
            output = skill.update(inputs)
            if output is None:
                output = {}

            output["skill_name"] = skill_name
            outputs.append(output)

            if bool(output.get("done", False)):
                self._last_terminal_output[skill_name] = output
                skills_to_stop.append(skill_name)

        for skill_name in skills_to_stop:
            self.stop_skill(skill_name)

        return outputs