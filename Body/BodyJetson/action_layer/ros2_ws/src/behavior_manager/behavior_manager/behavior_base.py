from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Set


class BehaviorBase(ABC):
    """
    Base interface for all skills.

    Every skill should have the same lifecycle so the manager can call it
    generically without knowing internal skill details.
    """

    @property
    @abstractmethod
    def name(self) -> str:
        raise NotImplementedError

    @abstractmethod
    def required_resources(self) -> Set[str]:
        """
        Return the set of resource tags required by this skill.

        Example:
            {"head_motion"}

        The manager treats these as generic strings and only checks for overlap.
        """
        raise NotImplementedError

    @abstractmethod
    def start(self, **kwargs: Any) -> None:
        """
        Called once when the skill is activated.
        """
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        """
        Called once when the skill is stopped/cancelled.
        """
        raise NotImplementedError

    @abstractmethod
    def update(self, inputs: Dict[str, Any]) -> Dict[str, Any]:
        """
        Called periodically by the manager.

        Parameters
        ----------
        inputs:
            A dictionary of currently available subscribed inputs.
            Example for now:
                {
                    "tracking": <Detection2DArray or None>
                }

        Returns
        -------
        Dict[str, Any]
            Generic output dictionary. The node/manager interpret only a few
            agreed-upon keys, while each skill may also store internal data.

            Current convention:
                {
                    "motor_command": <MotorCommand or None>,
                    "feedback": {
                        "target_visible": bool,
                        "current_track_id": int,
                        "error_x": float,
                        "error_y": float,
                    },
                    "done": bool,
                    "success": bool,
                    "message": str,
                }
        """
        raise NotImplementedError