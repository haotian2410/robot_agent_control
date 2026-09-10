"""Typed planning errors consumed by MoveSkill."""


class PlanningError(RuntimeError):
    def __init__(self, message: str, *, code: str = "PLANNING_FAILED", recoverable: bool = False, details: object = None) -> None:
        super().__init__(message)
        self.code = code
        self.recoverable = recoverable
        self.details = details


class CollisionPlanningError(PlanningError):
    def __init__(self, message: str, *, details: object = None) -> None:
        super().__init__(message, code="COLLISION_DETECTED", recoverable=True, details=details)
