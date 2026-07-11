from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from ultralytics_dfine.model import DFINE

__all__ = ["DFINE"]


def __getattr__(name: str) -> Any:
    if name == "DFINE":
        from ultralytics_dfine.model import DFINE

        return DFINE
    raise AttributeError(name)
