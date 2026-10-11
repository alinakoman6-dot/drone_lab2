from enum import Enum, auto


class FlightState(Enum):
    WAITING_FOR_SYSTEM = auto()
    PREARM_CHECK = auto()
    SETTING_GUIDED = auto()
    ARMING = auto()
    TAKING_OFF = auto()
    HOVERING = auto()
    # --- додано для лабораторної роботи №2 (політ до цілі) ---
    NAVIGATING = auto()
    STOPPING = auto()
    # ---------------------------------------------------------
    LANDING = auto()
    WAITING_FOR_LANDING = auto()
    FINISHED = auto()
    ERROR = auto()
