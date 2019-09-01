#!/usr/bin/env python3
from enum import IntEnum

class HumanResponse(IntEnum):
    QUIT                = 0
    TRAJECTORY_PLAY     = 1
    TRAJECTORY_ERASE    = 2
    TRAJECTORY_VIEW     = 3
    MOVE_HOME           = 4
    MOVE_EXTEND         = 5
    MOVE_HEART          = 6
    MOVE_RANDOM         = 7
    MOVE_STRAIGHT       = 8
    MOVE_BUTTON         = 9
    GO			= 10
    

    @classmethod
    def has_value(cls, value):
        return any(value == item.value for item in cls)

class DornaMovement(IntEnum):
    RELATIVE = 1
    GLOBAL = 0
    DEFAULT_JOINT_SPEED = 2000
    DEFAULT_XYZ_SPEED = 2500

class DornaConnection(IntEnum):
    DISCONNECTED = 0
    CONNECTING = 1
    CONNECTED = 2

class DornaState(IntEnum):
    NOT_BUSY = 0
    RUNNING = 1
