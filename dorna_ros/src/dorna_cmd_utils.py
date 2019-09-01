#!/usr/bin/env python2

from enum import IntEnum
from dorna_enums import DornaMovement
import json
import numpy as np
import rospy

from moveit_msgs.msg import DisplayTrajectory

joint_names = ['j0', 'j1', 'j2', 'j3', 'j4']
axis_names = ['x', 'y', 'z', 'a', 'b']


def radians_to_deg(rad_values):
    deg_values = []
    for value in rad_values:
        deg_values.append(round(value*180./np.pi, 3))
    
    return deg_values

def get_cmd_msg(path, movement, speed, cmd_dict):
    param_dict = get_move_msg(path, movement, speed, cmd_dict)
    cmd = {"command":"move", "prm":param_dict}
    return cmd

def get_move_msg(path, movement, speed, cmd_dict):
    try:
        if type(path) == str and type(cmd_dict) == dict:
            param_dict = {"path":path, "movement":movement, "speed":speed}
            cmd = param_dict.copy()
            cmd.update(cmd_dict)
            # cmd = {**param_dict, **cmd_dict}
            return cmd
        else:
            raise ValueError
    except ValueError:
        raise

def get_play_msg(command, parameters, fulfill = False):
        try:
            if type(command)==str and type(parameters) == dict:
                cmd = {"command":command, "prm":parameters, "fulfill": fulfill}
                return json.dumps(cmd)
            else:
                raise ValueError
        except ValueError:
            raise

def get_traj_msg(traj):
        cmds = []
        cmds.append({"command": "g2core", "prm": "{jt:3}"})
        # print(traj.joint_trajectory)
        for point in traj.joint_trajectory.points:
            deg_values = radians_to_deg(point.positions)
            cmd_dict = dict(zip(joint_names, deg_values))
            cmds.append(get_cmd_msg(path = "joint", movement = DornaMovement.GLOBAL, speed = DornaMovement.DEFAULT_JOINT_SPEED, cmd_dict = cmd_dict))

        return cmds

# def get_dorna_cmd(point):
#         pos = point.positions
#         return {"command":"move", "prm":{"path": "joint", "movement": DornaMovement.GLOBAL, "speed": DornaMovement.DEFAULT_SPEED, "j0": int(pos[0]*180./np.pi), "j1": int(pos[1]*180./np.pi), "j2": int(pos[2]*180./np.pi), "j3": int(pos[3]*180./np.pi), "j4": int(pos[4]*180./np.pi)}}