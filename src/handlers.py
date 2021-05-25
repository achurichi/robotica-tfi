import sim
from constants import *
from kinematics import *


def joint_handlers(joints, robot, num, clientID, mode):
    selected_robot = '' if robot == 'R1' else '_2'
    for i in range(num):
        handler_name = ROBOT_NAME + '_joint_' + str(i + 1) + selected_robot
        _, joints[i] = sim.simxGetObjectHandle(clientID, handler_name, mode)


def checkpoint_handlers(checkpoints, prefix, num, clientID, mode):
    for i in range(num):
        handler_name = prefix + '_' + str(i)
        _, checkpoints[i]['handle'] = sim.simxGetObjectHandle(clientID,
                                                              handler_name,
                                                              mode)


def checkpoints_set_streaming(checkpoints_list, clientID):
    for checkpoints_group in checkpoints_list:
        for checkpoint in checkpoints_group:
            sim.simxGetObjectPosition(clientID,
                                      checkpoint['handle'],
                                      -1,
                                      sim.simx_opmode_streaming)
            sim.simxGetObjectOrientation(clientID,
                                         checkpoint['handle'],
                                         -1,
                                         sim.simx_opmode_streaming)


def checkpoints_get_from_buffer(checkpoints_list, robot, clientID):
    for checkpoints_group in checkpoints_list:
        for checkpoint in checkpoints_group:
            sim.simxGetObjectPosition(
                clientID, checkpoint['handle'], -1, sim.simx_opmode_streaming)
            sim.simxGetObjectOrientation(
                clientID, checkpoint['handle'], -1, sim.simx_opmode_streaming)
            _, checkpoint['position'] = sim.simxGetObjectPosition(clientID,
                                                                  checkpoint['handle'],
                                                                  -1,
                                                                  sim.simx_opmode_buffer)
            _, checkpoint['orientation'] = sim.simxGetObjectOrientation(clientID,
                                                                        checkpoint['handle'],
                                                                        -1,
                                                                        sim.simx_opmode_buffer)
            checkpoint['angles'] = inverse_kinematics(checkpoint['position'],
                                                      checkpoint['orientation'],
                                                      robot)


def setJointsPosition(joints_pos, joints, num, clientID, mode):
    for i in range(num):
        sim.simxSetJointPosition(clientID, joints[i], joints_pos[i], mode)


def getObjectsPosition(joints_pos, joints, num, clientID, mode):
    for i in range(num):
        _, joints_pos[i] = sim.simxGetObjectPosition(
            clientID, joints[i], -1, mode)


def getObjectsOrientation(joints_orient, joints, num, clientID, mode):
    for i in range(num):
        _, joints_orient[i] = sim.simxGetObjectOrientation(
            clientID, joints[i], -1, mode)
