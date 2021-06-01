import time

import sim  # Elementos de Coppleia

import numpy as np  # array library
from constants import *


def defineMotors(clientID):
    errorCode, left_motor = sim.simxGetObjectHandle(
        clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
    errorCode2, right_motor = sim.simxGetObjectHandle(
        clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)
    if(errorCode == 0 and errorCode2 == 0):
        return left_motor, right_motor
    else:
        return False


def defineVisionSensors(clientID):

    errorCode1, vision_center = sim.simxGetObjectHandle(
        clientID, 'Vision_center', sim.simx_opmode_blocking)
    errorCode2, vision_left = sim.simxGetObjectHandle(
        clientID, 'Vision_left', sim.simx_opmode_blocking)
    errorCode3, vision_right = sim.simxGetObjectHandle(
        clientID, 'Vision_right', sim.simx_opmode_blocking)
    if (errorCode1 == 0 and errorCode2 == 0 and errorCode3 == 0):
        returnCode1, detectionState, auxPackets = sim.simxReadVisionSensor(
            clientID, vision_center, sim.simx_opmode_streaming)
        returnCode2, detectionState, auxPackets = sim.simxReadVisionSensor(
            clientID, vision_left, sim.simx_opmode_streaming)
        returnCode3, detectionState, auxPackets = sim.simxReadVisionSensor(
            clientID, vision_right, sim.simx_opmode_streaming)
        return vision_left, vision_center, vision_right
    else:
        return False


def readVisionSensors(clientID,vision_left, vision_center, vision_right):
    errorCode1, detectionState, auxPackets_l = sim.simxReadVisionSensor(
        clientID, vision_left, sim.simx_opmode_buffer)
    errorCode2, detectionState, auxPackets_c = sim.simxReadVisionSensor(
        clientID, vision_center, sim.simx_opmode_buffer)
    errorCode3, detectionState, auxPackets_r = sim.simxReadVisionSensor(
        clientID, vision_right, sim.simx_opmode_buffer)
    if errorCode1 == 0 and errorCode2 == 0 and errorCode3 == 0:
        return auxPackets_l[0][11], auxPackets_c[0][11], auxPackets_r[0][11]
    else:
        return False


def defineSensors(clientID):
    errorCode1, sensor_left = sim.simxGetObjectHandle(clientID, 'Proximity_left',
                                                      sim.simx_opmode_blocking)
    errorCode2, sensor_front = sim.simxGetObjectHandle(clientID, 'Proximity_front',
                                                       sim.simx_opmode_blocking)

    errorCode3, sensor_front2 = sim.simxGetObjectHandle(clientID, 'Proximity_front2',
                                                        sim.simx_opmode_blocking)

    errorCode4, sensor_right = sim.simxGetObjectHandle(clientID, 'Proximity_right',
                                                       sim.simx_opmode_blocking)

    if (errorCode1 == 0 and errorCode2 == 0 and errorCode3 == 0 and errorCode4 == 0):
        returnCode, detectionState_l, detectedPoint_l, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
            clientID, sensor_left, sim.simx_opmode_streaming)

        returnCode, detectionState_f, detectedPoint_f, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
            clientID, sensor_front, sim.simx_opmode_streaming)
        returnCode, detectionState_f2, detectedPoint_f, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
            clientID, sensor_front2, sim.simx_opmode_streaming)

        returnCode, detectionState_r, detectedPoint_r, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
            clientID, sensor_right, sim.simx_opmode_streaming)

        return sensor_left, sensor_front, sensor_front2, sensor_right
    else:
        return 0




def readSensors(clientID,l, f, f2, r):

    errorCode1, detectionState_l, detectedPoint_l, detectedObjectHandle_l, detectedSurfaceNormalVector_l = sim.simxReadProximitySensor(
        clientID, l, sim.simx_opmode_buffer)

    errorCode2, detectionState_f, detectedPoint_f, detectedObjectHandle_f, detectedSurfaceNormalVector_f = sim.simxReadProximitySensor(
        clientID, f, sim.simx_opmode_buffer)
    errorCode4, detectionState_f2, detectedPoint_f2, detectedObjectHandle_f, detectedSurfaceNormalVector_f = sim.simxReadProximitySensor(
        clientID, f2, sim.simx_opmode_buffer)

    errorCode3, detectionState_r, detectedPoint_r, detectedObjectHandle_r, detectedSurfaceNormalVector_r = sim.simxReadProximitySensor(
        clientID, r, sim.simx_opmode_buffer)

    if (errorCode1 == 0 and errorCode2 == 0 and errorCode3 == 0 and errorCode4 == 0):
        if not detectionState_l:
            detectedPoint_l = 0
        if not detectionState_f:
            detectedPoint_f = 0
        if not detectionState_f2:
            detectedPoint_f2 = 0
        if not detectionState_r:
            detectedPoint_r = 0
        # print("-")
        # print("left: "+str(detectionState_l) + " "+ str(detectedPoint_l))
        # print("front1: "+ str(detectionState_f) + " "+  str(detectedPoint_f))
        # print("front2: "+ str(detectionState_f2) + " "+  str(detectedPoint_f2))
        # print("right: "+ str(detectionState_r) + " "+  str(detectedPoint_r))
        # print("-")

        # return detectionState_l, detectionState_f, detectionState_f2, detectionState_r
        return np.linalg.norm(detectedPoint_l), np.linalg.norm(detectedPoint_f), np.linalg.norm(detectedPoint_f2), np.linalg.norm(detectedPoint_r)
    else:
        return False


def showDistance(dl, df, df2, dr):
    print("")
    print(
        f" ---left: {dl:.3f} ---front left: {df2:.3f} ---front right: {df:.3f}  ---right: {dr:.3f} ---")
    print("")


def showVisionRead(r_l, r_c, r_r):
    print("")
    print(f"---left: {r_l:.3f} ---center: {r_c:.3f} ---right: {r_r:.3f} ---")
    print("")


def moveForward(clientID, left_motor, right_motor, speed):
    print("moving forward")
    sim.simxSetJointTargetVelocity(
        clientID, left_motor, speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, right_motor, speed, sim.simx_opmode_oneshot)


def moveBackward(clientID, left_motor, right_motor, speed):
    print("moving backwards")
    sim.simxSetJointTargetVelocity(
        clientID, left_motor, -1*speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, right_motor, -1*speed, sim.simx_opmode_oneshot)


def moveRight(clientID, left_motor, right_motor, speed):
    print("turning right")

    sim.simxSetJointTargetVelocity(
        clientID, left_motor, speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, right_motor, -1*speed, sim.simx_opmode_oneshot)


def moveRightSoft(clientID, left_motor, right_motor, speed_left, speed_right):
    print("turning right softly")
    if speed_right >= speed_left:
        speed_right = 0
    sim.simxSetJointTargetVelocity(
        clientID, left_motor, speed_left, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, right_motor, speed_right, sim.simx_opmode_oneshot)


def moveLeft(clientID, left_motor, right_motor, speed):
    print("turning left")
    sim.simxSetJointTargetVelocity(
        clientID, left_motor, -1*speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, right_motor, speed, sim.simx_opmode_oneshot)


def moveLeftSoft(clientID, left_motor, right_motor, speed_left, speed_right):
    print("turning left softly")
    if speed_right <= speed_left:
        speed_right = 0
    sim.simxSetJointTargetVelocity(
        clientID, left_motor, speed_left, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, right_motor, speed_right, sim.simx_opmode_oneshot)


def motorStop(clientID, left_motor, right_motor):
    sim.simxSetJointTargetVelocity(
        clientID, left_motor, 0, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, right_motor, 0, sim.simx_opmode_oneshot)


def avoidObstacleAtLeft(clientID, left_motor, right_motor, proximity_l, proximity_f, proximity_f2, proximity_r):
    print("avoiding problem at left")
    distance_l, distance_f, distance_f2, distance_r = readSensors(clientID,
        proximity_l, proximity_f, proximity_f2, proximity_r)
    # print(distance_l,distance_f,distance_r)
    # moveRight(clientID, left_motor=left_motor, right_motor=right_motor, speed=MOTOR_SPEED)
    # time.sleep(5*LOOP_DELAY)
    if(distance_f):
        print("etapa 1")
        print(distance_f, distance_f2, distance_l)
        moveRight(clientID, left_motor=left_motor,
                  right_motor=right_motor, speed=3*MOTOR_SPEED)

    if(distance_l and distance_f):
        print("etapa 2")
        print(distance_f, distance_l)
        moveForward(clientID, left_motor=left_motor,
                    right_motor=right_motor, speed=4*MOTOR_SPEED)

    if(distance_l and not distance_f):
        print("etapa 3")
        # moveLeft(clientID, left_motor=left_motor, right_motor=right_motor, speed=0.4*MOTOR_SPEED)
        moveLeftSoft(clientID, left_motor=left_motor,
                     right_motor=right_motor, speed_left=1*MOTOR_SPEED, speed_right=4*MOTOR_SPEED)

    time.sleep(5*LOOP_DELAY)


def avoidObstacleAtRight(clientID, left_motor, right_motor, proximity_l, proximity_f, proximity_f2, proximity_r):
    print("avoiding problem at right")
    distance_l, distance_f, distance_f2, distance_r = readSensors(clientID,
        proximity_l, proximity_f, proximity_f2, proximity_r)
    # print(distance_l,distance_f,distance_r)
    # moveRight(clientID, left_motor=left_motor, right_motor=right_motor, speed=MOTOR_SPEED)
    # time.sleep(5*LOOP_DELAY)
    if(distance_f2):
        print("etapa 1")
        print(distance_f, distance_f2, distance_l)
        moveLeft(clientID, left_motor=left_motor,
                 right_motor=right_motor, speed=3*MOTOR_SPEED)

    if(distance_r and distance_f2):
        print("etapa 2")
        print(distance_f, distance_l)
        moveForward(clientID, left_motor=left_motor,
                    right_motor=right_motor, speed=4*MOTOR_SPEED)

    if(distance_r and not distance_f2):
        print("etapa 3")
        # moveLeft(clientID, left_motor=left_motor, right_motor=right_motor, speed=0.4*MOTOR_SPEED)
        moveRightSoft(clientID, left_motor=left_motor,
                      right_motor=right_motor, speed_left=4*MOTOR_SPEED, speed_right=1*MOTOR_SPEED)

    time.sleep(5*LOOP_DELAY)


def firstSequence(clientID, vision_left, vision_center, vision_right, proximity_l, proximity_f, proximity_f2, proximity_r, left_motor, right_motor):

    speedCounter = 2
    blueMark = False
    avoiding = False
    avoidingObstacle = ''
    firstWay = True
    while (firstWay):
        r_left, r_center, r_right = readVisionSensors(clientID,
            vision_left, vision_center, vision_right)
        # showVisionRead(r_left, r_center, r_right)
        distance_l, distance_f, distance_f2, distance_r = readSensors(clientID,
            proximity_l, proximity_f, proximity_f2, proximity_r)
        showDistance(distance_l, distance_f, distance_f2, distance_r)
        if(r_center > COLOR_GREEN_DOWN and r_center < COLOR_GREEN_UP):
            print("color green")
            moveForward(clientID, left_motor=left_motor,
                        right_motor=right_motor, speed=2*MOTOR_SPEED)
        if distance_f or distance_f2:
            avoiding = True
        if(r_center > COLOR_BLUE_MARK_DOWN and r_center < COLOR_BLUE_MARK_UP):
            print("color blue mark")
            blueMark = True
        # if(distance_f):
        #     motorStop(clientID, left_motor=left_motor, right_motor=right_motor)
        #     firstWay = False
        if(not avoiding):

            if(r_left > COLOR_WHITE and r_center < COLOR_BLACK and r_right > COLOR_WHITE):
                moveForward(clientID, left_motor=left_motor,
                            right_motor=right_motor, speed=speedCounter*MOTOR_SPEED)
                speedCounter += 0.3
            else:
                speedCounter = 2 + speedCounter/4
                if(r_left < COLOR_BLACK and r_center < COLOR_BLACK and r_right > COLOR_WHITE):
                    moveLeftSoft(clientID, left_motor=left_motor,
                                 right_motor=right_motor, speed_left=1*MOTOR_SPEED, speed_right=4*MOTOR_SPEED)

                if(r_left > COLOR_WHITE and r_center < COLOR_BLACK and r_right < COLOR_BLACK):
                    moveRightSoft(clientID, left_motor=left_motor,
                                  right_motor=right_motor, speed_left=4*MOTOR_SPEED, speed_right=1*MOTOR_SPEED)

                if(r_left < COLOR_BLACK and r_center > COLOR_WHITE and r_right > COLOR_WHITE):

                    moveLeft(clientID, left_motor=left_motor,
                             right_motor=right_motor, speed=MOTOR_SPEED)
                if(r_left > COLOR_WHITE and r_center > COLOR_WHITE and r_right < COLOR_BLACK):
                    moveRight(clientID, left_motor=left_motor,
                              right_motor=right_motor, speed=MOTOR_SPEED)

                if(r_left > COLOR_WHITE and r_center > COLOR_WHITE and r_right > COLOR_WHITE):
                    moveBackward(clientID, left_motor=left_motor,
                                 right_motor=right_motor, speed=3*MOTOR_SPEED)
                    # motorStop(clientID, left_motor=left_motor, right_motor=right_motor)
                    time.sleep(2*LOOP_DELAY)
                if(r_center > COLOR_BLUE_DOWN and r_center < COLOR_BLUE_UP):
                    print("color blue")
                    firstWay = False
                    motorStop(clientID, left_motor=left_motor,
                              right_motor=right_motor)
            time.sleep(LOOP_DELAY)

        if avoiding and not blueMark:

            if distance_f or avoidingObstacle == 'left':
                avoidingObstacle = 'left'
                avoidObstacleAtLeft(
                    clientID, left_motor, right_motor, proximity_l, proximity_f, proximity_f2, proximity_r)
                r_left, r_center, r_right = readVisionSensors(clientID,
                    vision_left, vision_center, vision_right)
                if r_right < COLOR_BLACK:
                    while avoiding:

                        # if not r_center < COLOR_BLACK:

                        moveForward(clientID, left_motor=left_motor,
                                    right_motor=right_motor, speed=MOTOR_SPEED)
                        time.sleep(LOOP_DELAY)

                        moveRight(clientID, left_motor=left_motor,
                                  right_motor=right_motor, speed=1*MOTOR_SPEED)

                        time.sleep(LOOP_DELAY)
                        r_left, r_center, r_right = readVisionSensors(clientID,
                            vision_left, vision_center, vision_right)
                        if r_left > COLOR_BLACK and r_center < COLOR_BLACK and r_right < COLOR_WHITE:
                            avoiding = False

            if(distance_f2 and not distance_f or avoidingObstacle == 'right'):
                avoidingObstacle = 'right'
                avoidObstacleAtRight(
                    clientID, left_motor, right_motor, proximity_l, proximity_f, proximity_f2, proximity_r)
                r_left, r_center, r_right = readVisionSensors(clientID,
                    vision_left, vision_center, vision_right)
                if r_left < COLOR_BLACK:
                    while avoiding:
                        moveForward(clientID, left_motor=left_motor,
                                    right_motor=right_motor, speed=MOTOR_SPEED)
                        time.sleep(LOOP_DELAY)
                        moveLeft(clientID, left_motor=left_motor,
                                 right_motor=right_motor, speed=1*MOTOR_SPEED)
                        time.sleep(LOOP_DELAY)

                        r_left, r_center, r_right = readVisionSensors(clientID,
                            vision_left, vision_center, vision_right)
                        if r_left > COLOR_WHITE and r_center < COLOR_BLACK and r_right < COLOR_BLACK:
                            avoiding = False
            if(r_center > COLOR_BLUE_DOWN and r_center < COLOR_BLUE_UP):
                print("color blue")
                avoiding = False
                motorStop(clientID, left_motor=left_motor,
                          right_motor=right_motor)
            distance_l, distance_f, distance_f2, distance_r = readSensors(clientID,
                proximity_l, proximity_f, proximity_f2, proximity_r)
            # showDistance(distance_l,distance_f,distance_f2,distance_r)
            time.sleep(LOOP_DELAY)
            print("avoiding: " + str(avoiding) + ' ' + avoidingObstacle)

        if blueMark:
            print("reaching the objective")
            if distance_f < TARGET_DISTANCE_F and distance_f > 0 and distance_f2 < TARGET_DISTANCE_F2 and distance_f2 > 0:

                print("la cerda esta en la pocilga")

                firstWay = False
            else:
                moveForward(clientID, left_motor=left_motor,
                            right_motor=right_motor, speed=3*MOTOR_SPEED)
            time.sleep(LOOP_DELAY/10)
