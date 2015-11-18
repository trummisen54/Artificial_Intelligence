# Make sure to have the server side running in V-REP:
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simExtRemoteApiStart(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

import vrep, math, time, env, random

####################################################
#DEFINITIONS
####################################################
overallimit = 0.6

HIGH_SPEED = 2
MID_SPEED = 1
SLOW_SPEED = 0.4
RUN_FRONT_SPEED = 2

RUN_FRONT_TIME = 0.3
MID_TIME = 1
SHORT_TIME = 0.01


stuck_counter_limit = 60
random_counter_limit = 100

recursion_counter = 0

hysteres = 0.1

MOTOR_RIGHT = 1;
MOTOR_LEFT = 2;
MOTOR_REVERSE = 3
MOTOR_FRONT = 4


# get distance to obstacle detected by a given sensor
def getObstacleDist(sensorHandler_):
    dist2Obstacle_LR = [0.0, 0.0]
    # Get raw sensor readings using API
    rawSR = vrep.simxReadProximitySensor(clientID, sensorHandler_, vrep.simx_opmode_oneshot_wait)
    #print(rawSR)
    # Calculate Euclidean distance
    if rawSR[1]: # if true, obstacle is within detection range, return distance to obstacle
        return math.sqrt(rawSR[2][0]*rawSR[2][0] + rawSR[2][1]*rawSR[2][1] + rawSR[2][2]*rawSR[2][2])
    else: # if false, obstacle out of detection range, return inf.
        return float('inf')



# execute agent's action
def execute(motorSpeed):
    vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, motorSpeed['speedLeft'], vrep.simx_opmode_oneshot )
    vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, motorSpeed['speedRight'], vrep.simx_opmode_oneshot )

def runPioneerToAngle(goto_angle):
    abs_angle_all = vrep.simxGetObjectOrientation(clientID, pioneerRobotHandle, -1, vrep.simx_opmode_oneshot)

    abs_angle = abs_angle_all[1][2]
    diff = goto_angle - abs_angle
    #diff = math.fabs(diff)

    if 0 <= diff and diff < math.pi :
        DIRECTION = MOTOR_LEFT
    elif math.pi <= diff :
        DIRECTION = MOTOR_RIGHT
    elif math.pi*(-1) <= diff and diff < 0 :
        DIRECTION = MOTOR_RIGHT
    elif diff < math.pi*(-1):
        DIRECTION = MOTOR_LEFT



    while abs_angle > (goto_angle + hysteres) or abs_angle < (goto_angle - hysteres):

        run_motor(DIRECTION, MID_SPEED, SHORT_TIME, 0)
        abs_angle_all = vrep.simxGetObjectOrientation(clientID, pioneerRobotHandle, -1, vrep.simx_opmode_oneshot)
        abs_angle = abs_angle_all[1][2]


def calculateRedBlockAngle(blocksArray):

    import math

    dist2NearestBlock, blockId, pioneer2BlockPos = env.getNearestConcretBlockDist(clientID, blocksArray, pioneerRobotHandle)


    #ret, Coordinates = vrep.simxGetObjectPosition(clientID, pioneerRobotHandle, blockId, vrep.simx_opmode_oneshot)

    x = pioneer2BlockPos[1][0]
    y = pioneer2BlockPos[1][1]

    angle = math.atan2(y,x)

    print('angle to block: ', angle)
    print('block id: ', blockId)


    angle = convertAngle(angle)



    if angle < 0:
        angle = angle + math.pi

    else:
        angle = angle - math.pi

    return angle



def avoidObstacles(distArray):

    index = -1;
    smallest = 100.0 #cant be bigger than this number


    for i in range(0, len(distArray), 1):
        if distArray[i][0] < smallest :

            if math.isinf(distArray[i][0]) == False :
                smallest = distArray[i][0]
                index = i


    if smallest < overallimit:
        runFromObstacle(index, distArray)

    if index == -1:
        return False # no obstacles found

    else :
        return True # did avoid obstacles






def runFromObstacle(sensorIndex, distArray):

    global recursion_counter

    abs_angle = vrep.simxGetObjectOrientation(clientID, pioneerRobotHandle, -1, vrep.simx_opmode_oneshot)

    if distArray[sensorIndex][1] == USS_LF_4:
        angle = convertAngle(abs_angle[1][2] - math.pi/2 - 0.1) #0.1 less than the turn in the other direction so the robot wont stuck.

    elif distArray[sensorIndex][1] == USS_LF_3:
        angle = convertAngle(abs_angle[1][2] - 2)

    elif distArray[sensorIndex][1] == USS_LF_2:
        angle = convertAngle(abs_angle[1][2] - 2.5)

    elif distArray[sensorIndex][1] == USS_LF_1:
        angle = convertAngle(abs_angle[1][2] - 2.5)

    elif distArray[sensorIndex][1] == USS_RF_1:
        angle = convertAngle(abs_angle[1][2] + 2.5)

    elif distArray[sensorIndex][1] == USS_RF_2:
        angle = convertAngle(abs_angle[1][2] + 2.5)

    elif distArray[sensorIndex][1] == USS_RF_3:
        angle = convertAngle(abs_angle[1][2] + 2)

    elif distArray[sensorIndex][1] == USS_RF_4:
        angle = convertAngle(abs_angle[1][2] + math.pi/2)



    runPioneerToAngle(angle)
    distArray = updateSensors()



    if avoidObstacles(distArray) == False :
        recursion_counter = 0
        run_motor(MOTOR_FRONT, MID_SPEED, RUN_FRONT_TIME, 0)

    else :
        recursion_counter = recursion_counter + 1
        if recursion_counter > 50 :
            recursion_counter = 0
            run_random()


def run_random() :

    import random


    abs_angle = vrep.simxGetObjectOrientation(clientID, pioneerRobotHandle, -1, vrep.simx_opmode_oneshot)

    random = random.random()

    if random < 0.95 :
        run_motor(MOTOR_FRONT, MID_SPEED, RUN_FRONT_TIME, 0)
    elif random < 0.975 :
        #RUN RIGHT
        angle = convertAngle(abs_angle[1][2] + math.pi/4)
        runPioneerToAngle(angle)
    elif random < 1 :
        #RUN LEFT
        angle = convertAngle(abs_angle[1][2] - math.pi/4)
        runPioneerToAngle(angle)




#returns the origin in format: -pi to pi
def convertAngle(angle):
    if angle < (-1)*math.pi:
        while angle < (-1)*math.pi:
            angle = angle + math.pi*2
    elif angle > math.pi:
        while angle > math.pi:
            angle = angle - math.pi*2

    return angle


def run_motor(LOCATION, speed, runtime, sleeptime):

    import time

    if LOCATION == MOTOR_RIGHT :
        motorSpeed = dict(speedLeft=speed, speedRight=(-1)*speed)
        execute(motorSpeed)
        time.sleep(runtime)

    elif LOCATION == MOTOR_LEFT :
        motorSpeed = dict(speedLeft=(-1)*speed, speedRight=speed)
        execute(motorSpeed)
        time.sleep(runtime)

    elif LOCATION == MOTOR_REVERSE :
        motorSpeed = dict(speedLeft=(-1)*speed, speedRight=(-1)*speed)
        execute(motorSpeed)
        time.sleep(runtime)

    elif LOCATION == MOTOR_FRONT :
        motorSpeed = dict(speedLeft=speed, speedRight=speed)
        execute(motorSpeed)
        time.sleep(runtime)


    motorSpeed = dict(speedLeft=0, speedRight=0)
    execute(motorSpeed)
    time.sleep(sleeptime)



def updateSensors():





    dist_LF_4 = getObstacleDist(USS_LF_4)
    dist_LF_3 = getObstacleDist(USS_LF_3)
    dist_LF_2 = getObstacleDist(USS_LF_2)
    dist_LF_1 = getObstacleDist(USS_LF_1)
    dist_RF_1 = getObstacleDist(USS_RF_1)
    dist_RF_2 = getObstacleDist(USS_RF_2)
    dist_RF_3 = getObstacleDist(USS_RF_3)
    dist_RF_4 = getObstacleDist(USS_RF_4)

    USS_LF_4_array = [dist_LF_4, USS_LF_4]
    USS_LF_3_array = [dist_LF_3, USS_LF_3]
    USS_LF_2_array = [dist_LF_2, USS_LF_2]
    USS_LF_1_array = [dist_LF_1, USS_LF_1]
    USS_RF_1_array = [dist_RF_1, USS_RF_1]
    USS_RF_2_array = [dist_RF_2, USS_RF_2]
    USS_RF_3_array = [dist_RF_3, USS_RF_3]
    USS_RF_4_array = [dist_RF_4, USS_RF_4]


    distArray = [USS_LF_4_array,
                 USS_LF_3_array,
                 USS_LF_2_array,
                 USS_LF_1_array,
                 USS_RF_1_array,
                 USS_RF_2_array,
                 USS_RF_3_array,
                 USS_RF_4_array]

    return distArray



print 'Program started'
vrep.simxFinish(-1) # just in case, close all opened connections

int_portNb = 19999 # define port_nr
clientID = vrep.simxStart( '127.0.0.1', int_portNb, True, True, 5000, 5) # connect to server
if clientID != -1:
    print 'Connected to remote API server'
    res,objs = vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_oneshot_wait) # get all objects in the scene
    if res == vrep.simx_return_ok: # Remote function call succeeded
        print 'Number of objects in the scene: ',len(objs)# print number of object in the scene

        ret_lm,  leftMotorHandle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
        ret_rm,  rightMotorHandle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
        ret_pr,  pioneerRobotHandle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait)



        #USS =  UltraSonicSensor

        #FRONT SENSORS
        ret_sl,  USS_LF_4 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(1),vrep.simx_opmode_oneshot_wait) #max left
        ret_sl,  USS_LF_3 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(2),vrep.simx_opmode_oneshot_wait)
        ret_sl,  USS_LF_2 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(3),vrep.simx_opmode_oneshot_wait)
        ret_sl,  USS_LF_1 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(4),vrep.simx_opmode_oneshot_wait) #centrum
        ret_sl,  USS_RF_1 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(5),vrep.simx_opmode_oneshot_wait)
        ret_sr,  USS_RF_2 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(6),vrep.simx_opmode_oneshot_wait)
        ret_sr,  USS_RF_3 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(7),vrep.simx_opmode_oneshot_wait)
        ret_sr,  USS_RF_4 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(8),vrep.simx_opmode_oneshot_wait) #max right

        #BACK SENSORS
        ret_sl,  USS_RB_4 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(9),vrep.simx_opmode_oneshot_wait) #max right
        ret_sl,  USS_RB_3 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(10),vrep.simx_opmode_oneshot_wait)
        ret_sl,  USS_RB_2 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(11),vrep.simx_opmode_oneshot_wait)
        ret_sl,  USS_RB_1 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(12),vrep.simx_opmode_oneshot_wait) #centrum
        ret_sl,  USS_LB_1 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(13),vrep.simx_opmode_oneshot_wait)
        ret_sr,  USS_LB_2 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(14),vrep.simx_opmode_oneshot_wait)
        ret_sr,  USS_LB_3 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(15),vrep.simx_opmode_oneshot_wait)
        ret_sr,  USS_LB_4 = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(16),vrep.simx_opmode_oneshot_wait) #max left



        updateSensors()

        blocksArray = env.getConcretBlockHandle(clientID)
        env.initConcretBlockPosition(clientID, blocksArray) # initialize position of the blocks


        stuck_counter = 0
        random_counter = 0


        while True: # main Control loop

            #######################################################
            # Perception Phase: Get information about environment #
            #######################################################

            # get distance to nearest block
            dist2NearestBlock, blockId, pioneer2BlockPos = env.getNearestConcretBlockDist(clientID, blocksArray, pioneerRobotHandle)

            ##############################################
            # Reasoning: figure out which action to take #
            ##############################################


            ret, arr = vrep.simxGetObjectOrientation(clientID, pioneerRobotHandle, blockId, vrep.simx_opmode_oneshot_wait)








            ##MOTOR

            distArray = updateSensors()
            avoidObstacles(distArray) #Because the prio one is to avoid obstacles, the program will be stuck here until the robot is in a safe distance from all nearby obstacles

            if stuck_counter > stuck_counter_limit :

                print('*** RANDOM ***')

                run_random()
                random_counter = random_counter + 1
                print('random_counter = ', random_counter)

                if random_counter > random_counter_limit:
                    stuck_counter = 0
                    random_counter = 0

            else :

                redBlockAngle = calculateRedBlockAngle(blocksArray)

                runPioneerToAngle(redBlockAngle)

                distArray = updateSensors()
                avoidObstacles(distArray)

                run_motor(MOTOR_FRONT, RUN_FRONT_SPEED, RUN_FRONT_TIME, 0)


                # collect nearest block within a short range
                if env.collectNearestBlock(clientID, blocksArray, pioneerRobotHandle) :
                    stuck_counter = 0;

                else :
                    stuck_counter = stuck_counter + 1
                    print('stuck_counter = ', stuck_counter)

    else:
        print 'Remote API function call returned with error code: ',res
    vrep.simxFinish(clientID) # close all opened connections
else:
    print 'Failed connecting to remote API server'
    print 'Program finished'


'''
if dist_LF_3 < left4_limit :
                run_motor(MOTOR_RIGHT, SLOW_SPEED, MID_TIME, 1)

            elif dist_RF_3 < right4_limit:
                run_motor(MOTOR_LEFT, SLOW_SPEED, MID_TIME, 1)

            elif dist_LF_1 < left1_limit :
                if random.random()*2 < 1:
                    run_motor(MOTOR_LEFT, SLOW_SPEED, MID_TIME, 1)

                else :
                    run_motor(MOTOR_RIGHT, SLOW_SPEED, MID_TIME, 1)

            else :
                run_motor(MOTOR_FRONT, MID_SPEED, RUN_FRONT_TIME, 0)
'''






