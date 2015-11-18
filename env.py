__author__ = 'fyt'

import vrep, math

# get handler of all concret blocks
def getConcretBlockHandle(clientID, nrBlock=12):
    blockHandleArray = [[] for ix in range(nrBlock)] # initialize a container for all the blocks
    for i_block in range(nrBlock):
       blockHandleArray[i_block] = vrep.simxGetObjectHandle(clientID, 'ConcretBlock#'+str(i_block), vrep.simx_opmode_oneshot_wait) # get object by name
    return blockHandleArray

# assign random location to some of the blocks
def initConcretBlockPosition(clientID, blockHandleArray):
    import random
    for i_block in range(7,12):
        rand_loc = [random.random()*6-1.5, random.random()*7-2.5, 0.0537] # x[-1.5,4.5] y[-2.5,-4.5]
        vrep.simxSetObjectPosition(clientID, blockHandleArray[i_block][1], -1, rand_loc, vrep.simx_opmode_oneshot)

# get nearest block's distances
def getNearestConcretBlockDist(clientID, blockHandleArray, pioneerRobotHandle):

    distances = [0. for ix in range(len(blockHandleArray))] # initialize a container
    pioneer2BlockPos = [[] for ix in range(len(blockHandleArray))] # initialize a container
    for i_block in range(len(blockHandleArray)):

        pioneer2BlockPos[i_block] = vrep.simxGetObjectPosition(clientID, pioneerRobotHandle, blockHandleArray[i_block][1], vrep.simx_opmode_oneshot_wait) # get relative position to robot
        distances[i_block] = math.sqrt(pioneer2BlockPos[i_block][1][0]**2 + pioneer2BlockPos[i_block][1][1]**2) # compute Euclidean distance
        #print(i_block, distances[i_block])

    return min(distances), distances.index(min(distances)), pioneer2BlockPos[distances.index(min(distances))]

# Collect if any block is nearby
def collectNearestBlock(clientID, blockHandleArray, pioneerRobotHandle):

    dist2block, id, pioneer2BlockPos = getNearestConcretBlockDist(clientID, blockHandleArray, pioneerRobotHandle)
    if dist2block <= 0.35:
        vrep.simxSetObjectPosition(clientID, blockHandleArray[id][1], -1, [0,0,-2], vrep.simx_opmode_oneshot) # removes the red block
        print('Collect succeed ;)')
        blockHandleArray.remove(blockHandleArray[id])
        return True

    else:
        print('No blocks nearby')
        return False