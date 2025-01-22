import robotic as ry
import numpy as np
import time
import easy_calib

print('version:', ry.__version__, ry.compiled())







C = ry.Config()
# C.addFile(ry.raiPath('scenarios/pandaDual.g'))
# print(ry.raiPath('scenarios/pandaDual.g'))
C.addFile('pandaDual.g')
# print(C.getFrame("l_panda_base").getPosition()) 
# print(C.getFrame())
# C.view(True)
pcl = C.addFrame('pcl', 'cameraWrist')


## get obs positon ###
# obj_pose = easy_calib.calib()
# obj_position = obj_pose[:3,3]
# l_panda_base= C.getFrame("l_panda_base").getPosition()
# obj_real_postion = np.zeros_like(obj_position)
# obj_real_postion[:2] = l_panda_base[:2]-obj_position[:2]
# obj_real_postion[2] = l_panda_base[2] + obj_position[2]
# print("obj_real_postion:  ", obj_real_postion)

###################################

# C.addFrame('tube') \
#     .setPosition([.05,-.1, 1]) \
#     .setShape(ry.ST.ssBox, size=[.47,.025,.204,.005]) \
#     .setColor([1,.5,0]) \
#     .setContact(0)

# C.addFrame('goal') \
#     .setPosition([0.,.1, .9]) \
#     .setQuaternion([.991, 0, 0, -.131]) \
#     .setShape(ry.ST.ssBox, size=[.47,.025,.204,.005]) \
#     .setColor([1,.7,0]) \
#     .setContact(0)


C.addFrame('tube') \
    .setPosition([0.,.0, .9]) \
    .setShape(ry.ST.ssBox, size=[.47,.025,.204,.005]) \
    .setColor([1,.5,0]) \
    .setContact(1)

C.addFrame('goal') \
    .setPosition([0.,.1, 1.1]) \
    .setQuaternion([.991, 0, 0, -.131]) \
    .setShape(ry.ST.ssBox, size=[.47,.025,.204,.005]) \
    .setColor([1,.7,0]) \
    .setContact(1)

## obj_real_postion:   [ 0.02584938 -0.04827474  0.81677391]

# C.addFrame('obs') \
#     .setPosition([0.02584938, -0.04827474 , 0.81677391]) \
#     .setShape(ry.ST.ssBox, size=[.76,.027,.35,.005]) \
#     .setColor([.1,.5,0]) \
#     .setContact(0)


# C.addFrame('goal2') \
#     .setPosition([0.,.3, 1.3]) \
#     .setQuaternion([.1, 0, 0, .0]) \
#     .setShape(ry.ST.ssBox, size=[.9,.06,.06,.005]) \
#     .setColor([1,.7,0]) \
#     .setContact(1)


C.view(True, 'this is your workspace data structure C -- NOT THE SIMULTATION')



# while True:
#     breakpoint


# True = real robot!!
bot = ry.BotOp(C, useRealRobot=True)

bot.sync(C)
bot.moveTo(bot.get_qHome())
bot.gripperMove(ry._left, width=.08, speed=.1)
bot.gripperMove(ry._right, width=.08, speed=.1)
bot.wait(C)
l_gripper = C.getFrame("l_gripper")
r_gripper = C.getFrame("r_gripper")
tube = C.getFrame("tube")
l= tube.getSize()[0]
print("l: ",l )
# print(C.getJointState())

# ###### rotation along z -15 ###############
# komo = ry.KOMO(C, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=False)
# komo.addControlObjective([], 0, 1e-1)
# komo.addControlObjective([], 1, 1e0)
# komo.addObjective([1], ry.FS.positionRel, ['r_gripper', 'tube'], ry.OT.eq, [1e1], target=[l/2,0,0])
# komo.addObjective([1], ry.FS.positionRel, ['l_gripper', 'tube'], ry.OT.eq, [1e1], target=[-l/2,0,0])
# komo.addObjective([1], ry.FS.scalarProductXX, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])
# komo.addObjective([1], ry.FS.scalarProductXZ, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])
# komo.addObjective([1], ry.FS.scalarProductZZ, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])

# komo.addObjective([1], ry.FS.scalarProductXX, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
# komo.addObjective([1], ry.FS.scalarProductXZ, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
# komo.addObjective([1], ry.FS.scalarProductZZ, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
# komo.addObjective([1], ry.FS.scalarProductXZ, ['tube', 'l_gripper'], ry.OT.eq, [1e1], [-1])
# ret = ry.NLP_Solver(komo.nlp(), verbose=0) .solve()

# q = komo.getPath()
# print('size of path:', q.shape)
# # print("q-1", q[-1])
# q0 = q[-1]
# q0[7] = 0.04
# print(q0)
# # bot.moveTo(q[-1])
# # bot.wait(C)
# # bot.gripperMove(ry._left, width=.0, speed=.1)
# # bot.gripperMove(ry._right, width=.0, speed=.1)
# # while not (bot.gripperDone(ry._left) and bot.gripperDone(ry._right)):
# #     bot.sync(C)
# # print("l-r ",l_gripper.getPosition()-r_gripper.getPosition())

# komo = ry.KOMO(C, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=False)
# komo.addControlObjective([], 0, 1e-1)
# komo.addControlObjective([], 1, 1e0)
# komo.addObjective([1], ry.FS.positionRel, ['r_gripper', 'goal'], ry.OT.eq, [1e1], target=[l/2,0,0])
# komo.addObjective([1], ry.FS.positionRel, ['l_gripper', 'goal'], ry.OT.eq, [1e1], target=[-l/2,0,0])
# komo.addObjective([1], ry.FS.scalarProductXX, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])
# komo.addObjective([1], ry.FS.scalarProductXZ, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])
# komo.addObjective([1], ry.FS.scalarProductZZ, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])

# komo.addObjective([1], ry.FS.scalarProductXX, ['l_gripper', 'goal'], ry.OT.eq, [1e1], [0])
# komo.addObjective([1], ry.FS.scalarProductXZ, ['l_gripper', 'goal'], ry.OT.eq, [1e1], [0])
# komo.addObjective([1], ry.FS.scalarProductZZ, ['l_gripper', 'goal'], ry.OT.eq, [1e1], [0])
# komo.addObjective([1], ry.FS.scalarProductXZ, ['goal', 'l_gripper'], ry.OT.eq, [1e1], [-1])
# ret = ry.NLP_Solver(komo.nlp(), verbose=0) .solve()
# q = komo.getPath()
# q1 = q[-1]



# bot.moveTo(q0)
# bot.wait(C)
# bot.moveTo(q1)
# bot.wait(C)

###########################################################333


###################ratation by given constraint between two ee ###############################

komo = ry.KOMO(C, phases=1, slicesPerPhase=80, kOrder=1, enableCollisions=False)
komo.addControlObjective([], 0, 1e-1)
komo.addControlObjective([], 1, 1e0)
komo.addObjective([.3], ry.FS.positionRel, ['r_gripper', 'tube'], ry.OT.eq, [1e1], target=[l/2,0,0])
komo.addObjective([.3], ry.FS.positionRel, ['l_gripper', 'tube'], ry.OT.eq, [1e1], target=[-l/2,0,0])
komo.addObjective([.3], ry.FS.scalarProductXX, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([.3], ry.FS.scalarProductXZ, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([.3], ry.FS.scalarProductZZ, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])

komo.addObjective([.3], ry.FS.scalarProductXX, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([.3], ry.FS.scalarProductXZ, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([.3], ry.FS.scalarProductZZ, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([.3], ry.FS.scalarProductXZ, ['tube', 'l_gripper'], ry.OT.eq, [1e1], [-1])

komo.addObjective([1], ry.FS.positionRel, ['r_gripper', 'goal'], ry.OT.eq, [1e1], target=[l/2,0,0])
komo.addObjective([1], ry.FS.scalarProductXX, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXZ, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductZZ, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])
# komo.addObjective([1], ry.FS.scalarProductYY, ['l_gripper', 'r_gripper'], ry.OT.eq, [1e1], [-1])
# komo.addObjective([1], ry.FS.scalarProductZZ, ['l_gripper', 'r_gripper'], ry.OT.eq, [1e1], [-1])
# komo.addObjective([1], ry.FS.positionRel, ['l_gripper', 'r_gripper'], ry.OT.eq, [1e1], [0,0,-l])
# using poseRel is better 
komo.addObjective([1], ry.FS.poseRel, ['l_gripper', 'r_gripper'], ry.OT.eq, [1e1], [0,0,-l,0,1,0,0])


ret = ry.NLP_Solver(komo.nlp(), verbose=0) .solve()
q = komo.getPath()


########################################################################3


# be careful about the max velocity!!!
q0= q[0:24]
q1= [-0.989721, 0.460272, -0.31306, -1.45885, -2.75816, 2.80719, 0.55961, 0.477026, 0.878017, -0.13644, -0.902709, 1.87192, 2.32057, -1.06615]

bot.moveAutoTimed(q0, .3, .2) # path, max vel, max acc
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)


# close the gripper #
bot.gripperMove(ry._left, width=.0, speed=.1)
bot.gripperMove(ry._right, width=.0, speed=.1)
while not (bot.gripperDone(ry._left) and bot.gripperDone(ry._right)):
    bot.sync(C)


bot.moveTo(q1)
bot.wait(C)


# bot.moveAutoTimed(q1, .1, .1) # path, max vel, max acc
# while bot.getTimeToEnd()>0:
#     bot.sync(C, .1)



## move to home at the end ##

# bot.sync(C)
# bot.moveTo(bot.get_qHome())
# bot.gripperMove(ry._left, width=.08, speed=.1)
# bot.gripperMove(ry._right, width=.08, speed=.1)
# bot.wait(C)



