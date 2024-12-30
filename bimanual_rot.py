import robotic as ry
import numpy as np
import time
print('version:', ry.__version__, ry.compiled())

C = ry.Config()
# C.addFile(ry.raiPath('scenarios/pandaDual.g'))
# print(ry.raiPath('scenarios/pandaDual.g'))
C.addFile('pandaDual.g')
# print(C.getFrame("l_panda_base").getPosition()) 
# print(C.getFrame())
# C.view(True)
pcl = C.addFrame('pcl', 'cameraWrist')

C.addFrame('tube') \
    .setPosition([0.,.1, .9]) \
    .setShape(ry.ST.ssBox, size=[.43,.06,.06,.005]) \
    .setColor([1,.5,0]) \
    .setContact(1)

C.addFrame('goal') \
    .setPosition([0.,.1, 1.1]) \
    .setQuaternion([.991, 0, 0, -.131]) \
    .setShape(ry.ST.ssBox, size=[.43,.06,.06,.005]) \
    .setColor([1,.7,0]) \
    .setContact(1)

# C.addFrame('goal2') \
#     .setPosition([0.,.3, 1.3]) \
#     .setQuaternion([.1, 0, 0, .0]) \
#     .setShape(ry.ST.ssBox, size=[.9,.06,.06,.005]) \
#     .setColor([1,.7,0]) \
#     .setContact(1)


C.view(True, 'this is your workspace data structure C -- NOT THE SIMULTATION')

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
# print(C.getJointState())

# ###### rotation along z -15 ###############
komo = ry.KOMO(C, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=False)
komo.addControlObjective([], 0, 1e-1)
komo.addControlObjective([], 1, 1e0)
komo.addObjective([1], ry.FS.positionRel, ['r_gripper', 'tube'], ry.OT.eq, [1e1], target=[l/2,0,0])
komo.addObjective([1], ry.FS.positionRel, ['l_gripper', 'tube'], ry.OT.eq, [1e1], target=[-l/2,0,0])
komo.addObjective([1], ry.FS.scalarProductXX, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXZ, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductZZ, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])

komo.addObjective([1], ry.FS.scalarProductXX, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXZ, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductZZ, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXZ, ['tube', 'l_gripper'], ry.OT.eq, [1e1], [-1])
ret = ry.NLP_Solver(komo.nlp(), verbose=0) .solve()

q = komo.getPath()
print('size of path:', q.shape)
# print("q-1", q[-1])
q0 = q[-1]
q0[7] = 0.04
print(q0)
# bot.moveTo(q[-1])
# bot.wait(C)
# bot.gripperMove(ry._left, width=.0, speed=.1)
# bot.gripperMove(ry._right, width=.0, speed=.1)
# while not (bot.gripperDone(ry._left) and bot.gripperDone(ry._right)):
#     bot.sync(C)
# print("l-r ",l_gripper.getPosition()-r_gripper.getPosition())

komo = ry.KOMO(C, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=False)
komo.addControlObjective([], 0, 1e-1)
komo.addControlObjective([], 1, 1e0)
komo.addObjective([1], ry.FS.positionRel, ['r_gripper', 'goal'], ry.OT.eq, [1e1], target=[l/2,0,0])
komo.addObjective([1], ry.FS.positionRel, ['l_gripper', 'goal'], ry.OT.eq, [1e1], target=[-l/2,0,0])
komo.addObjective([1], ry.FS.scalarProductXX, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXZ, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductZZ, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])

komo.addObjective([1], ry.FS.scalarProductXX, ['l_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXZ, ['l_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductZZ, ['l_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXZ, ['goal', 'l_gripper'], ry.OT.eq, [1e1], [-1])
ret = ry.NLP_Solver(komo.nlp(), verbose=0) .solve()
q = komo.getPath()
q1 = q[-1]



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
q1= q[24:]

bot.moveAutoTimed(q0, .2, .2) # path, max vel, max acc
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)


bot.gripperMove(ry._left, width=.0, speed=.1)
bot.gripperMove(ry._right, width=.0, speed=.1)
while not (bot.gripperDone(ry._left) and bot.gripperDone(ry._right)):
    bot.sync(C)

bot.moveAutoTimed(q1, .2, .2) # path, max vel, max acc
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)



