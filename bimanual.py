import robotic as ry
import numpy as np
import time
print('version:', ry.__version__, ry.compiled())

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaDual.g'))
print(ry.raiPath('scenarios/pandaDual.g'))
# C.addFile('pandaDual.g')
# print(C.getFrame("l_panda_base").getPosition()) 
# print(C.getFrame())
# C.view(True)
pcl = C.addFrame('pcl', 'cameraWrist')

C.addFrame('tube') \
    .setPosition([0.,.1, .9]) \
    .setShape(ry.ST.ssBox, size=[.9,.06,.06,.005]) \
    .setColor([1,.5,0]) \
    .setContact(1)

C.addFrame('goal') \
    .setPosition([0.,-.1, 1.1]) \
    .setQuaternion([.1, 0, 0, .0]) \
    .setShape(ry.ST.ssBox, size=[.9,.06,.06,.005]) \
    .setColor([1,.7,0]) \
    .setContact(1)

C.addFrame('goal2') \
    .setPosition([0.,.3, 1.3]) \
    .setQuaternion([.1, 0, 0, .0]) \
    .setShape(ry.ST.ssBox, size=[.9,.06,.06,.005]) \
    .setColor([1,.7,0]) \
    .setContact(1)


C.view(True, 'this is your workspace data structure C -- NOT THE SIMULTATION')

# True = real robot!!
bot = ry.BotOp(C, useRealRobot=True)

bot.sync(C)
bot.moveTo(bot.get_qHome())
bot.wait(C)
l_gripper = C.getFrame("l_gripper")
r_gripper = C.getFrame("r_gripper")
tube = C.getFrame("tube")
l= tube.getSize()[0]
print("l ", l)
# print(C.getJointState())

# ###### komo ###############
komo = ry.KOMO(C, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=False)
komo.addControlObjective([], 0, 1e-1)
komo.addControlObjective([], 1, 1e0)
komo.addObjective([1], ry.FS.positionRel, ['l_gripper', 'tube'], ry.OT.eq, [1e1], target=[-l/2,0,0])
komo.addObjective([1], ry.FS.positionRel, ['r_gripper', 'tube'], ry.OT.eq, [1e1], target=[l/2,0,0])
komo.addObjective([1], ry.FS.scalarProductXX, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXZ, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductYZ, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXX, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXZ, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductYZ, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])
ret = ry.NLP_Solver(komo.nlp(), verbose=0) .solve()

q = komo.getPath()
print('size of path:', q.shape)
# print("q-1", q[-1])
q0 = q[-1]
# bot.moveTo(q[-1])
# bot.wait(C)
# print("l-r ",l_gripper.getPosition()-r_gripper.getPosition())

komo = ry.KOMO(C, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=False)
komo.addControlObjective([], 0, 1e-1)
komo.addControlObjective([], 1, 1e0)
komo.addObjective([1], ry.FS.positionRel, ['l_gripper', 'goal'], ry.OT.eq, [1e1], target=[-l/2,0,0])
komo.addObjective([1], ry.FS.positionRel, ['r_gripper', 'goal'], ry.OT.eq, [1e1], target=[l/2,0,0])
komo.addObjective([1], ry.FS.scalarProductXX, ['l_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXZ, ['l_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductYZ, ['l_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXX, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXZ, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductYZ, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])
ret = ry.NLP_Solver(komo.nlp(), verbose=0) .solve()
q = komo.getPath()
q1 = q[-1]

# bot.moveTo(q0)
# bot.wait(C)
# bot.moveTo(q1)
# bot.wait(C)

komo = ry.KOMO(C, phases=1, slicesPerPhase=20, kOrder=1, enableCollisions=False)
komo.addControlObjective([], 0, 1e-1)
komo.addControlObjective([], 1, 1e0)
komo.addObjective([0.3], ry.FS.jointState, [], ry.OT.sos, [1e1], target=q0)
komo.addObjective([0.3, 1], ry.FS.positionDiff, ['l_gripper', 'r_gripper'], ry.OT.eq, [1e1], target=[-l,0,0])
# komo.addObjective([0, 1], ry.FS.quaternionDiff, ['l_gripper', 'r_gripper'], ry.OT.eq, [1e1], target=[0,0,0,0])
komo.addObjective([0.7], ry.FS.positionRel, ['l_gripper', 'goal'], ry.OT.eq, [1e1], target=[-l/2,0,0])
komo.addObjective([0.7], ry.FS.positionRel, ['r_gripper', 'goal'], ry.OT.eq, [1e1], target=[l/2,0,0])
komo.addObjective([0.7], ry.FS.scalarProductXX, ['l_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([0.7], ry.FS.scalarProductXZ, ['l_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([0.7], ry.FS.scalarProductYZ, ['l_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([0.7], ry.FS.scalarProductXX, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([0.7], ry.FS.scalarProductXZ, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([0.7], ry.FS.scalarProductYZ, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.positionRel, ['l_gripper', 'goal2'], ry.OT.eq, [1e1], target=[-l/2,0,0])
komo.addObjective([1], ry.FS.positionRel, ['r_gripper', 'goal2'], ry.OT.eq, [1e1], target=[l/2,0,0])
komo.addObjective([1], ry.FS.scalarProductXX, ['l_gripper', 'goal2'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXZ, ['l_gripper', 'goal2'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductYZ, ['l_gripper', 'goal2'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXX, ['r_gripper', 'goal2'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXZ, ['r_gripper', 'goal2'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductYZ, ['r_gripper', 'goal2'], ry.OT.eq, [1e1], [0])

# positionRel is not working very well

# komo = ry.KOMO(C, phases=1, slicesPerPhase=10, kOrder=1, enableCollisions=False)
# komo.addControlObjective([], 0, 1e-1)
# komo.addControlObjective([], 1, 1e0)
# komo.addObjective([0.3], ry.FS.jointState, [], ry.OT.sos, [1e1], target=q0)
# komo.addObjective([0.3, 1], ry.FS.positionDiff, ['l_gripper', 'r_gripper'], ry.OT.eq, [1e1], target=[-l,0,0])
# # komo.addObjective([0, 1], ry.FS.quaternionDiff, ['l_gripper', 'r_gripper'], ry.OT.eq, [1e1], target=[0,0,0,0])
# komo.addObjective([1], ry.FS.jointState, [], ry.OT.sos, [1e1], target=q1)

ret = ry.NLP_Solver(komo.nlp(), verbose=0) .solve()
q = komo.getPath()

# print("q0, ", q0)
# print("cal ", q[0])
#
# for t in range(q.shape[0]):
#     bot.moveTo(q[t])
#     bot.wait(C)

#     print("l-r ",l_gripper.getPosition()-r_gripper.getPosition())
#     # print("r ",r_gripper.getPosition())
#     # print( r_gripper.getQuaternion())

# be careful about the max velocity!!!
bot.moveAutoTimed(q, .5, .2) # path, max vel, max acc
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)