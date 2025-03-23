import robotic as ry
import numpy as np
import time
print('version:', ry.__version__, ry.compiled())

C = ry.Config()
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
    .setShape(ry.ST.ssBox, size=[.43,.06,.06,.005]) \
    .setColor([1,.7,0]) \
    .setContact(1)



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

####### start pose ###############
komo = ry.KOMO(C, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=False)
komo.addControlObjective([], 0, 1e-1)
komo.addControlObjective([], 1, 1e0)
komo.addObjective([1], ry.FS.positionRel, ['r_gripper', 'tube'], ry.OT.eq, [1e1], target=[l/2,0,0])
komo.addObjective([1], ry.FS.positionRel, ['l_gripper', 'tube'], ry.OT.eq, [1e1], target=[-l/2,0,0])
komo.addObjective([1], ry.FS.scalarProductXX, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductXZ, ['tube', 'r_gripper'], ry.OT.eq, [1e1], [1])
komo.addObjective([1], ry.FS.scalarProductZZ, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])
# red is x axis, blue is z axis, green is y axis
komo.addObjective([1], ry.FS.scalarProductXX, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
# komo.addObjective([1], ry.FS.scalarProductXZ, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
komo.addObjective([1], ry.FS.scalarProductZZ, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
# komo.addObjective([1], ry.FS.scalarProductZZ, ['l_gripper', 'r_gripper'], ry.OT.eq, [1e1], [-1])
komo.addObjective([1], ry.FS.scalarProductXZ, ['tube', 'l_gripper'], ry.OT.eq, [1e1], [-1])
ret = ry.NLP_Solver(komo.nlp(), verbose=0) .solve()

q = komo.getPath()
q0 = q[-1]

####### end pose ###############
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

bot.moveTo(q1)
bot.wait(C)
time.sleep(1)
bot.gripperMove(ry._left, width=.0, speed=.1)
bot.gripperMove(ry._right, width=.0, speed=.1)
C.view(True)