import robotic as ry
import time

C = ry.Config()
C.addFile('/home/frankie/git/komo_real/pandaDual.g')

C.addFrame('tube') \
    .setPosition([0.15,-0.1, 1.55]) \
    .setShape(ry.ST.ssBox, size=[.43,.025,.104,.005]) \
    .setColor([1,.5,0]) \
    .setContact(0)


C.addFrame('obs') \
    .setPosition([8, 8, 8]) \
    .setShape(ry.ST.ssBox, size=[.44,.21,.1,.005]) \
    .setColor([.1,.1,0]) \
    .setContact(1)


C.addFrame('goal') \
    .setPosition([0.15, -0.1, 1.13]) \
    .setShape(ry.ST.ssBox, size=[.43,.025,.104,.005]) \
    .setColor([.5,.5,0]) \
    .setContact(0)

tube = C.getFrame('tube')
l= tube.getSize()[0]

# C.view(False,"real_env")

C.view(True)

default_obs = [0.15,-0.15, 1.33]
C.getFrame("obs").setPosition(default_obs)
komo = ry.KOMO(C, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=True)
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

q0 = komo.getPath()[0]

# IK qT
komo = ry.KOMO(C, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=True)
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
qT = komo.getPath()[0]

C.setJointState(q0)
C.view(True)

C.setJointState(qT)
C.view(True)

ry.params_clear()
ry.params_add({'rrt/stepsize':.1, 'rrt/verbose': 3}) #verbose=3 makes it very slow, and displays result, and verbose=4 waits keypress..

rrt = ry.PathFinder()
rrt.setProblem(C, [q0], [qT])
ret = rrt.solve()
# # print(ret)
# # path = ret.x

# # ry.params_print()