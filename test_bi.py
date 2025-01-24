import robotic as ry
import time





C = ry.Config()
C.addFile('/home/frankie/git/komo_real/pandaDual.g')
C.view()




C.addFrame('tube') \
            .setPosition([0.15,-0.1, 1.55]) \
            .setShape(ry.ST.ssBox, size=[.47,.025,.104,.005]) \
            .setColor([1,.5,0]) \
            .setContact(0)
        
        # .3,.15,.1,.005
        # -0.12 ,-0.0357981 , 0.90725959
C.addFrame('obs') \
            .setPosition([0.2,-0.1757981 , 1.3725959]) \
            .setShape(ry.ST.ssBox, size=[.40,.21,.1,.005]) \
            .setColor([.1,.1,0]) \
            .setContact(0)

C.addFrame('goal') \
    .setPosition([0.15, -0.1, 1.13]) \
    .setShape(ry.ST.ssBox, size=[.47,.025,.104,.005]) \
    .setColor([.5,.5,0]) \
    .setContact(0)
# y -0.957981  white box #



C.view(True,"real_env")
bot = ry.BotOp(C, useRealRobot=False)

# bot.sync(C)
# bot.moveTo(bot.get_qHome())
# bot.gripperMove(ry._left, width=.08, speed=.1)
# bot.gripperMove(ry._right, width=.08, speed=.1)
# bot.wait(C)



# C.addFrame('boxL','table') \
#     .setRelativePosition([-.1,0,.5]) \
#     .setShape(ry.ST.ssBox, size=[.1,.1,.1,.02]) \
#     .setColor([1,.5,0])
C.view()
tube = C.getFrame('tube')
l= tube.getSize()[0]

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

q0 = komo.getPath()[0]
# print('size of path:', q.shape)
# print("q-1", q[-1])

# C.setJointState(q[-1])
# C.view(True,"start pose")

# store the start configuration
# q0 = C.getJointState()

# q1 = q[-1]
# bot.moveTo(q0)
# bot.wait(C)


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

ret = ry.NLP_Solver() \
    .setProblem(komo.nlp()) \
    .setOptions( stopTolerance=1e-2, verbose=4 ) \
    .solve()
print(ret)

# that's the goal configuration
qT = komo.getPath()[0]
# C.setJointState(qT)
# C.view(False, "IK solution")

#define a path finding problem
rrt = ry.PathFinder()
rrt.setProblem(C, [q0], [qT])

ret = rrt.solve()
print(ret)
path = ret.x

print("path: ",path)
for t in range(0, path.shape[0]-1):
    C.setJointState(path[t])
    C.view()
    time.sleep(.1)

# bot.gripperMove(ry._left, width=.0, speed=.1)
# bot.gripperMove(ry._right, width=.0, speed=.1)
# while not (bot.gripperDone(ry._left) and bot.gripperDone(ry._right)):
#     bot.sync(C)

# bot.moveAutoTimed(path, .4, .2) # path, max vel, max acc
# while bot.getTimeToEnd()>0:
#     bot.sync(C, .1)


# time.sleep(5)


# bot.sync(C)
# bot.moveTo(bot.get_qHome())
# bot.gripperMove(ry._left, width=.08, speed=.1)
# bot.gripperMove(ry._right, width=.08, speed=.1)
# bot.wait(C)

# display the path


# # run the path with botop
# C.setJointState(q0)

del bot
del rrt

del C

