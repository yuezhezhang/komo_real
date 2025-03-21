import robotic as ry
import time

C = ry.Config()
C.addFrame("base") .setPosition([0,0,.5])

C.addFrame("ego", "base") \
    .setJoint(ry.JT.transXYPhi, [-1.,-1.,-3.,1.,1.,3.]) \
    .setRelativePosition([.2, .0, .0]) \
    .setShape(ry.ST.ssBox, size=[.05, .3, .05, .01]) \
    .setColor([0, 1., 1.]) \
    .setContact(1)

C.addFrame("obstacle") \
    .setPosition([.0, .0, .5]) \
    .setShape(ry.ST.ssBox, size=[.05, .3, .05, .01]) \
    .setColor([1, .5, 0]) \
    .setContact(1)

C.addFrame("obs") \
    .setPosition([.0, .0, .5]) \
    .setShape(ry.ST.ssBox, size=[.05, .3, .05, .01]) \
    .setColor([1, .5, 0]) \
    .setContact(1)

C.addFrame("l_gripper") \
    .setPosition([.0, .0, .5]) \
    .setShape(ry.ST.ssBox, size=[.05, .3, .05, .01]) \
    .setColor([1, .5, 0]) \
    .setContact(0)

C.addFrame("r_gripper") \
    .setPosition([.0, .0, .5]) \
    .setShape(ry.ST.ssBox, size=[.05, .3, .05, .01]) \
    .setColor([1, .5, 0]) \
    .setContact(1)


C.view(False)

q0 = [-.2, 0, 0]
qT = [+.2, 0, 0]

ry.params_clear()
ry.params_add({'rrt/stepsize':.1, 'rrt/verbose': 3}) #verbose=3 makes it very slow, and displays result, and verbose=4 waits keypress..

rrt = ry.PathFinder()
rrt.setProblem(C, [q0], [qT])
# rrt.setStartGoal([q0], [qT])
# ret = rrt.solve()
# print(ret)
# path = ret.x

# ry.params_print()