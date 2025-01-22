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
    .setPosition([0.05,-.05, 1.1]) \
    .setShape(ry.ST.ssBox, size=[.47,.025,.204,.005]) \
    .setColor([1,.5,0]) \
    .setContact(0)

C.addFrame('goal') \
    .setPosition([-0, .2, 0.9]) \
    .setQuaternion([.991, 0, 0, -.131]) \
    .setShape(ry.ST.ssBox, size=[.47,.025,.204,.005]) \
    .setColor([1,.7,0]) \
    .setContact(0)

# obj_real_postion:   [ 0.09522017 -0.05757981  1.00725959]

C.addFrame('obs') \
    .setPosition([0.09522017 ,-0.09757981 , 0.92725959]) \
    .setShape(ry.ST.ssBox, size=[.76,.35,.027,.005]) \
    .setColor([.1,.5,0]) \
    .setContact(1)


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
bot.gripperMove(ry._left, width=.075, speed=.1)
bot.gripperMove(ry._right, width=.075, speed=.1)
bot.wait(C)
l_gripper = C.getFrame("l_gripper")
r_gripper = C.getFrame("r_gripper")
tube = C.getFrame("tube")
l= tube.getSize()[0]
print("l: ",l )

# while True:
#     breakpoint
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
# q0= [-0.953093, 0.636755, -0.413813, -1.75244, -2.74487, 2.39187, 0.738323, 0.743294, 0.734697, -0.127894, -1.53943, 2.65222, 2.34956, -1.90737]


# q1=[[-1.08744, 0.537657, -0.393765, -1.35877, -2.65228, 2.84496, 0.514074, 0.751352, 0.326867, -0.102414, -1.67723, 2.4523, 2.58999, -1.70936],
#  [-1.07724, 0.504129, -0.417232, -1.3997, -2.62695, 2.82951, 0.509482, 0.73704, 0.323641, -0.0716403, -1.6713, 2.39284, 2.56593, -1.67726],
#  [-1.06358, 0.483932, -0.465289, -1.41942, -2.62636, 2.82457, 0.504321, 0.76593, 0.300626, -0.0720752, -1.6796, 2.32165, 2.56426, -1.66804],
#  [-1.06096, 0.508161, -0.44891, -1.42622, -2.63013, 2.78727, 0.467684, 0.766872, 0.313581, -0.0937148, -1.69094, 2.2654, 2.55666, -1.62971],
#  [-0.893217, 0.995328, -0.969228, -0.896346, -0.295027, 2.34096, 0.0573909, 0.49845, 0.31246, 0.117415, -1.47877, 0.724557, 2.03236, -0.598121],
#  [-0.904265, 1.36301, -1.12779, -0.539669, -0.0200293, 2.77779, 0.403644, 0.234921, 0.601218, 0.0822838, -1.27792, 0.942081, 2.07279, -1.1321],
#  [-0.885153, 1.47842, -1.21202, -0.521997, 0.00617874, 2.87841, 0.463187, 0.157128, 0.680721, 0.10149, -1.28229, 0.966487, 2.09023, -1.19817],
#  [-0.550782, 1.38116, -1.24171, -0.771861, -0.00677825, 2.11947, 0.498541, 0.129633, 0.715113, 0.117393, -1.26663, 1.00034, 2.09486, -1.26709],
#  [-0.523508, 1.19943, -0.989831, -1.01902, -0.0117243, 2.25569, 0.509361, 0.162876, 0.760648, 0.110943, -1.27483, 1.03471, 2.12041, -1.28765],
#  [-0.762123, 1.13512, -0.637597, -0.902225, -0.0149002, 2.34842, 0.600555, 0.146288, 0.855579, 0.123112, -1.24723, 1.08194, 2.13951, -1.45036],
#  [-1.00249, 1.35514, -0.396519, -0.674217, -1.48563, 2.5505, 0.608304, 0.239177, 0.930811, 0.1124, -1.25305, 1.197, 2.16421, -1.49777],
#  [-1.00587, 1.37231, -0.414105, -0.633239, -1.54427, 2.58569, 0.602513, 0.219743, 0.926788, 0.117715, -1.25355, 1.24451, 2.15528, -1.50335],
#  [-0.835562, 1.29931, -0.544758, -0.857715, -2.04399, 2.51445, 0.488722, 0.0807087, 1.04989, 0.360795, -1.25013, 2.01593, 2.29524, -1.7962],
#  [-0.838912, 1.3276, -0.570248, -0.854652, -2.10041, 2.54861, 0.477379, 0.103091, 1.05877, 0.342967, -1.26409, 2.05268, 2.32147, -1.80457],
#  [-0.843627, 1.31277, -0.545594, -0.876343, -2.13724, 2.55717, 0.459169, 0.127247, 1.06868, 0.319283, -1.24929, 2.08919, 2.32828, -1.85679],
#  [-0.805955, 1.22315, -0.533681, -1.03234, -2.12749, 2.55337, 0.429945, 0.173884, 1.08925, 0.282035, -1.22179, 2.14209, 2.32031, -1.859]]


# [-1.08739, 0.537233, -0.394552, -1.35653, -2.65032, 2.84769, 0.512116, 0.751352, 0.326867, -0.102414, -1.67723, 2.4523, 2.58999, -1.70936,
#  -0.806133, 0.773943, -0.504856, -1.00723, -1.19448, 2.11309, 0.102466, 0.345365, 0.607731, 0.109504, -1.15829, 1.40908, 2.13755, -1.0461,
#  -0.865681, 1.04835, -0.478527, -0.731051, -1.38581, 2.43332, 0.0965445, 0.237254, 0.726624, 0.0698646, -1.12594, 1.50298, 2.16595, -1.10169,
#  -0.889682, 1.13272, -0.496601, -0.6386, -1.4254, 2.50334, 0.0772817, 0.190699, 0.75844, 0.114283, -1.12924, 1.51505, 2.17745, -1.06472,
#  -0.870505, 1.19856, -0.596154, -0.638652, -1.54393, 2.62881, 0.0980037, 0.127341, 0.825387, 0.20685, -1.13758, 1.55396, 2.23405, -1.265,
#  -0.724355, 1.17205, -0.690731, -0.948633, -1.81404, 2.70165, 0.183024, 0.259837, 0.984779, 0.108368, -1.12939, 2.08804, 2.34432, -1.58412]

q0= q[0:24]
# q1= q[24:]

bot.moveAutoTimed(q0, .3, .2) # path, max vel, max acc
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)
# bot.moveTo(q0,.5)



bot.wait(C)

# close the gripper #
bot.gripperMove(ry._left, width=.00, speed=.2)
bot.gripperMove(ry._right, width=.00, speed=.2)
# while not (bot.gripperDone(ry._left) and bot.gripperDone(ry._right)):
#     bot.sync(C)


# q1 = [-1.08744, 0.537657, -0.393765, -1.35877, -2.65228, 2.84496, 0.514074, 0.751352, 0.326867, -0.102414, -1.67723, 2.4523, 2.58999, -1.70936]
# q2 = [-0.859589, 1.09017, -0.523263, -0.784225, -1.90028, 2.66792, 0.294405, 0.298556, 0.754048, 0.0191136, -1.25928, 2.09865, 2.34652, -1.6187]
# q3 = [-0.798323, 1.14817, -0.54347, -0.846782, -1.70093, 2.58032, 0.276456, 0.232338, 0.946599, 0.0873761, -1.10443, 1.94192, 2.25357, -1.53132]
# q4 = [-0.790249, 1.35788, -0.609743, -0.810159, -1.84941, 2.60573, 0.23348, 0.264731, 1.18791, 0.0970014, -1.00883, 2.1353, 2.28851, -1.66863]



####  obs run successfully###############################
q1 =[[-1.08744, 0.537657, -0.393765, -1.35877, -2.65228, 2.84496, 0.514074, 0.751352, 0.326867, -0.102414, -1.67723, 2.4523, 2.58999, -1.70936],
 [-1.07724, 0.504129, -0.417232, -1.3997, -2.62695, 2.82951, 0.509482, 0.73704, 0.323641, -0.0716403, -1.6713, 2.39284, 2.56593, -1.67726],
 [-1.09203, 0.497798, -0.382309, -1.42594, -2.61384, 2.80929, 0.450174, 0.757433, 0.347211, -0.101926, -1.65815, 2.38495, 2.56818, -1.65819],
 [-0.9071, 0.550746, -0.475894, -1.48345, -2.2851, 2.85346, 0.373339, 0.570341, 0.604646, -0.0260797, -1.44007, 2.25191, 2.45237, -1.65463],
 [-0.873374, 0.560371, -0.502583, -1.45919, -2.22241, 2.82707, 0.32836, 0.550691, 0.594524, -0.0199242, -1.42547, 2.23596, 2.45674, -1.64583],
 [-0.730029, 0.902851, -0.565568, -1.1225, -1.73824, 2.56666, 0.270158, 0.305823, 0.829531, 0.0522881, -1.16554, 2.02637, 2.26226, -1.45523],
 [-0.74872, 1.07788, -0.553258, -0.938243, -1.6717, 2.48211, 0.238666, 0.261041, 0.941986, 0.0497303, -1.06781, 1.95161, 2.20572, -1.40617],
 [-0.742941, 1.12802, -0.572848, -0.926821, -1.655, 2.49171, 0.285592, 0.248862, 0.982116, 0.0759846, -1.06377, 1.94741, 2.22763, -1.45362],
 [-0.797216, 1.32278, -0.603736, -0.774605, -1.79718, 2.6143, 0.193319, 0.248952, 1.11202, 0.0959205, -1.05045, 2.14303, 2.29068, -1.6554],
 [-0.772548, 1.31362, -0.626275, -0.851966, -1.85065, 2.62324, 0.234657, 0.280907, 1.13949, 0.0672121, -1.04585, 2.15127, 2.28527, -1.65007]]

# bot.moveTo(q1)
# bot.wait(C)

# bot.moveTo(q2)
# bot.wait(C)

# bot.moveTo(q3)
# bot.wait(C)

# bot.moveTo(q4)
# bot.wait(C)

# q = [q1, q2, q3, q4]

bot.moveAutoTimed(q1, .4, .25) # path, max vel, max acc
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)



## move to home at the end ##

# bot.sync(C)
# bot.moveTo(bot.get_qHome())
# bot.gripperMove(ry._left, width=.08, speed=.1)
# bot.gripperMove(ry._right, width=.08, speed=.1)
# bot.wait(C)



