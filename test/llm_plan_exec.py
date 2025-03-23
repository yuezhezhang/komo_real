import robotic as ry
import numpy as np
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-t', '--teach',
                    action='store_true')
args = parser.parse_args()

print('version:', ry.__version__, ry.compiled())

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))
# C.addFile("pandaRight.g")
pcl = C.addFrame('pcl', 'cameraWrist')
C.view(False, 'this is your workspace data structure C -- NOT THE SIMULTATION')

# True = real robot!!
bot = ry.BotOp(C, useRealRobot=True)

if args.teach:
    # # kinesthetic teaching
    while bot.getKeyPressed()!=ord('q'):
        bot.hold(floating=True, damping=False)
        C.view(False, 'floating=True, damping=False -- Try moving the robot by hand!\n-- press any key to continue --')
        print("joint", C.getJointState())
        bot.sync(C, .1)
else:

    banana_grasp1 = [ 0.58420194, 0.4279313, 0.15042631, -2.52552051, -0.28327101, 2.94892156, 0.78557129]
    banana_grasp2 = [ 0.58256997, 0.43780531, 0.11344885, -2.53427242, -0.44022781, 2.98110086, -0.52243164]
    banana_grasp3 = [ 0.5750709, 0.45548646, 0.16363373, -2.51715382, -0.44055544, 2.95910236, -1.21204403]
    middle_point = [ 0.2090654, -0.27165537, -0.23320505, -2.22703813, -0.06249367, 1.98358131, 0.71559224]
    table_left_place = [ 0.12563387, 0.78554154, -0.41728069, -1.62554276, 0.43686632, 2.30927602, -1.21585499]
    knife_cut_pos =  [ 0.19315008, 0.55122841, -0.44018169, -2.08813777, 0.44000105, 2.6989948, 0.16890844]
    knife_holder_place = [-0.09630505,  1.15467125, -0.57445716, -1.18037255, 0.6548454, 2.1942581, 0.03896525]
    knife_middle_point = [0.08792488, 0.65759848, -0.3979188, -1.66965057, 0.29520867, 2.29120338, 0.37905899]
    banana_regrasp = [0.15961891, 0.94741644, -0.44922144, -1.55598883, 0.44558556, 2.4062637, -1.18401617]
    banana_final = [0.15559363, -0.06248226, -0.97154614, -2.63751808, -0.16517116, 2.55951153, -0.07293641]

    q = bot.get_qHome()
    print("home ", q) # [ 0.  -0.5  0.  -2.   0.   2.  -0.5]
    # q[1] = q[1] + .05
    # .08

    bot.gripperMove(ry._left, width=.08, speed=.1)
    bot.wait(C)
    bot.moveTo(q)
    bot.wait(C)


    bot.moveTo(banana_grasp1)
    bot.wait(C)
    bot.gripperMove(ry._left, width=.0, speed=.2)
    # bot.wait(C)
    time.sleep(1)

    bot.moveTo(middle_point)
    bot.gripperMove(ry._left, width=.0, speed=.2)
    bot.wait(C)

    bot.moveTo(table_left_place)
    bot.wait(C)
    # time.sleep(0.7)
    bot.gripperMove(ry._left, width=.08, speed=.2)
    time.sleep(0.7)

    bot.moveTo(middle_point)
    bot.wait(C)
    time.sleep(0.7)

    bot.moveTo(knife_holder_place)
    bot.wait(C)
    # time.sleep(0.7)
    bot.gripperMove(ry._left, width=.0, speed=.2)
    bot.wait(C)
    time.sleep(1)

    bot.moveTo(knife_middle_point)
    bot.wait(C)
    bot.gripperMove(ry._left, width=.0, speed=.2)
    time.sleep(0.7)

    bot.moveTo(knife_cut_pos)
    bot.wait(C)
    time.sleep(0.7)

    bot.moveTo(knife_middle_point)
    bot.gripperMove(ry._left, width=.0, speed=.2)
    bot.wait(C)
    time.sleep(0.7)

    bot.moveTo(knife_cut_pos)
    bot.wait(C)
    time.sleep(0.7)

    bot.moveTo(knife_middle_point)
    bot.gripperMove(ry._left, width=.0, speed=.2)
    bot.wait(C)
    time.sleep(0.7)

    bot.moveTo(knife_holder_place)
    bot.wait(C)
    # time.sleep(0.7)
    bot.gripperMove(ry._left, width=.7, speed=.2)
    bot.wait(C)
    time.sleep(1)

    bot.moveTo(knife_middle_point)
    bot.wait(C)
    time.sleep(0.7)

    bot.moveTo(banana_regrasp)
    bot.wait(C)
    bot.gripperMove(ry._left, width=.0, speed=.2)
    bot.wait(C)
    time.sleep(1)

    bot.moveTo(banana_final)
    bot.wait(C)
    bot.gripperMove(ry._left, width=.8, speed=.2)
    bot.wait(C)
    time.sleep(1)


