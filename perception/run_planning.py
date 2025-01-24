
import robotic as ry
import time
import json
import numpy as np

class Planning:
    def __init__(self, args=None):
        self.C = ry.Config()
        self.C.addFile('/home/frankie/git/komo_real/pandaDual.g')
        # C.view()
        # 0.05,-.05, 1.4
        self.C.addFrame('tube') \
            .setPosition([0.15,-0.1, 1.55]) \
            .setShape(ry.ST.ssBox, size=[.47,.025,.104,.005]) \
            .setColor([1,.5,0]) \
            .setContact(0)
        
        # .3,.15,.1,.005
        # -0.12 ,-0.0357981 , 0.90725959
        self.C.addFrame('obs') \
            .setPosition([8, 8, 8]) \
            .setShape(ry.ST.ssBox, size=[.4,.21,.1,.005]) \
            .setColor([.1,.1,0]) \
            .setContact(1)

        # 0.15, -0.1, 1.1 yesterday
        # 0.0, -0.1, 1.1
        # 0.05, -0.05, 0.9
        self.C.addFrame('goal') \
            .setPosition([0.15, -0.1, 1.13]) \
            .setShape(ry.ST.ssBox, size=[.47,.025,.104,.005]) \
            .setColor([.5,.5,0]) \
            .setContact(0)
        
        # C.view(False,"real_env")

        self.bot = ry.BotOp(self.C, useRealRobot=True)
        self.bot.sync(self.C)

        self.q_home = self.bot.get_qHome()
        tube = self.C.getFrame('tube')
        self.l= tube.getSize()[0]
        self.l_panda_base= self.C.getFrame("l_panda_base").getPosition()

        self.saved_frames = [10, 10, 10]  # To track the last processed data
        self.latest_data = [10, 10, 10]

    def run_ik(self):
        # IK q0
        komo = ry.KOMO(self.C, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=False)
        komo.addControlObjective([], 0, 1e-1)
        komo.addControlObjective([], 1, 1e0)
        komo.addObjective([1], ry.FS.positionRel, ['r_gripper', 'tube'], ry.OT.eq, [1e1], target=[self.l/2,0,0])
        komo.addObjective([1], ry.FS.positionRel, ['l_gripper', 'tube'], ry.OT.eq, [1e1], target=[-self.l/2,0,0])
        komo.addObjective([1], ry.FS.scalarProductXX, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])
        komo.addObjective([1], ry.FS.scalarProductXZ, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])
        komo.addObjective([1], ry.FS.scalarProductZZ, ['r_gripper', 'tube'], ry.OT.eq, [1e1], [0])

        komo.addObjective([1], ry.FS.scalarProductXX, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
        komo.addObjective([1], ry.FS.scalarProductXZ, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
        komo.addObjective([1], ry.FS.scalarProductZZ, ['l_gripper', 'tube'], ry.OT.eq, [1e1], [0])
        komo.addObjective([1], ry.FS.scalarProductXZ, ['tube', 'l_gripper'], ry.OT.eq, [1e1], [-1])
        ret = ry.NLP_Solver(komo.nlp(), verbose=0) .solve()

        self.q0 = komo.getPath()[0]

        # IK qT
        komo = ry.KOMO(self.C, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=False)
        komo.addControlObjective([], 0, 1e-1)
        komo.addControlObjective([], 1, 1e0)
        komo.addObjective([1], ry.FS.positionRel, ['r_gripper', 'goal'], ry.OT.eq, [1e1], target=[self.l/2,0,0])
        komo.addObjective([1], ry.FS.positionRel, ['l_gripper', 'goal'], ry.OT.eq, [1e1], target=[-self.l/2,0,0])
        komo.addObjective([1], ry.FS.scalarProductXX, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])
        komo.addObjective([1], ry.FS.scalarProductXZ, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])
        komo.addObjective([1], ry.FS.scalarProductZZ, ['r_gripper', 'goal'], ry.OT.eq, [1e1], [0])

        komo.addObjective([1], ry.FS.scalarProductXX, ['l_gripper', 'goal'], ry.OT.eq, [1e1], [0])
        komo.addObjective([1], ry.FS.scalarProductXZ, ['l_gripper', 'goal'], ry.OT.eq, [1e1], [0])
        komo.addObjective([1], ry.FS.scalarProductZZ, ['l_gripper', 'goal'], ry.OT.eq, [1e1], [0])
        komo.addObjective([1], ry.FS.scalarProductXZ, ['goal', 'l_gripper'], ry.OT.eq, [1e1], [-1])

        ret = ry.NLP_Solver(komo.nlp(), verbose=0) .solve()
        self.qT = komo.getPath()[0]

    def get_obj_position(self, obj_in_camera_pos):
        obj_position = obj_in_camera_pos
        if obj_in_camera_pos == [10, 10, 10]:
            return obj_position
        else:
            obj_position[:2] = self.l_panda_base[:2]-obj_position[:2]
            obj_position[2] = self.l_panda_base[2] + obj_position[2] -0.06 # 0.06 is box size
            # print("camera", obj_position)
            # print("base ", self.l_panda_base)
            # print("obj", obj_position)
            return obj_position
    
    def get_data(self):
        FILE_PATH = "data.json"
        try:
            # # Read the latest data (last 10 frames) from the file
            with open(FILE_PATH, "r") as f:
                latest_frames = json.load(f)
            
            # If the frames are new, process them
            if latest_frames != self.saved_frames:
                # print(f"Processing new data (last 10 frames): {latest_frames}")
                
                # # Example: Process each frame (e.g., compute the sum of each frame)
                # processed_results = [sum(frame) for frame in latest_frames]
                # print(f"Processed results (sums): {processed_results}")
                # print("saved ", self.saved_frames[-1])
                
                self.saved_frames = latest_frames
        
            # # Open the file in read mode and get the last line
            # with open(FILE_PATH, "r") as f:
            #     # Read all lines and get the last one
            #     lines = f.readlines()
            #     if lines:
            #         self.latest_data = json.loads(lines[-1].strip())
            #         print(f"Latest data: {self.latest_data}")
            #     else:
            #         print("No data found in the file.")
            
        except FileNotFoundError:
            print(f"Waiting for data file '{FILE_PATH}' to be created...")
        except json.JSONDecodeError:
            print("File is empty or corrupted. Waiting for valid data...")

    def run_planning(self):
        latest_plan_t = time.time()
        self.run_ik()
        print("qhome: ", self.q_home)
        print("q0: ", self.q0)
        print("qT: ", self.qT)

        # self.bot.sync(self.C)
        self.q_home[0] -= 0.5
        # self.bot.moveTo(self.q_home)
        # print(self.q_home)

        self.bot.gripperMove(ry._left, width=.7, speed=.1)
        self.bot.gripperMove(ry._right, width=.7, speed=.1)
        while not (self.bot.gripperDone(ry._left) and self.bot.gripperDone(ry._right)):
            self.bot.sync(self.C)

        print("joint ", self.C.getJointState())
        self.bot.sync(self.C)
        self.bot.moveTo(self.q0)
        self.bot.wait(self.C)

        self.bot.gripperMove(ry._left, width=.0, speed=.1)
        self.bot.gripperMove(ry._right, width=.0, speed=.1)
        while not (self.bot.gripperDone(ry._left) and self.bot.gripperDone(ry._right)):
            self.bot.sync(self.C)

        i = 0

        obs_position = [8, 8, 8]

        while True:
            i += 1
            # Lock before accessing the shared data

            dt = time.time() - latest_plan_t
            latest_plan_t = time.time()
            self.get_data()
            # print(f"receiving {self.saved_frames[-1]}")
            # average_postition = [sum(x)/10 for x in zip(*self.saved_frames)] # may crash, write and read too many times
            # print(average_postition)
            obs_position = self.get_obj_position(self.saved_frames[-1])
            print(f"Planning runs at {1/dt} Hz")

            self.C.getFrame("obs").setPosition(obs_position)
            self.bot.sync(self.C, .01)

            time.sleep(0.01)

            if np.linalg.norm(self.bot.get_q()-self.q0) < 0.05:
                start = self.q0
                goal = self.qT
            elif np.linalg.norm(self.bot.get_q()-self.qT) < 0.05:
                start = self.qT
                goal = self.q0
            else: 
                start = None
                goal = None

            if start is not None:
                # print("start==q0 ", start == self.q0)
                rrt = ry.PathFinder()
                rrt.setProblem(self.C, [start], [goal])

                ret = rrt.solve()
                # print(ret.feasible)
                path = ret.x
                # print(path)

                if ret.feasible:
                    # for i in range(len(path)):
                        # self.bot.moveTo(path[i])
                        # self.bot.sync(self.C)
                    print(len(path))
                    if len(path) < 3:
                        # print("moveTo", path[0] == start)
                        for i in range(len(path)):
                            self.bot.moveTo(path[i])    
                            self.bot.sync(self.C)
                    else:
                        self.bot.moveAutoTimed(path, .4, .2) # path, max vel, max acc .4, .2
                        self.bot.sync(self.C, .1)
                        # self.bot.wait(self.C)
                        # while self.bot.getTimeToEnd()>0:
                        # #     print("long")
                        #     self.bot.sync(self.C, .1)

                else:
                    print("no path find")

if __name__ == "__main__":
    planner = Planning()
    planner.run_planning()