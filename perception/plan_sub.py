import zerorpc
import time
import cloudpickle
import numpy as np
import robotic as ry



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
            .setShape(ry.ST.ssBox, size=[.40,.21,.1,.005]) \
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

        self.bot.moveTo(self.q_home)
        self.bot.wait(self.C)
        tube = self.C.getFrame('tube')
        self.l= tube.getSize()[0]
        self.l_panda_base= self.C.getFrame("l_panda_base").getPosition()

        self.saved_frames = [10, 10, 10]  # To track the last processed data
        self.latest_data = [10, 10, 10]
        self.run_ik()

        self.bot.gripperMove(ry._left, width=.7, speed=.1)
        self.bot.gripperMove(ry._right, width=.7, speed=.1)
        while not (self.bot.gripperDone(ry._left) and self.bot.gripperDone(ry._right)):
            self.bot.sync(self.C)

        print("joint ", self.C.getJointState())
        self.bot.sync(self.C)
        self.bot.moveTo(self.q0)
        self.bot.wait(self.C)
        
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
        if obj_in_camera_pos is not None:
            obj_position = obj_in_camera_pos
            if np.all(obj_in_camera_pos == np.array([10, 10, 10])):
                return obj_position
            else:
                obj_position[:2] = self.l_panda_base[:2]-obj_position[:2]
                obj_position[1] = obj_position[1]+0.02
                obj_position[2] = self.l_panda_base[2] + obj_position[2] -0.06 # 0.06 is box size
                # print("camera", obj_position)
                # print("base ", self.l_panda_base)
                # print("obj", obj_position)
                return obj_position
    

    def run_planning(self, obj_pos):
        # print("obj_pos: ",obj_pos)
        latest_plan_t = time.time()
        # self.run_ik()
        # print("qhome: ", self.q_home)
        # print("q0: ", self.q0)
        # print("qT: ", self.qT)

        # self.bot.sync(self.C)
        self.q_home[0] -= 0.5
        # self.bot.moveTo(self.q_home)
        # print(self.q_home)


        self.bot.gripperMove(ry._left, width=.0, speed=.1)
        self.bot.gripperMove(ry._right, width=.0, speed=.1)
        while not (self.bot.gripperDone(ry._left) and self.bot.gripperDone(ry._right)):
            self.bot.sync(self.C)

        i = 0


        # while True:
        i += 1
        # Lock before accessing the shared data

        dt = time.time() - latest_plan_t
        latest_plan_t = time.time()
        # print(f"receiving {self.saved_frames[-1]}")
        # average_postition = [sum(x)/10 for x in zip(*self.saved_frames)] # may crash, write and read too many times
        # print(average_postition)
        obs_position = self.get_obj_position(obj_pos)
        print(f"Planning runs at {1/dt} Hz")

        # self.C.getFrame("obs").setPosition(obs_position)
        # print("set obs")
        # self.bot.sync(self.C, .01)

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
            self.C.getFrame("obs").setPosition(obs_position)
            print("set obs")
            self.bot.sync(self.C, .01)
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
                        self.bot.wait(self.C)
                else:
                    self.bot.moveAutoTimed(path, .5, .2) # path, max vel, max acc .4, .2
                    # self.bot.sync(self.C, .1)
                    # self.bot.wait(self.C)
                    while self.bot.getTimeToEnd()>0:
                    #     print("long")
                        self.bot.sync(self.C, .1)

            else:
                print("no path find")



def main():
    # Connect to the ZeroRPC server
    client = zerorpc.Client()
    client.connect("tcp://127.0.0.1:4242")
    print("Connected to the ZeroRPC server")
    planner = Planning()
    try:
        while True:
            # Call the `step` function on the server
            data = client.step()
            
            data = data['obj_in_base']
            data = np.asanyarray(data)
            print(data[:3,-1])
            planner.run_planning(data[:3,-1])
            # Sleep to simulate processing or interval
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting loop...")
    finally:
        client.close()

if __name__ == "__main__":
    main()