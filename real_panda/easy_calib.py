import os, sys
import time
import cv2
import numpy as np
import argparse
import pyzed.sl as sl
import multiprocessing
import matplotlib.pyplot as plt
import yaml

from utils import ARUCO_DICT, aruco_display, aruco_pose_esitmation, average_se4
import threading
import robotic as ry
import json
from collections import deque

ARUCO_TABLE_IDS = [24, 25, 26]
ARUCO_OBSTACLE_ID = 66

def resize_K(K, original_size, new_size):
    """
    Adjust the intrinsic matrix K for a resized image.

    Parameters:
    - K: numpy array (3x3), the original intrinsic matrix.
    - original_size: tuple (width, height) of the original image size.
    - new_size: tuple (width, height) of the new image size.

    Returns:
    - K_new: numpy array (3x3), the adjusted intrinsic matrix.
    """
    # Calculate scale factors for x and y dimensions
    sx = new_size[0] / original_size[0]
    sy = new_size[1] / original_size[1]
    
    # Scale the focal lengths and principal points
    K_new = K.copy()
    K_new[0, 0] *= sx   # fx
    K_new[1, 1] *= sy   # fy
    K_new[0, 2] *= sx   # cx
    K_new[1, 2] *= sy   # cy
    return K_new 

class ZEDArucoPosEst:
    def __init__(self, args):
        # Shared data
        self.shared_data = {"obs_pose":None, "obj_pose":None}
        # Lock to ensure thread safety
        self.lock = threading.Lock()

        # run zed camera
        # initialize zed camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.coordinate_units = sl.UNIT.METER
        init_params.camera_fps = args.fps  # Set the frame rate
        
        self._close = args.close
        self._far = args.far

        # load offset yaml
        self._offset = yaml.load(open(args.offset, 'r'))

        # aruco parameters
        self.aruco = {}
        self.aruco["type"] = args.aruco_type
        self.aruco["size"] = args.aruco_size

        try:
            self.aruco_dict = cv2.aruco.Dictionary_get(getattr(cv2.aruco, self.aruco["type"]))
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except:
            raise ValueError("Invalid aruco dictionary type")

        init_params.depth_minimum_distance = args.close # Set the minimum depth perception distance to 15cm 
        init_params.depth_maximum_distance = args.far # Set the maximum depth perception distance to 20m

        # set the depth mode
        if args.depth_mode == 0:
            init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        elif args.depth_mode == 1:
            init_params.depth_mode = sl.DEPTH_MODE.QUALITY
        elif args.depth_mode == 2:
            init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        elif args.depth_mode == 3:
            init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        else:
            raise ValueError("Invalid depth mode")

        # set the resolution
        if args.resolution == 0:
            init_params.camera_resolution = sl.RESOLUTION.HD2K
        elif args.resolution == 1:
            init_params.camera_resolution = sl.RESOLUTION.HD1080
        elif args.resolution == 2:
            init_params.camera_resolution = sl.RESOLUTION.HD720
        elif args.resolution == 3:
            init_params.camera_resolution = sl.RESOLUTION.VGA
        else:
            raise ValueError("Invalid resolution")

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        self.init_params = init_params

        # create runtime parameters
        # TODO change runtime parameters
        self.runtime_params = sl.RuntimeParameters()

        # get the camera calibration parameters
        calibration_params = self.zed.get_camera_information().camera_configuration.calibration_parameters

        # Focal length of the left eye in pixels
        fx = calibration_params.left_cam.fx
        fy = calibration_params.left_cam.fy

        # Principal point of the left eye in pixels
        cx = calibration_params.left_cam.cx
        cy = calibration_params.left_cam.cy

        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

        # set camera distortion parameters
        self.distortion = np.zeros((1, 5))    
        #### Other camera settings
        ## Set exposure to 50% of camera framerate
        #zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 50)
        ## Set white balance to 4600K
        #zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITE_BALANCE, 4600)
        ## Reset to auto exposure
        #zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, -1)

        # create Mat to store images
        self.image = sl.Mat()
        self.depth = sl.Mat()

        # get the width and height of the camera
        self.width = args.width
        self.height = args.height

        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                # Retrieve the RGB image
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)  # Retrieve the left RGB image
            # Retrieve the depth map
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)  # Retrieve the depth map

            # Convert the images to OpenCV format (BGR for RGB and a grayscale for depth)
            rgb = self.image.get_data()[:, :, :3]  # Get the RGB part of the image
            depth = self.depth.get_data()

            assert rgb.shape[:2] == depth.shape[:2], "RGB and depth image shapes do not match"

            # Get the width and height of the images
            W, H = rgb.shape[1], rgb.shape[0]

        # resize instrinsic matrix
        self.K = resize_K(K, (W, H), (self.width, self.height))

        self.DEBUG_LEVEL = args.debug_level

        # initialize hypar parameters
        self._latest_zed_data_t = time.time()

        # self.run_ik()
        self.last_10_frames = deque(maxlen=10)

    def run_perception(self):
        print("INFO: Starting GUI")
        # TODO change to interactive GUI later 
        cv2.namedWindow("ArucoPoseEtimator", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ArucoPoseEtimator", 2 * self.width, self.height)

        while 1:
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image, sl.VIEW.LEFT)  # Retrieve the left RGB image
                # Retrieve the depth map
                self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)  # Retrieve the depth map
                dt = (time.time() - self._latest_zed_data_t)
                self._latest_zed_data_t = time.time()

                print(f"run_perception runs at {1/dt} Hz")
                # Convert the images to OpenCV format (BGR for RGB and a grayscale for depth)
                rgb = self.image.get_data()[:, :, :3]  # Get the RGB part of the image
                depth = self.depth.get_data()

                depth[depth < self._close] = 0
                depth[depth > self._far] = 0
                depth[np.isnan(depth)] = 0.

                # resize the images
                rgb = cv2.resize(rgb, (self.width, self.height))
                depth = cv2.resize(depth, (self.width, self.height))

                corners, ids, rejected = cv2.aruco.detectMarkers(rgb, self.aruco_dict, parameters=self.aruco_params)
                #detected_markers = aruco_display(corners, ids, rejected, rgb)
                
                # predicted franse base pose
                base_cands = []
                obstacle_pose = None
                if ids is not None:
                    # print(f"INFO: Detected markers: {ids.flatten()}")
                    aruco_poses ,rgb = aruco_pose_esitmation(rgb, ARUCO_DICT[self.aruco['type']], self.aruco['size'], self.K, self.distortion)

                    for id, pose in aruco_poses:
                        if int(id) in self._offset.keys():
                            xyz = np.array(self._offset[int(id)]['xyz'], dtype=np.float32)
                            rpy = np.array(self._offset[int(id)]['rpy'], dtype=np.float32)
                            offset = np.eye(4)
                            offset[:3, 3] = xyz
                            offset[:3, :3] = cv2.Rodrigues(rpy)[0]
                            pose = np.dot(pose, offset)
                            base_cands.append(pose)
                            rvec, tvec = cv2.Rodrigues(pose[:3, :3])[0], pose[:3, 3]
                            cv2.aruco.drawAxis(rgb, self.K, self.distortion, rvec, tvec, 0.1)
                        if int(id) == ARUCO_OBSTACLE_ID:
                            obstacle_pose = pose
                            # print("INFO: Obstacle detected, pose: ", pose)

                if len(base_cands) > 0:
                    # NOTE average the base poses
                    base_aver_pose = average_se4(base_cands)
                    rvec, tvec = cv2.Rodrigues(base_aver_pose[:3, :3])[0], base_aver_pose[:3, 3]
                    cv2.aruco.drawAxis(rgb, self.K, self.distortion, rvec, tvec, 0.15)

                
                # with self.lock:
                #     if obstacle_pose is not None:
                #         obj_in_base = np.dot(np.linalg.inv(base_aver_pose), obstacle_pose)
                #         print("INFO: obstacle pose in base frame: ", obj_in_base[:3, 3])
                #         # yield f"Sending data!! {obj_in_base}"
                        
                #         # Add data to the shared resource
                #         self.shared_data["obs_pose"] = obj_in_base
                #         # print("obs_pose added")
                #     else:
                #         # Add data to the shared resource
                #         self.shared_data["obs_pose"] = None

                
                FILE_PATH = "data.json"
                if obstacle_pose is not None:
                    obj_in_base = np.dot(np.linalg.inv(base_aver_pose), obstacle_pose)
                    print("INFO: obstacle pose in base frame: ", obj_in_base[:3, 3])
                    data = list(obj_in_base[:3, 3])
                else:
                    data = [10, 10, 10]

                # with open(FILE_PATH, "w") as f:
                #     json.dump(data, f)
                # print(f"Generated data: {data}")

                self.last_10_frames.append(data)
                # Serialize the last 10 frames and write them to the file
                with open(FILE_PATH, "w") as f:
                    json.dump(list(self.last_10_frames), f)

                # with open(FILE_PATH, "a") as f:
                #     f.write(json.dumps(data) + "\n")

                # Normalize the depth image to the range [0, 255]
                normalized_depth = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
                normalized_depth = np.uint8(normalized_depth)
                colored_depth = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)

                # Stack the RGB image and the colored depth image horizontally
                fused_img = np.hstack((rgb, colored_depth))
                cv2.imshow("ArucoPoseEtimator", fused_img)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break

        self.zed.close()
        cv2.destroyAllWindows()

    def process_data(self):
        while True:
            # Lock before accessing the shared data
            with self.lock:
                if self.shared_data["obs_pose"] is not None:
                    # Process and remove data from the shared resource
                    data = self.shared_data["obs_pose"]
                    print(f"Data processed: {data}")
                else:
                    time.sleep(0.01)
            # Simulate some work
            time.sleep(1)

    def run_ik(self):
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
            .setPosition([0.15, -0.1, 1.12]) \
            .setShape(ry.ST.ssBox, size=[.47,.025,.104,.005]) \
            .setColor([.5,.5,0]) \
            .setContact(0)
        
        # C.view(False,"real_env")

        self.bot = ry.BotOp(self.C, useRealRobot=True)
        self.bot.sync(self.C)

        self.q_home = self.bot.get_qHome()
        tube = self.C.getFrame('tube')
        l= tube.getSize()[0]

        # IK q0
        komo = ry.KOMO(self.C, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=False)
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

        self.q0 = komo.getPath()[0]

        # IK qT
        komo = ry.KOMO(self.C, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=False)
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
        self.qT = komo.getPath()[0]
        self.l_panda_base= self.C.getFrame("l_panda_base").getPosition()


    def get_obj_position(self, obj_pose_from_camera):
        obj_position = obj_pose_from_camera[:3,3]
        obj_position[:2] = self.l_panda_base[:2]-obj_position[:2]
        obj_position[2] = self.l_panda_base[2] + obj_position[2] -0.06 # 0.06 is box size
        print("camera", obj_pose_from_camera[:3,3])
        # print("base ", self.l_panda_base)
        # print("obj", obj_position)
        # print("hhhhh")
        return obj_position

    def run_planning(self):
        latest_plan_t = time.time()
        print("qhome: ", self.q_home)
        print("q0: ", self.q0)
        print("qT: ", self.qT)
        obs_position = None

        # self.bot.sync(self.C)
        self.q_home[0] -= 0.5
        self.bot.moveTo(self.q_home)
        # print(self.q_home)

        # while True:
        #     breakpoint
        # self.bot.wait(self.C)

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
            # print(f"Planning runs at {1/dt} Hz")
            with self.lock:
                if self.shared_data["obs_pose"] is not None:
                    # Process and remove data from the shared resource
                    obs_pose_from_camera = self.shared_data["obs_pose"]
                    self.shared_data["obs_pose"] = None
                    print(f"obstacle position: {obs_pose_from_camera[:3, 3]}")
                    obs_position = self.get_obj_position(obs_pose_from_camera)
                    # self.C.getFrame("obs").setPosition(obs_position)
                    # time.sleep(0.01)
                else:
                    obs_position = [100, 8, 8]
                    # time.sleep(0.01)


            self.C.getFrame("obs").setPosition(obs_position)
            self.bot.sync(self.C, .01)

            time.sleep(0.01) # important!!!!
            # # print("current joint q0", np.linalg.norm(self.bot.get_q()-self.q0))
            # # print("current joint qT", np.linalg.norm(self.bot.get_q()-self.qT))
            if np.linalg.norm(self.bot.get_q()-self.q0) < 0.05:
                start = self.q0
                goal = self.qT
                # self.C.getFrame("obs").setPosition(obs_position)
                # self.bot.sync(self.C)
            elif np.linalg.norm(self.bot.get_q()-self.qT) < 0.05:
                start = self.qT
                goal = self.q0
                # self.C.getFrame("obs").setPosition(obs_position)
                # self.bot.sync(self.C)
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
                    if len(path) < 3:
                        # print
                        for i in range(len(path)):
                            self.bot.moveTo(path[i])
                            self.bot.sync(self.C, .01)
                    else:
                        self.bot.moveAutoTimed(path, .4, .2) # path, max vel, max acc .4, .2
                        while self.bot.getTimeToEnd()>0:
                            self.bot.sync(self.C, .02)
                else:
                    print("no path find")
                # time.sleep(2)
                

    def start_threads(self):
        # Create threads for both functions
        thread1 = threading.Thread(target=self.run_perception)
        thread2 = threading.Thread(target=self.run_planning)
        # Start the threads
        thread1.start()
        thread2.start()
        # Optionally, join threads (blocks the main thread)
        thread1.join()
        thread2.join()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run 6DPoseSplats on zed camera")
    parser.add_argument("--fps", type=int, default=30, help="Frame rate of the recorded video (default: 30)")
    parser.add_argument("--depth_mode", type=int, default=0, help="Depth mode of zed camera (default: 0) (0: ULTRA, 1: QUALITY, 2: PERFORMANCE, 3: NEURAL)")
    parser.add_argument("--resolution", type=int, default=1, help="Resolution of the recorded video (default: 1) (0: HD2K, 1: HD1080, 2: HD720, 3: VGA)")
    parser.add_argument("--close", type=float, default=0.15, help="Minimum depth perception distance (default: 0.15)")
    parser.add_argument("--far", type=float, default=10., help="Maximum depth perception distance (default: 10.)")
    parser.add_argument("--offset", type=str, default="offset.yaml", help="Path to the offset file (default: offset.yaml)")
    parser.add_argument("--width", type=int, default=1280, help="Width of the recorded video (default: 640)")
    parser.add_argument("--height", type=int, default=760, help="Height of the recorded video (default: 480)")
    parser.add_argument("--aruco_type", type=str, default="DICT_5X5_100", help="Type of the aruco dictionary (default: DICT_5X5_100)")
    parser.add_argument("--aruco_size", type=float, default=0.081, help="Size of the aruco markers (default: 0.1)")
    parser.add_argument("--debug_level", type=int, default=3, help="Debug level (default: 2)")
    args = parser.parse_args()
    
    ape = ZEDArucoPosEst(args)
    ape.run_perception()
    # ape.start_threads()
