import zerorpc
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
import cloudpickle
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

class StepServer:
    def __init__(self,args):
        #zed#   
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
    def step(self):
        print("INFO: Starting GUI")
        # TODO change to interactive GUI later 
        cv2.namedWindow("ArucoPoseEtimator", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ArucoPoseEtimator", 2 * self.width, self.height)

        # while 1:
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)  # Retrieve the left RGB image
            # Retrieve the depth map
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)  # Retrieve the depth map
            dt = (time.time() - self._latest_zed_data_t)
            self._latest_zed_data_t = time.time()

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
                print(f"INFO: Detected markers: {ids.flatten()}")
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
                        print("INFO: Obstacle detected, pose: ", pose)

            if len(base_cands) > 0:
                # NOTE average the base poses
                base_aver_pose = average_se4(base_cands)
                rvec, tvec = cv2.Rodrigues(base_aver_pose[:3, :3])[0], base_aver_pose[:3, 3]
                cv2.aruco.drawAxis(rgb, self.K, self.distortion, rvec, tvec, 0.15)

                print("INFO: franke base pose: ", base_aver_pose)


            # Normalize the depth image to the range [0, 255]
            normalized_depth = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
            normalized_depth = np.uint8(normalized_depth)
            colored_depth = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)

            # Stack the RGB image and the colored depth image horizontally
            fused_img = np.hstack((rgb, colored_depth))
            cv2.imshow("ArucoPoseEtimator", fused_img)
        
            if obstacle_pose is not None:
                obj_in_base = np.dot(np.linalg.inv(base_aver_pose), obstacle_pose)
                print("INFO: obstacle pose in base frame: ", obj_in_base)
                return {"obj_in_base": obj_in_base.tolist()}
            return {"obj_in_base": [[10,10,10,10],[10,10,10,10],[10,10,10,10],[10,10,10,10]]}

# Create the ZeroRPC server
def main():
    parser = argparse.ArgumentParser(description="Run 6DPoseSplats on zed camera")
    parser.add_argument("--fps", type=int, default=30, help="Frame rate of the recorded video (default: 30)")
    parser.add_argument("--depth_mode", type=int, default=0, help="Depth mode of zed camera (default: 0) (0: ULTRA, 1: QUALITY, 2: PERFORMANCE, 3: NEURAL)")
    parser.add_argument("--resolution", type=int, default=1, help="Resolution of the recorded video (default: 1) (0: HD2K, 1: HD1080, 2: HD720, 3: VGA)")
    parser.add_argument("--close", type=float, default=0.15, help="Minimum depth perception distance (default: 0.15)")
    parser.add_argument("--far", type=float, default=10., help="Maximum depth perception distance (default: 10.)")
    parser.add_argument("--offset", type=str, default="/home/frankie/ros2_ws/src/py_pubsub/py_pubsub/offset.yaml", help="Path to the offset file (default: offset.yaml)")
    parser.add_argument("--width", type=int, default=1280, help="Width of the recorded video (default: 640)")
    parser.add_argument("--height", type=int, default=760, help="Height of the recorded video (default: 480)")
    parser.add_argument("--aruco_type", type=str, default="DICT_5X5_100", help="Type of the aruco dictionary (default: DICT_5X5_100)")
    parser.add_argument("--aruco_size", type=float, default=0.081, help="Size of the aruco markers (default: 0.1)")
    parser.add_argument("--debug_level", type=int, default=3, help="Debug level (default: 2)")
    args = parser.parse_args()
    server = zerorpc.Server(StepServer(args))
    # Bind the server to a specific address (e.g., localhost:4242)
    server.bind("tcp://0.0.0.0:4242")
    print("ZeroRPC server running on tcp://0.0.0.0:4242")
    server.run()

if __name__ == "__main__":
    main()