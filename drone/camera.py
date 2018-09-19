"""
Thread handling image acquisition and running image processing method
"""

from threading import Thread
import cv2
import cv2.aruco as aruco
import os
import logging
from time import sleep
from datetime import datetime
import numpy as np

import drone_globals as gl
from vision_utils import *

log = logging.getLogger()


class CameraThread(Thread):
    def __init__(self):
        Thread.__init__(self)

        self.mainFlag = True
        self.images_iterator = 0
        self.camera = None
        self.mtx = None
        self.mtx_inv = None
        self.dist = None

        self.init_camera()
        self.init_camera_directories()

        self.cam_attitude = np.radians(np.array([0, 90, 0]))

        log.debug('Camera {} initialized'.format(gl.args.camera_id))

    def init_camera(self):
        self.camera = cv2.VideoCapture(gl.args.camera_id)
        if not self.camera.isOpened():
            raise IOError('VideoCapture uninitialized')

        cv_file = cv2.FileStorage(gl.camera_calibration_path, cv2.FILE_STORAGE_READ)
        self.mtx = cv_file.getNode("camera_matrix").mat()
        self.dist = cv_file.getNode("dist_coeff").mat()
        cv_file.release()

        self.mtx_inv = np.linalg.inv(self.mtx)

    @staticmethod
    def init_camera_directories():
        if not os.path.exists(gl.camera_images_path):
            os.makedirs(gl.camera_images_path)

    def capture_image(self):
        time_string = datetime.now().strftime("%Y.%m.%d__%H.%M.%S")
        image_filename = gl.camera_images_path + time_string + ".png"

        ret, frame = self.camera.read()
        cv2.imwrite(image_filename, frame)

        return image_filename

    def run(self):
        while self.mainFlag:
            if gl.takeoff_event.wait(1):
                break

        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        font = cv2.FONT_HERSHEY_SIMPLEX

        while self.mainFlag:
            # Get image
            ret, frame = self.camera.read()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if np.all(ids is not None):

                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, self.mtx, self.dist)
                # (rvec-tvec).any()  # get rid of that nasty numpy value array error

                for i in range(0, ids.size):
                    aruco.drawAxis(frame, self.mtx, self.dist, rvec[i], tvec[i], 0.1)  # Draw Axis
                    # print get_angles(quad_center(corners[i]), self.mtx_inv)
                    cam_transform = np.matmul(euler2rotmat(self.cam_attitude), euler2rotmat(gl.pilot.attitude()))
                    rvecmat = np.zeros((3,3))
                    cv2.Rodrigues(rvec[i], rvecmat)

                    direction = get_direction(quad_center(corners[i]), self.mtx_inv)
                    direction = np.matmul(direction, cam_transform)
                    normal = np.matmul(np.array([0,0, 1]), rvecmat)
                    normal = np.matmul(normal, np.array([[0, 0, -1], [0, -1, 0], [1, 0, 0]]))
                    normal = np.matmul(normal, cam_transform)
                    # print 'normal ' + str(normal)
                    # TMP check
                    # normal = np.array([0, 0, -1])

                    delta_pos = direction * (-gl.pilot.position_ned()[2] / direction[0][2])
                    marker_pos = gl.pilot.position_ned() + delta_pos
                    target_offset = normal * (gl.alt / -normal[2])
                    wpt = marker_pos + target_offset
                    gl.pilot.goto_position_target_local_ned(wpt[0][0], wpt[0][1], wpt[0][2])
                    gl.pilot.set_roi(80, 20)
                    # print 'delta ' + str(wpt - gl.pilot.position_ned())

                aruco.drawDetectedMarkers(frame, corners)  # Draw a square around the markers

                strg = ''
                for i in range(0, ids.size):
                    strg += str(ids[i][0]) + ', '

                cv2.putText(frame, "Id: " + strg, (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            if gl.args.sitl_on:
                cv2.imshow('frame', frame)
                cv2.waitKey(1)

    def stop(self):
        self.mainFlag = False
        self.camera.release()
