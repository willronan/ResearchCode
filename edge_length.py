'''hardware_test_intelrealsense.py

This example demonstrates how to read and display depth & RGB image data
from the Intel Realsense camera.
'''
import time
import cv2
import numpy as np
from pal.products.qcar import QCarRealSense, IS_PHYSICAL_QCAR
from pal.utilities.vision import Camera3D
from hal.utilities.image_processing import ImageProcessing
import os

MAX_DISTANCE = 1.2  # pixels in RGB image farther than this will appear white
MIN_DISTANCE = 0.0001  # pixels in RGB image closer than this will appear black

if not IS_PHYSICAL_QCAR:
    import qlabs_setup
    qlabs_setup.setup()


#Initial SetupTr
runTime = 100.0 # seconds
max_distance = 2 # meters (for depth camera)

with QCarRealSense(mode='RGB, Depth') as myCam:
    t0 = time.time()
    while time.time() - t0 < runTime:

        myCam.read_RGB()

        img = myCam.imageBufferRGB

        #blurred = cv2.GaussianBlur(img, (3, 3), 0)

        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(gray_image, threshold1=100, threshold2=200)
        

        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        total_length = sum(cv2.arcLength(contour, True) for contour in contours)

        os.system('clear')
        print(f"Total contour length = {total_length}")


        edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        edges_colored[edges == 255] = [0, 0, 255]
        highlighted_image = cv2.addWeighted(img, 0.8, edges_colored, 0.5, 0)
        cv2.imshow("Highlighted Edges", highlighted_image)


        cv2.waitKey(100)
