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
from skimage.measure import shannon_entropy

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

        texture_entropy = shannon_entropy(img)

        os.system('clear')
        print(f"Image Entropy = {texture_entropy}")

        cv2.imshow("Original Image", img)


        cv2.waitKey(100)
