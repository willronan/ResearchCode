'''lab4_publisher.py

This code will publish a simple
go or stop command based on keyboard
input.

'''

# Jetson Inference imports
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, cudaFromNumpy, cudaConvertColor, cudaAllocMapped

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Quanser imports
from pal.utilities.vision import Camera3D
from pal.products.qcar import QCar
import time
import numpy as np
#import cupy as cp
import os
from pal.utilities.gamepad import LogitechF710
import cv2
from skimage.measure import shannon_entropy

networkPath = "/home/nvidia/jetson-inference/python/training/detection/ssd/models/qsigns/"


def entropy(img):
    return shannon_entropy(img)

def fourier(img):
    f_transform = np.fft.fft2(img)
    f_shift = np.fft.fftshift(f_transform)  # Shift zero frequency to center
    magnitude_spectrum = 20 * np.log(np.abs(f_shift) + 1)

    # Analyze frequency components
    high_freq_power = np.mean(magnitude_spectrum[magnitude_spectrum > np.median(magnitude_spectrum)])
    low_freq_power = np.mean(magnitude_spectrum[magnitude_spectrum <= np.median(magnitude_spectrum)])

    return high_freq_power, low_freq_power

def edgeLengh(gray_image):
    edges = cv2.Canny(gray_image, threshold1=100, threshold2=200)
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    total_length = sum(cv2.arcLength(contour, True) for contour in contours)
    return total_length

def luminosity(gray_image):
    return gray_image.mean()

def deviation(gray_image):
    np.std(gray_image)


# Gamepad Node
class Detector_Node(Node):

    def __init__(self):

        super().__init__('detector_node')
        self.publisher_ = self.create_publisher(String, 'detection', 10)

        # Basic IO - write motor commands
        self.throttle = 0.04
        self.steering = -0.07
        self.runTime = 3.0  # seconds



        imageWidth = 1280
        imageHeight = 720

        # Connect to remote controller & car hardware
        self.camera = Camera3D(mode='RGB&DEPTH', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight)
        self.net = detectNet(model=networkPath + "ssd-mobilenet.onnx", 
            labels=networkPath + "labels.txt", input_blob="input_0", 
            output_cvg="scores", output_bbox="boxes", 
            threshold=0.2)
        #self.display = display = videoOutput("display://0") 
        self.myCar = QCar()
        self.gpad  = LogitechF710()
        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])


        self.perform_detection()


    # Detection system
    def perform_detection(self):

        try:

            # Time step for sensor measurements
            dt = 0.063

            # Initialize Sensor Data
            velocity = 0
            max_velocity = 0
            distance = 0

            # Initialize Message
            msg = String()

            standby = True


            directory = '/home/nvidia/ros2/detections'

            if not os.path.exists(directory):
                os.makedirs(directory)
                
            while True:


                while standby == True:

                    os.system('clear')
                    print("waiting")

                
                    new = self.gpad.read()

                    if self.gpad.buttonA == 1:
                        standby = False

                        total_time = 0
                        last_distance = 0
                        last_velocity = 0
                        total_distance = 0


                        print("starting...")



                        # -------------------------------------- #
                            # Do "warm up" detections
                        for i in range(10):  # Loop for 10 iterations
                        
                            
                            # Read camera
                            self.camera.read_RGB()
                            img = self.camera.imageBufferRGB

                            # Preprocess image
                            bgrImg = cudaFromNumpy(img, isBGR=True)
                            cudaImg = cudaAllocMapped(width=bgrImg.width, height=bgrImg.height, format='rgb8')
                            cudaConvertColor(bgrImg, cudaImg)

                            if cudaImg is None:  # capture timeout
                                continue

                            # Perform model detection
                            detections = self.net.Detect(cudaImg)
                        # -------------------------------------- #


                        #create a start time measurment and write motor commands
                        t0 = time.time()
                        self.myCar.write(self.throttle, self.steering, self.LEDs)
                    
                        #while time.time() - t0 < self.runTime:
                        while total_distance < 2:


                            t1 = time.time()

                            # Read from onboard sensors
                            self.myCar.read()
                            
                            # Read camera
                            self.camera.read_RGB()
                            img = self.camera.imageBufferRGB

                            #acceleration = self.myCar.accelerometer[0]
                            
                            #distance += velocity * dt
                            total_distance = self.myCar.motorEncoder[0] / 136815.453
                            distance = total_distance - last_distance
                            last_distance = total_distance
                                        
                            # Update velocity and distance
                            velocity = distance / dt
                            current_velo = velocity
                            last_velocity = velocity
                            


                            if velocity > max_velocity:
                                max_velocity = velocity

                            # Image statistics 

                            
                            high_freq_power, low_freq_power = fourier(img)
                            texture_entropy = entropy(img)

                            gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


                            length = edgeLengh(gray_image)
                            brightness = luminosity(gray_image)
                            contrast = deviation(gray_image)



                            




                            # Preprocess image
                            bgrImg = cudaFromNumpy(img, isBGR=True)
                            cudaImg = cudaAllocMapped(width=bgrImg.width, height=bgrImg.height, format='rgb8')
                            cudaConvertColor(bgrImg, cudaImg)


                            if cudaImg is None:  # capture timeout
                                continue


                            ###----------------------------### convert detection image back to numpy to be printed

                            #save_img = cp.asnumpy(cudaImg)


                            # Perform model detection
                            detections = self.net.Detect(cudaImg)


                            #if self.display:
                            #    self.display.Render(cudaImg)
                            #    self.display.SetStatus("Object Detection | Network {:.0f} FPS".format(self.net.GetNetworkFPS()))


                            #os.system('clear')
                            # Sort detections
                            if len(detections) == 1:
                                for detection in detections:
                                    confidence = detection.Confidence
                                    msg.data = f'{str(confidence)},{velocity},{total_distance},{low_freq_power},{high_freq_power},{texture_entropy},{length},{brightness},{contrast}'

                                    #if confidence < 0.30:
                                        #cv2.imwrite(os.path.join(directory, f'img_confidence{confidence}-distance{total_distance}-velocity{velocity}.png'), save_img)
                                    #elif confidence > 0.95:
                                        #cv2.imwrite(os.path.join(directory, f'img_confidence{confidence}-distance{total_distance}-velocity{velocity}.png'), save_img)
                                    #print(f'Confidence = {confidence} %')
                            elif len(detections) > 1: 
                                highest_confidence = max(detections, key=lambda detection: detection.Confidence).Confidence      
                                msg.data = f'{str(highest_confidence)},{velocity},{total_distance},{low_freq_power},{high_freq_power},{texture_entropy},{length},{brightness},{contrast}'
                                #print(f'Confidence = {highest_confidence} %')
                            else:
                                msg.data = f'0,{velocity},{total_distance},{low_freq_power},{high_freq_power},{texture_entropy},{length},{brightness},{contrast}'
                                #cv2.imwrite(os.path.join(directory, f'img_confidence{confidence}-distance{total_distance}-velocity{velocity}.png'), save_img)
                                #print(f'Confidence = 0 %')


                            # Publish drive command
                            #print(f"Publishing {msg.data}")
                            self.publisher_.publish(msg)
                            self.get_logger().info(f"Publishing {msg.data} to the detection topic")
                            #print(f'Distance = {distance} m')
                            #print(f'Velocity = {velocity} m/s')
                            #print(f'Acceleration = {acceleration} m/s2')
                            #print(f"Detection length: {len(detections)}")

                            t2 = time.time()''

                            dt = t2 - t1

                            #print(f'exectution time {dt}')



                            total_time += dt


                        # Stop the car after 3 seconds
                        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
                        self.myCar.write(0, 0, self.LEDs)
                        msg = String()
                        msg.data = "Done"
                        self.get_logger().info(f"Publishing {msg.data} to the detection topic")
                        self.publisher_.publish(msg)
                        print(f'total time was {total_time}')
                        print(f'total distance was {total_distance}')
                        print(f'max vel was {max_velocity}')


        except Exception as e:  # Catch any exception
            print(f'yo there was an errror {e}')  # Store the exception


        finally:

            print("destroy")
            self.destroy()


    # Safely disconnect hardware
    def destroy(self):
        self.camera.terminate()
        self.LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.myCar.write(0, 0, self.LEDs)
        super().destroy_node()

def main(args=None):

    # ROS exectution pipeline
    rclpy.init(args=args)
    detector_node = Detector_Node()
    rclpy.spin(detector_node)
    detector_node.destroy()
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
