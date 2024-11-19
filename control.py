'''
ROS Motor Control Node

Takes in feedback from all
other nodes. Based on sensor
information, executes
corresponding drive commands.
'''

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Quanser imports
from pal.products.qcar import QCar
from pal.utilities.gamepad import LogitechF710

# Standard imports
import sys
import time
import numpy as np
import matplotlib.pyplot as plt  # Import matplotlib for plotting

plt.ion() 

class Motor_Node(Node):

    def __init__(self):

        super().__init__('motor_node')


        self.detection_subscription = self.create_subscription(String, 'detection', self.detection_callback, 10)
        self.detection_subscription


        # Lists to store the data
        self.acceleration_list = []
        self.velocity_list = []
        self.distance_list = []
        self.confidence_list = []

        self.max_velocity = 0
        self.distance_travelled = 0

        #----------------------------------------#


    def write_list_to_file(self, list_name, list_data):
        # Create the filename based on the list name
        filename = f"{list_name}.txt"
        
        # Write the contents of the list to the file
        with open(filename, 'w') as file:
            for item in list_data:
                file.write(f"{item}\n")


    def detection_callback(self, msg):

        try:

            print("message heard")

            if msg.data == "Done":
                self.write_list_to_file(f'Vel={self.max_velocity} acceleration_list', self.acceleration_list)
                self.write_list_to_file(f'Vel={self.max_velocity} velocity_list', self.velocity_list)
                self.write_list_to_file(f'Vel={self.max_velocity} distance_list', self.distance_list)
                self.write_list_to_file(f'Vel={self.max_velocity} confidence_list', self.confidence_list)
                print(f'Distance traveled = {self.distance_travelled}')


                
                plt.clf()

                # Plot confidence vs distance
                plt.plot(self.distance_list, self.confidence_list, label='Confidence')
                plt.xlabel('Distance (m)')
                plt.ylabel('Confidence')
                plt.title('Confidence vs Distance')
                plt.suptitle(f'Max Velocity: {self.max_velocity:.2f} m/s')  # Add max velocity as a subtitle
                # Set constant axis limits
                plt.xlim(2, 0)  # x-axis range from 0 to 2
                plt.ylim(0, 1)  # y-axis range from 0 to 1
                plt.legend()
                plt.grid(True)
                plt.show(True)  # Display the plot

            else:

                self.get_logger().info('Data status is %s' % msg.data)
                confidence, acceleration, velocity, distance = map(float, msg.data.split(','))

                distance = 2 - distance

                self.acceleration_list.append(acceleration)
                self.velocity_list.append(velocity)
                self.distance_list.append(distance)
                self.confidence_list.append(confidence)
            
                # Track the maximum velocity
                if velocity > self.max_velocity:
                    self.max_velocity = velocity

                self.distance_travelled = distance

        except Exception as e:
            print(f'Exception: {e}')

    

    def destroy(self):

        super().destroy_node()


def main(args=None):
    #------- 2.5 EXECUTION PIPELINE -------#

    rclpy.init(args=args)
    motor_node = Motor_Node()
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()

    #----------------------------------------#


if __name__ == '__main__':
    main()
