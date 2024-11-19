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
import csv

plt.ion() 

class Motor_Node(Node):

    def __init__(self):

        super().__init__('motor_node')


        self.detection_subscription = self.create_subscription(String, 'detection', self.detection_callback, 10)
        self.detection_subscription


        # Lists to store the data
        self.velocity_list = []
        self.distance_list = []
        self.confidence_list = []
        self.low_freq_power_list = []
        self.high_freq_power_list = []
        self.texture_entropy_list = []
        self.brightness_list = []
        self.length_list = []
        self.contrast_list = []

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

    def write_data_to_csv(self, filename, confidence_list, distance_list, velocity_list, low_freq_power_list, high_freq_power_list, texture_entropy_list, length_list, brightness_list, contrast_list):
        # Combine all lists into rows for CSV writing
        rows = zip(confidence_list, distance_list, velocity_list, low_freq_power_list, high_freq_power_list, texture_entropy_list, length_list, brightness_list, contrast_list)
        
        # Define the header for the CSV file
        header = ['Confidence', 'Distance', 'Velocity', 'Low Power Freq', 'High Power Freq', 'Entropy', 'Brightness', 'Length', 'Contrast']
        
        # Open the file and write the data
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            
            # Write the header row
            writer.writerow(header)
            
            # Write the data rows
            writer.writerows(rows)


    def detection_callback(self, msg):

        try:

            print("message heard")

            if msg.data == "Done":
                self.write_data_to_csv(
                    filename=f'velocity={self.max_velocity}.csv',
                    confidence_list=self.confidence_list,
                    distance_list=self.distance_list,
                    velocity_list=self.velocity_list,
                    low_freq_power_list=self.low_freq_power_list,
                    high_freq_power_list=self.high_freq_power_list,
                    texture_entropy_list=self.texture_entropy_list,
                    length_list=self.length_list,
                    brightness_list=self.brightness_list,
                    contrast_list=self.contrast_list

                )
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
                confidence, velocity, distance, low_freq_power, high_freq_power, texture_entropy, length, brightness, contrast  = map(float, msg.data.split(','))

                distance = 2 - distance

                self.confidence_list.append(confidence)
                self.velocity_list.append(velocity)
                self.distance_list.append(distance)
                self.low_freq_power_list.append(low_freq_power)
                self.high_freq_power_list.append(high_freq_power)
                self.texture_entropy_list.append(texture_entropy)
                self.length.append(length)
                self.brightness.append(brightness)
                self.contrast_list.append(contrast)


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
