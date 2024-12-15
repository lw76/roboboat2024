"""
This is the preliminary documentation for Kate-Documented.
"""

from serial import Serial
from pynmeagps import NMEAReader

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64 
from wamv_msgs.msg import GPS_data
import numpy 
import math
import time

# This class reads GPS data from a serial port and publishes it as a ROS topic
class GPS_Reader(Node):
    # reads and publishes GPS data as a ROS topic
    def __init__(self):
         """
        Initializes the GPS reader:
        - Sets up the publisher to send GPS data to ROS.
        - Sets up a timer to get data every 0.01 seconds.
        - Connects to the GPS device through serial communication.
        """
        
        super().__init__('gps_reader')
        self.lat_pub = self.create_publisher(GPS_data, '/wamv/sensors/gps', 10)
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.stream = Serial('/dev/ttyACM0', 115200, timeout=3)
    
    def timer_callback(self):
        """
        This function is called every 0.01 seconds:
        - Reads GPS data from the serial connection.
        - Converts the data into a format that ROS can send.
        - Sends the GPS data to the ROS topic '/wamv/sensors/gps'.
        """
        #print("timer callback")
        nmr = NMEAReader(self.stream)
        (raw_data, parsed_data) = nmr.read()
        #print("read")
        #print (parsed_data)
        try:
            # Create a GPS_data message to store the parsed GPS data
            #self.lat_pub.publish(parsed_data.lat)
            msg = GPS_data()
            #print(msg)
            msg.lat = parsed_data.lat
            msg.lon = parsed_data.lon
            msg.hdg = parsed_data.heading #
            msg.ts = time.time()

            self.lat_pub.publish(msg)
            print(msg)
        except Exception as e:
            #print(parsed_data)
            #print("exception")
            #print(e)
            pass

def main(args=None):
    """
    Main function to start the ROS system:
    - Initializes the system.
    - Starts the GPS reader node.
    - Keeps the node running until it's manually stopped.
    """
    rclpy.init(args = args)
    gps_reader = GPS_Reader()
    rclpy.spin(gps_reader)
    gps_reader.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
