from serial import Serial
from pynmeagps import NMEAReader

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64 
import numpy 
import math

# Class to read GPS data and publish latitude, longitude, and heading
class GPS_Reader(Node):
    def __init__(self):
         """
        Initializes the GPS reader node:
        - Sets up publishers for latitude, longitude, and heading as Float64 messages.
        - Creates a timer to read and publish GPS data at regular intervals.
        - Establishes a serial connection with the GPS device.
        """
        super().__init__('gps_reader')
        self.lat_pub = self.create_publisher(Float64, '/wamv/sensors/gps/lat', 10)
        self.lon_pub = self.create_publisher(Float64, '/wamv/sensors/gps/lon', 10)
        self.hdg_pub = self.create_publisher(Float64, '/wamv/sensors/gps/hdg', 10)
        #self.gps_pub = self.create_publisher(Float64, "gps", 10) #need to find data type for array of floats
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.stream = Serial('/dev/ttyACM0', 115200, timeout=3)
    def timer_callback(self):
        """
        Callback function executed at regular intervals:
        - Reads data from the GPS device using NMEAReader.
        - Extracts latitude, longitude, and heading from the parsed data.
        - Publishes each value to its respective topic.
        """
        #print("timer callback")
        nmr = NMEAReader(self.stream)
        (raw_data, parsed_data) = nmr.read()
        # try:
        #     msg=Float64()
        #     msg.data=parsed_data
        #     self.gps_pub.publish(msg)
    #     #print("read")
    #     #print (parsed_data)
        try:
            # Publish latitude to the corresponding topic
            #self.lat_pub.publish(parsed_data.lat)
            msg = Float64()
            #print(msg)
            msg.data = parsed_data.lat
            self.lat_pub.publish(msg)
            print(msg)
        except Exception as e:
            '''ignore errors if latitude is missing'''
            #print(parsed_data)
            #print("exception")
            #print(e)
            pass
        try:
            ''''Publish longitude to the corresponding topic'''
            msg = Float64()
            msg.data = parsed_data.lon
            self.lon_pub.publish(msg)
            print(msg)
        except:
            '''ignore errors if longitude is missing'''
            pass
        try:
            '''Publish heading to the corresponding topic'''
            msg = Float64()
            msg.data = parsed_data.heading
            self.hdg_pub.publish(msg)
            print(msg)
        except Exception as e:
            '''ignore errors if heading is missing'''
            #print(parsed_data)
            #print("exception")
            #print(e)
            pass
def main(args=None):
     '''Main function to start the ROS node:
    - Initializes the ROS 2 system.
    - Creates and starts the GPS reader node.
    - Keeps the node running until manually stopped.'''
    
    rclpy.init(args = args)
    gps_reader = GPS_Reader()
    rclpy.spin(gps_reader)
    gps_reader.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
