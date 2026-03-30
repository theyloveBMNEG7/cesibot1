#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
import math

# Sharp GP2YOA21YKOF sensor specifications
# Range of the IR (10cm - 80cm)

IR_MIN_RANGE = 0.10  # meters
IR_MAX_RANGE = 0.80  # meters
IR_FOV = 0.087  # radians

class IrSensorNode(Node):
    
    
    # Creates a suscriber that read the raw voltage data sent from the teensy Micro-ROS
    def __init__(self):
        super().__init__('ir_sensor_node')
        self.create_subscription(
            Float32,
            '/robot/distance_voltage',
            self.voltage_callback,
            10
        )
        
        # create a publisher that publishes range messages for Nav2 stack using the sensor_msgs/Range 
        self.range_pub = self.create_publisher(
            Range,
            'ir_sensor/range',
            10
        )
        
        self.get_logger().info('IR sensor node started - Sharp GP2YOA21YKOF')
        
    def voltage_callback(self, msg):
        voltage = msg.data 
        
        if voltage < 0.4:
            distance_m = IR_MAX_RANGE
        elif voltage > 3.2:
            distance_m = IR_MIN_RANGE
        else: 
            distance_cm = 29.988 * (voltage ** -1.173)
            distance_m = distance_cm / 100.0 
            
        distance_m = max(IR_MIN_RANGE, min(IR_MAX_RANGE, distance_m))
        
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = 'ir_link'
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = IR_FOV
        range_msg.min_range = IR_MIN_RANGE
        range_msg.max_range = IR_MAX_RANGE
        range_msg.range = distance_m
        
        self.range_pub.publish(range_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = IrSensorNode()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()