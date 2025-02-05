import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

import serial

from serial import SerialException





class FramePublisher(Node):

    def __init__(self):
        super().__init__('padman_wii_tf_goal_publisher')

        print("Init")
        ser_0 = serial.Serial('/dev/ttyUSB0', 115200, timeout = 0.01)
        ser_1 = serial.Serial('/dev/ttyUSB2', 115200, timeout = 0.01)


        # # Declare and acquire `turtlename` parameter
        # self.turtlename = self.declare_parameter(
        #   'turtlename', 'turtle').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # # callback function on each message
        # self.subscription = self.create_subscription(
        #     Pose,
        #     f'/{self.turtlename}/pose',
        #     self.handle_turtle_pose,
        #     1)
        # self.subscription  # prevent unused variable warning

        self.scale = np.array([0.1, 0.1, 0.1]) #np.array([0.05, 0.05, 0.05])

        self.target_origin_l = np.array([0.06, 0.22, -0.03])
        self.target_frame_l = "base_link"
        
        self.target_origin_r = np.array([0.10, 0.22, -0.03])
        self.target_frame_r = "base_link"

        self.t_last_update_r = self.get_clock().now()
        self.t_last_update_l = self.get_clock().now()

        self.z_r = 0.0
        self.z_l = 0.0

        while rclpy.ok():
            # print("---")
            try:
                s0 = ser_0.readline().decode("utf-8")
                self.send_wii_as_transform(s0)
            except SerialException as e:
                print(".\n")

            try:
                s1 = ser_1.readline().decode("utf-8")
                self.send_wii_as_transform(s1)
            except SerialException as e:
                print(".\n")

            
            # print("Received on s0:", s0)
            # print("Received on s1:", s1)

            
            

    def send_wii_as_transform(self, s: str):
        
        side, data = s.split(":")
        x,y,button0,button1 = data.strip().split(",")
        
        x = np.clip(float(x)/100., -1., 1.)
        y = np.clip(float(y)/100., -1., 1.)

        #print(button0, "\t", button1, "\t", button0 == "1", "\t", button1 == "1")

        button0 = button0 == "1"
        button1 = button1 == "1"

        # print(button0, type(button0))
        v_z = 0.4 * button0 - 0.4 * button1
        t_now = self.get_clock().now()
        if side == "r":
            dt = t_now - self.t_last_update_r
            # print(dt.nanoseconds / 1e9, self.z_r)
            self.z_r = np.clip(self.z_r + dt.nanoseconds / 1e9 * v_z, -1., 1.)
            z=self.z_r
            self.t_last_update_r = t_now

        elif side == "l":

            dt = t_now - self.t_last_update_l
            # print(dt.nanoseconds / 1e9, self.z_l)
            self.z_l = np.clip(self.z_l + dt.nanoseconds / 1e9 * v_z, -1., 1.)
            z=self.z_l
            self.t_last_update_l = t_now
        else:
            raise Exception("Side needs to be l or r. Is: "+side)


        delta = np.array([x, y, z]) * self.scale

        if side == "r":
            target = self.target_origin_r + delta
            parent = self.target_frame_r
            frame_id = "ee_target_r"
        elif side == "l":
            target = self.target_origin_l + delta
            parent = self.target_frame_l
            frame_id = "ee_target_l"
        else:
            raise Exception("Side needs to be l or r. Is: "+side)

        self.get_logger().info('Publishing target frame_id: '+str(target), throttle_duration_sec=1)

        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = frame_id

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = target[0]
        t.transform.translation.y = target[1]
        t.transform.translation.z = target[2]

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        #q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = 0.#q[0]
        t.transform.rotation.y = 0.#q[1]
        t.transform.rotation.z = 0.#q[2]
        t.transform.rotation.w = 1.#q[3]


        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()