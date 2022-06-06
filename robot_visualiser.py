#import os
#import sys
import rclpy
from rclpy.node import Node
#from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from std_msgs.msg import String
import time
from geometry_msgs.msg import Twist

import math

class RobotVisualiser(Node):

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = sy * cp * cr - cy * sp * sr
        q[2] = cy * cp * sr - sy * sp * cr
        q[3] = sy * cp * sr + cy * sp * cr

        return q
        


    def left_callback(self, msg):
        global left
        try:
            left = int(msg.data[:-2]) #remove the 'cm' and convert to int.
        except:
            print("error")
        if (left < 30):
            self.spawnWall()

    def right_callback(self, msg):
        global right
        try:
            right = int(msg.data[:-2]) #remove the 'cm' and convert to int.
        except:
            print("error")
        if (right < 30):
            self.spawnWall()

    def front_callback(self, msg):
        global front
        try:
            front = int(msg.data[:-2]) #remove the 'cm' and convert to int.
        except:
            print("error")
            
        if (front < 30):
            self.spawnWall()
            
    def spawnWall(self):
        global front
        global left
        global right
        global x
        global y
        global z
        global qx
        global qy
        global qz
        global counter
        
        
        client = self.create_client(SpawnEntity, "/spawn_entity")
        delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        if (front < 60):
            if (counter == 6):
                delete_request = DeleteEntity.Request()
                delete_request.name = "wall"
                future = delete_client.call_async(delete_request)
                counter = 0
        
            request = SpawnEntity.Request()
            request.name = 'wall'
            request.xml = "<?xml version='1.0'?><sdf version='1.4'><model name='my_model'><pose>" + str(0.5 + x + (0.04 * front)) + " " + str(y) + " " + str(0.5) + " " + str(0) + " " + str(0) + " " + str(0) + "</pose><static>true</static><link name='link'><collision name='collision'><geometry><box><size>0.3 1 1</size></box></geometry></collision><visual name='visual'><geometry><box><size>0.3 1 1</size></box></geometry></visual></link></model></sdf>"
            request.robot_namespace = "demo"

            self.get_logger().info("Sending service request to `/spawn_entity`")
            future = client.call_async(request)
            counter = counter + 1
            
        if (left < 60):
            if (counter == 6):
                delete_request = DeleteEntity.Request()
                delete_request.name = "wall2"
                future = delete_client.call_async(delete_request)
                counter = 0
        
            request = SpawnEntity.Request()
            request.name = 'wall2'
            request.xml = "<?xml version='1.0'?><sdf version='1.4'><model name='my_model'><pose>" + str(0.5 + x + (0.04 * left)) + " " + str(y + 0.5 + (0.04 * left)) + " " + str(0.5) + " " + str(0) + " " + str(0) + " " + str(-1) + "</pose><static>true</static><link name='link'><collision name='collision'><geometry><box><size>1 0.3 1</size></box></geometry></collision><visual name='visual'><geometry><box><size>1 0.3 1</size></box></geometry></visual></link></model></sdf>"
            request.robot_namespace = "demo"

            self.get_logger().info("Sending service request to `/spawn_entity`")
            future = client.call_async(request)
            counter = counter + 1
            
            
        if (right < 60):
            if (counter == 6):
                delete_request = DeleteEntity.Request()
                delete_request.name = "wall3"
                future = delete_client.call_async(delete_request)
                counter = 0
        
            request = SpawnEntity.Request()
            request.name = 'wall3'
            request.xml = "<?xml version='1.0'?><sdf version='1.4'><model name='my_model'><pose>" + str(0.5 + x + (0.04 * right)) + " " + str(y -  0.5 - (0.04 * right)) + " " + str(0.5) + " " + str(0) + " " + str(0) + " " + str(0.5) + "</pose><static>true</static><link name='link'><collision name='collision'><geometry><box><size>1 0.3 1</size></box></geometry></collision><visual name='visual'><geometry><box><size>1 0.3 1</size></box></geometry></visual></link></model></sdf>"
            request.robot_namespace = "demo"
            

            self.get_logger().info("Sending service request to `/spawn_entity`")
            future = client.call_async(request)
            counter = counter + 1
    
    


    def listener_callback(self, msg):
        global x
        global y
        global z
        global qx
        global qy
        global qz
        
        #now move the object
        client = self.create_client(SetEntityState, "/world/set_entity_state")

        self.get_logger().info("Connecting to `/world/set_entity_state` service...")

        # Set data for request
        request = SetEntityState.Request()
        entitystate = EntityState()
        entitystate.name= "robot"
        request.state = entitystate
        request.state.pose.position.x = msg.linear.x
        request.state.pose.position.y = msg.linear.y
        request.state.pose.position.z = msg.linear.z
        q = self.quaternion_from_euler(msg.angular.x,msg.angular.y,msg.angular.z)
        request.state.pose.orientation.x = q[0]
        request.state.pose.orientation.y = q[1]
        request.state.pose.orientation.z = q[2]
        request.state.pose.orientation.w = q[3]
        print(request)
        self.get_logger().info("Sending service request to `/world/set_entity_state`")
        future = client.call_async(request)
        
        x = request.state.pose.position.x
        y = request.state.pose.position.y
        z = request.state.pose.position.z
        qx = request.state.pose.orientation.x
        qy = request.state.pose.orientation.y
        qz = request.state.pose.orientation.z
            

    def __init__(self):
        global front
        global right
        global left
        global x
        global y
        global counter
        global z
        global qx
        global qy
        global qz
        x = 0
        y = 0
        z = 0
        qx = 0
        qy = 0
        qz = 0
        front = 50
        left = 50
        right = 50
        counter = 0
        
        super().__init__('robot_visualiser')

        #first spawn
        self.get_logger().info(
            'Creating Service client to connect to `/spawn_entity`')
        client = self.create_client(SpawnEntity, "/spawn_entity")

        self.get_logger().info("Connecting to `/spawn_entity` service...")
        if not client.service_is_ready():
            client.wait_for_service()
            self.get_logger().info("...connected!")

        # Set data for request
        request = SpawnEntity.Request()
        request.name = 'robot'
        request.xml = "<?xml version='1.0'?><sdf version='1.4'><model name='my_model'><pose>0 0 0.5 0 0 0</pose><static>true</static><link name='link'><collision name='collision'><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='visual'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></sdf>"
        request.robot_namespace = "demo"

        self.get_logger().info("Sending service request to `/spawn_entity`")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print('response: %r' % future.result())
        else:
            raise RuntimeError(
                'exception while calling service: %r' % future.exception())

        #subscribe to the post channel.
        self.subscription = self.create_subscription(
                Twist,
                '/robot/pose',
                self.listener_callback,
                10)
        self.subscription  # prevent unused variable warning
        
        self.subscription = self.create_subscription(
            String,
            '/robot/left',
            self.left_callback,
            1)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            String,
            '/robot/front',
            self.front_callback,
            1)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            String,
            '/robot/right',
            self.right_callback,
            1)
        self.subscription  # prevent unused variable warning


def main(args=None):

    rclpy.init(args=args)

    visualiser = RobotVisualiser()
    rclpy.spin(visualiser)

    visualiser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
