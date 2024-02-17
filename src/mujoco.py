#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
import sys
import time
import mujoco
import mujoco.viewer
import numpy as np
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Pose2D


pos = {
    "joint1": 0,
}

joint = [
    "joint1"
]

class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')

        self.paused = False

        self.subscribe_to_joints()

        self.pub_jointstate = self.create_publisher(JointState,"/joint_states", 10)
        self.pub_odom = self.create_publisher(Pose2D,"/robot_pos", 10)
        self.clock_pub = self.create_publisher(Clock,"/clock", 10)

    def subscribe_to_joints(self):
        self.create_subscription(JointState,"/joint_states", self.joint_states_callback, 10)

            

    def mujoco(self):
        current_script_path = os.path.abspath(__file__)
        root = os.path.dirname(os.path.dirname(os.path.dirname(current_script_path))) + "/share/mujoco_fold/mjcf/scene.xml"
        m = mujoco.MjModel.from_xml_path(root)
        d = mujoco.MjData(m)
        with mujoco.viewer.launch_passive(m, d, key_callback=self.key_callback) as viewer:
            start_time = time.time()
            zero_time = d.time
            while 1:
                step_start = time.time()
                d.ctrl[joint.index("joint1")] = pos["joint1"]

                self.publish_sensors(m, d, self.pub_jointstate,self.pub_ft_sensor, self.pub_imu, self.pub_odom)

                if not self.paused:
                    mujoco.mj_step(m, d)
                    viewer.sync()


                time_until_next_step = m.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

                # clock_msg = Clock()        
                # time_sec = d.time - zero_time
                # clock_msg.clock.sec = int(time_sec)
                # clock_msg.clock.nanosec = int((time_sec - int(time_sec)) * 1e9)
                # self.clock_pub.publish(clock_msg)
                
                # elapsed_real_time = time.time() - start_time
                # time_factor = d.time / elapsed_real_time if elapsed_real_time != 0 else 0
                # print("time_factor : ",time_factor)

                rclpy.spin_once(self, timeout_sec=0.0)

    def publish_sensors(self, m, d, pub_jointstate, pub_odom):
        joint_msg = JointState()
        joint_msg.name = joint
        joint_msg.position = [d.joint(i).qpos[0] for i in joint]
        joint_msg.velocity = [d.joint(i).qvel[0] for i in joint]
        joint_msg.effort = [d.joint(i).qfrc_smooth[0] for i in joint]
   
        # r1 = d.xmat[1][0]
        # r2 = d.xmat[1][3]
        # yaw = math.atan2(r2, r1)
        # yaw_degrees = yaw * (180.0 / math.pi)
        # if yaw_degrees < 0:
        #     yaw_degrees += 360
        # pose2d_msg = Pose2D()
        # pose2d_msg.x = d.sensor("framepos_body_main").data[0]
        # pose2d_msg.y = d.sensor("framepos_body_main").data[1]
        # pose2d_msg.theta = yaw_degrees

        # 메시지 발행
        pub_jointstate.publish(joint_msg)
        # pub_odom.publish(pose2d_msg)

    def joint_states_callback(self,msg):
        for i, name in enumerate(msg.name):
            if name in pos:
                pos[name] = msg.position[i]

    def key_callback(self,keycode):
     
        if chr(keycode) == ' ':
          if self.paused == True:
              self.paused = False
          else :
              self.paused = True
    
        

def main(args=None):
    rclpy.init(args=args)
    mujoco_node = MujocoNode()

    try:
        mujoco_node.mujoco()  # Mujoco 시뮬레이션 시작
    except KeyboardInterrupt:
        pass
    finally:
        mujoco_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()