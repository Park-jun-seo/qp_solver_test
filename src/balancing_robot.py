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
from sensor_msgs.msg import Imu


pos = {
    "foward": 0,
    "trun": 0,
}

joint = [
    "foward"
    "trun"
]

class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')

        self.paused = False
        self.wheel_velocity = 0.0
        self.goal_velocity = 0
        self.ctrl = 0.0
        self.subscribe_to_joints()

        self.Kp = 0.2  # 비례 항
        self.Ki = 1 # 적분 항
        self.Kd = 0.002  # 미분 항

        self.previous_error = 0.0
        self.integral_error = 0.0

        # self.pub_jointstate = self.create_publisher(JointState,"/joint_states", 10)
        # self.pub_odom = self.create_publisher(Pose2D,"/robot_pos", 10)
        self.clock_pub = self.create_publisher(Clock,"/clock", 10)
        self.pub_imu = self.create_publisher(Imu,"/imu/data", 1)
        self.pub_wheel = self.create_publisher(Float64,"/wheel/data", 1)

    def subscribe_to_joints(self):
        self.create_subscription(Float64,"/foward", self.foward_callback, 10)
        # self.create_subscription(Float64,"/trun", self.trun_callback, 10)

            

    def mujoco(self):
        current_script_path = os.path.abspath(__file__)
        root = os.path.dirname(os.path.dirname(os.path.dirname(current_script_path))) + "/share/qp_tutorial/mjcf/scene.xml"
        m = mujoco.MjModel.from_xml_path(root)
        d = mujoco.MjData(m)
        with mujoco.viewer.launch_passive(m, d, key_callback=self.key_callback) as viewer:
            start_time = time.time()
            zero_time = d.time
            while 1:
                step_start = time.time()
                # d.ctrl[0] = self.goal_velocity
                # d.ctrl[1] = pos["trun"]
                dt = m.opt.timestep
                self.wheel_velocity = (d.sensor("jointvel_l").data.copy()[0] * -1 + d.sensor("jointvel_r").data.copy()[0] * -1)/2
                control_signal = -1 * self.calculate_pid(dt)
                d.ctrl[0] = control_signal 
                self.publish_sensors(m, d, self.pub_imu)

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
              
                
                # print(self.wheel_velocity)
                rclpy.spin_once(self, timeout_sec=0.0)

    def publish_sensors(self, m, d,pub_imu):

        # IMU 데이터 처리
        imu_msg = Imu()
        imu_msg.orientation.x = d.sensor("framexaxis_body_main").data.copy()[1].item()
        imu_msg.orientation.y = d.sensor("framexaxis_body_main").data.copy()[2].item()
        imu_msg.orientation.z = d.sensor("framexaxis_body_main").data.copy()[3].item()
        imu_msg.orientation.w = d.sensor("framexaxis_body_main").data.copy()[0].item()
        imu_msg.angular_velocity.x = d.sensor("angular_velocity").data.copy()[0].item()
        imu_msg.angular_velocity.y = d.sensor("angular_velocity").data.copy()[1].item()
        imu_msg.angular_velocity.z = d.sensor("angular_velocity").data.copy()[2].item()
        imu_msg.linear_acceleration.x = d.sensor("linear_acceleration").data.copy()[0].item()
        imu_msg.linear_acceleration.y = d.sensor("linear_acceleration").data.copy()[1].item()
        imu_msg.linear_acceleration.z = d.sensor("linear_acceleration").data.copy()[2].item()
        # quat = [imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z] 
        # yaw = np.arctan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2]**2 + quat[3]**2))
        # pitch = np.arcsin(np.clip(2.0 * (quat[0] * quat[2] - quat[3] * quat[1]), -1.0, 1.0))
        # roll = np.arctan2(2.0 * (quat[0] * quat[1] + quat[2] * quat[3]), 1.0 - 2.0 * (quat[1]**2 + quat[2]**2))

        # 각도를 도(degree) 단위로 변환
        # yaw_deg = np.degrees(yaw)
        # pitch_deg = np.degrees(pitch)
        # roll_deg = np.degrees(roll)

        # print(f"Roll: {roll_deg}, Pitch: {pitch_deg}, Yaw: {yaw_deg}")
        # 메시지 발행
        pub_imu.publish(imu_msg)
        float_msg = Float64()
        float_msg.data = self.wheel_velocity
        self.pub_wheel.publish(float_msg)

    def foward_callback(self,msg):
        # pos["foward"] = msg.data
        self.goal_velocity = msg.data 
    # def trun_callback(self,msg):
    #     pos["trun"] = msg.data

    def key_callback(self,keycode):
     
        if chr(keycode) == ' ':
          if self.paused == True:
              self.paused = False
          else :
              self.paused = True

    def calculate_pid(self, dt):
        """
        PID 제어 계산을 수행합니다.
        """
        error = self.goal_velocity - self.wheel_velocity
        self.integral_error += error * dt
        derivative_error = (error - self.previous_error) / dt

        # PID 컨트롤러로부터 제어 신호 계산
        control = (self.Kp * error) + (self.Ki * self.integral_error) + (self.Kd * derivative_error)

        self.previous_error = error

        return control
    
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