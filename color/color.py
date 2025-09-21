import lcm
import sys
import time
from threading import Thread, Lock
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from scipy.signal import savgol_filter
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
import cv2


class Robot_Ctrl(object):
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_responce)
        self.send_thread = Thread(target=self.send_publish)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.mode_ok = 0
        self.gait_ok = 0
        self.runing = 1

    def run(self):
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg = robot_control_response_lcmt().decode(data)
        if (self.rec_msg.order_process_bar >= 95):
            self.mode_ok = self.rec_msg.mode
        else:
            self.mode_ok = 0

    def rec_responce(self):
        while self.runing:
            self.lc_r.handle()
            time.sleep(0.002)

    def Wait_finish_15(self, mode, gait_id):
        count = 0
        while self.runing and count < 3000:  # 10s
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1

    def Wait_finish_10(self, mode, gait_id):
        count = 0
        while self.runing and count < 2000:  # 10s
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1

    def Wait_finish_5(self, mode, gait_id):
        count = 0
        while self.runing and count < 1000:  # 5s
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1

    def Wait_finish_3(self, mode, gait_id):
        count = 0
        while self.runing and count < 600:  # 3s
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1

    def send_publish(self):
        while self.runing:
            self.send_lock.acquire()
            if self.delay_cnt > 20:  # Heartbeat signal 10HZ, It is used to maintain the heartbeat when life count is not updated
                self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
                self.delay_cnt = 0
            self.delay_cnt += 1
            self.send_lock.release()
            time.sleep(0.005)

    def Send_cmd(self, msg):
        self.send_lock.acquire()
        self.delay_cnt = 50
        self.cmd_msg = msg
        self.send_lock.release()

    def quit(self):
        self.runing = 0
        self.rec_thread.join()
        self.send_thread.join()


class ColorComparison(Node):
    def __init__(self):
        super().__init__('color_comparison')
        self.bridge = CvBridge()
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(
            Image,
            '/rgb_camera/image_raw',
            self.image_callback,
            qos_profile)
        self.subscription  # 防止未使用变量的警告
        self.result = None
        self.data_lock = Lock()  # 用于保护共享数据的锁

    def image_callback(self, msg):
        # 将ROS 2的图像消息转换为OpenCV图像
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 调用颜色比较函数
        result = self.compare_colors(cv_image)
        with self.data_lock:
            self.result = result

    def compare_colors(self, cv_image):
        # 将BGR图像转换为RGB图像
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # 分离红色、绿色和蓝色通道
        red_channel = rgb_image[:, :, 0]
        green_channel = rgb_image[:, :, 1]
        blue_channel = rgb_image[:, :, 2]

        # 计算各颜色通道的总和
        total_red = np.sum(red_channel)
        total_green = np.sum(green_channel)
        total_blue = np.sum(blue_channel)

        # 找出最强的颜色
        max_color = max(total_red, total_green, total_blue)
        
        if max_color == total_red:
            return "红色：直行", total_red
        elif max_color == total_green:
            return "绿色：右转", total_green
        elif max_color == total_blue:
            return "蓝色：左转", total_blue
        else:
            return "未知颜色", 0

    def get_result(self):
        with self.data_lock:
            return self.result

def standup():  # 站立
    msg.mode = 12  
    msg.gait_id = 0
    msg.life_count += 1  
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish_10(12, 0)

def go_straight(vel, step, duration):  # 前进
    msg.mode = 11
    msg.gait_id = 3
    msg.vel_des = [vel, 0, 0]
    msg.step_height = [step, step]
    msg.duration = duration
    msg.life_count += 1
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish_5(11, 3)

def turn(vel, step, duration):  # 转向
    msg.mode = 11
    msg.gait_id = 3
    msg.vel_des = [0, 0, vel]
    msg.step_height = [step, step]
    msg.duration = duration
    msg.life_count += 1
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish_5(11, 3)

def color_detection_behavior():
    # 站立准备
    standup()
    
    # 主循环
    while True:
        result = color_comparison.get_result()
        if result is not None:
            color_name, _ = result
            print(f"检测到{color_name}")
            
            if color_name == "红色：直行":
                # 红色：快走
                go_straight(1.0, 0.06, 1200)
            elif color_name == "绿色：右转":
                # 绿色：右转
                turn(-0.64, 0.06, 3000)
            elif color_name == "蓝色：左转":
                # 蓝色：左转
                turn(0.64, 0.06, 3000)
            else:
                # 未知颜色：停止
                msg.mode = 7
                msg.gait_id = 0
                msg.life_count += 1
                Ctrl.Send_cmd(msg)
                Ctrl.Wait_finish_5(7, 0)
        
        # 短暂延迟，避免过于频繁的检测
        time.sleep(0.2)

def spin_executor():
    try:
        executor.spin()
    finally:
        # 确保节点在程序退出时被正确销毁
        color_comparison.destroy_node()

        rclpy.shutdown()


Ctrl = Robot_Ctrl()
Ctrl.run()
msg = robot_control_cmd_lcmt()

rclpy.init()
color_comparison = ColorComparison()


executor = MultiThreadedExecutor()

# 添加节点到执行器
executor.add_node(color_comparison)


spin_thread = Thread(target=spin_executor)
spin_thread.start()

def main():
    try:
        color_detection_behavior()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        spin_thread.join()
        Ctrl.quit()
        sys.exit()


if __name__ == '__main__':
    main()
