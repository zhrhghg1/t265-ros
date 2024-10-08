#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
import serial
import math
import tf

class T265RobotController:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node('t265_robot_controller', anonymous=True)

        # 物理模型参数
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.05)  # 轮子半径 (m)
        self.wheel_base = rospy.get_param('~wheel_base', 0.3)       # 两轮间距 (m)
        self.max_rpm = rospy.get_param('~max_rpm', 40.0)            # 最大电机转速 (RPM)

        # PID控制参数
        self.kp_linear = rospy.get_param('~kp_linear', 1.0)
        self.ki_linear = rospy.get_param('~ki_linear', 0.0)
        self.kd_linear = rospy.get_param('~kd_linear', 0.0)

        self.kp_angular = rospy.get_param('~kp_angular', 1.0)
        self.ki_angular = rospy.get_param('~ki_angular', 0.0)
        self.kd_angular = rospy.get_param('~kd_angular', 0.0)

        # 容忍度参数
        self.position_tolerance = rospy.get_param('~position_tolerance', 0.05)  # 位置误差容忍度 (m)
        self.angle_tolerance = rospy.get_param('~angle_tolerance', 0.05)        # 角度误差容忍度 (rad)

        # 串口设置
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            rospy.loginfo("串口已打开：{}".format(self.serial_port))
        except serial.SerialException as e:
            rospy.logerr("无法打开串口: {}".format(e))
            exit(1)

        # 初始目标位姿（x, y, theta）
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0

        # 当前位姿（x, y, theta）
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # PID控制误差变量
        self.linear_error = 0.0
        self.angular_error = 0.0

        # 订阅T265的里程计数据
        self.odom_sub = rospy.Subscriber("/t265/odom/sample", Odometry, self.odom_callback)

        # 订阅目标位置话题
        self.goal_sub = rospy.Subscriber("/goal", Pose2D, self.goal_callback)

        # 控制循环
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def odom_callback(self, msg):
        # 从T265里程计数据获取当前位置和朝向
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

    def goal_callback(self, msg):
        # 从接收到的目标位置设置目标x, y和theta
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_theta = msg.theta
        rospy.loginfo("接收到新的目标位置: x={:.2f}, y={:.2f}, theta={:.2f}".format(
            self.target_x, self.target_y, self.target_theta))

    def get_yaw_from_quaternion(self, orientation):
        # 提取四元数中的yaw（偏航角度）
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw

    def normalize_angle(self, angle):
        # 将角度规范化到[-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def calculate_pid(self, error, kp, ki, kd, dt, prev_error, integral):
        # 经典PID公式
        derivative = (error - prev_error) / dt
        integral += error * dt
        output = kp * error + ki * integral + kd * derivative
        return output, error, integral

    def control_loop(self, event):
        # 控制循环
        if event.last_real is None:
            dt = 0.1  # 初始dt值，与Timer的周期一致
        else:
            dt = (event.current_real - event.last_real).to_sec()
        if dt == 0:
            dt = 0.1  # 防止除以零

        # 计算当前位置与目标位置的误差
        delta_x = self.target_x - self.current_x
        delta_y = self.target_y - self.current_y
        self.linear_error = math.hypot(delta_x, delta_y)
        desired_theta = math.atan2(delta_y, delta_x)
        self.angular_error = self.normalize_angle(desired_theta - self.current_theta)

        # 添加停止条件
        if self.linear_error < self.position_tolerance and abs(self.angular_error) < self.angle_tolerance:
            # 误差足够小，停止机器人
            rpm_left = 0.0
            rpm_right = 0.0
            rospy.loginfo("已到达目标位置")
        else:
            # PID控制器输出计算
            linear_output, self.prev_linear_error, self.linear_integral = self.calculate_pid(
                self.linear_error, self.kp_linear, self.ki_linear, self.kd_linear,
                dt, getattr(self, 'prev_linear_error', 0.0), getattr(self, 'linear_integral', 0.0))

            angular_output, self.prev_angular_error, self.angular_integral = self.calculate_pid(
                self.angular_error, self.kp_angular, self.ki_angular, self.kd_angular,
                dt, getattr(self, 'prev_angular_error', 0.0), getattr(self, 'angular_integral', 0.0))

            # 计算左右轮的线速度 (m/s)
            v_left = linear_output - angular_output * (self.wheel_base / 2.0)
            v_right = linear_output + angular_output * (self.wheel_base / 2.0)

            # 将线速度转换为RPM
            rpm_left = (v_left / (2.0 * math.pi * self.wheel_radius)) * 60.0
            rpm_right = (v_right / (2.0 * math.pi * self.wheel_radius)) * 60.0

            # 限制RPM在最大范围内
            rpm_left = max(min(rpm_left, self.max_rpm), -self.max_rpm)
            rpm_right = max(min(rpm_right, self.max_rpm), -self.max_rpm)

        # 串口输出
        output_str = "{:.2f},{:.2f}\n".format(rpm_left, rpm_right)
        try:
            self.ser.write(output_str.encode('utf-8'))
            rospy.logdebug("发送数据：{}".format(output_str.strip()))
        except serial.SerialException as e:
            rospy.logerr("串口通信错误: {}".format(e))
            self.ser.close()

    def shutdown(self):
        # 节点关闭时执行的函数
        if self.ser.is_open:
            self.ser.close()
            rospy.loginfo("串口已关闭")

if __name__ == '__main__':
    try:
        controller = T265RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.shutdown()
