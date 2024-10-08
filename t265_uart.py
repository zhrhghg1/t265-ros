#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
import tf.transformations as tf_trans
import serial
import threading

# 串口配置
SERIAL_PORT = '/dev/ttyACM0'  # 根据实际情况修改
BAUD_RATE = 115200

# 全局变量
current_position = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
target_position = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

# 初始化串口
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def odom_callback(data):
    global current_position
    # 提取位置信息
    position = data.pose.pose.position
    x = position.x
    y = position.y

    # 提取四元数并转换为欧拉角
    orientation = data.pose.pose.orientation
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    euler = tf_trans.euler_from_quaternion(quaternion)

    yaw = euler[2]  # 使用Yaw角 (绕Z轴的旋转)

    # 更新当前位姿
    current_position['x'] = x
    current_position['y'] = y
    current_position['yaw'] = yaw

def send_data_to_esp32():
    """循环发送当前位姿和目标位姿到ESP32"""
    global current_position, target_position
    rate = rospy.Rate(10)  # 发送频率为10Hz
    while not rospy.is_shutdown():
        # 构造发送字符串，格式为 "curX,curY,curYaw;tarX,tarY,tarYaw,servo \n"
        message = "{:.2f},{:.2f},{:.2f};{:.2f},{:.2f},{:.2f}\n".format(
            current_position['x'], current_position['y'], current_position['yaw'],
            target_position['x'], target_position['y'], target_position['yaw']
        )
        ser.write(message.encode('utf-8'))
        rospy.loginfo("Sent to ESP32: {}".format(message.strip()))
        rate.sleep()

def input_target_position():
    """线程函数，允许用户输入目标位置"""
    global target_position
    while not rospy.is_shutdown():
        try:
            # 获取用户输入
            input_str = input("请输入目标位置 (格式: x y yaw servo)：")
            x_str, y_str, yaw_str= input_str.strip().split()
            target_position['x'] = float(x_str)
            target_position['y'] = float(y_str)
            target_position['yaw'] = float(yaw_str)
            #servo=int(servo_str)
            rospy.loginfo("目标位置已更新: X: {:.2f}, Y: {:.2f}, Yaw: {:.2f}, servo: {}".format(
                target_position['x'], target_position['y'], target_position['yaw']
            ))
        except ValueError:
            rospy.logerr("输入格式错误，请重新输入！格式: x y yaw")
        except Exception as e:
            rospy.logerr("发生错误: {}".format(e))

def main():
    global ser
    rospy.init_node('t265_odom_listener', anonymous=True)

    # 启动订阅T265里程计数据的回调函数
    rospy.Subscriber('/camera/odom/sample', Odometry, odom_callback)

    # 启动发送数据到ESP32的线程
    send_thread = threading.Thread(target=send_data_to_esp32)
    send_thread.start()

    # 启动用户输入目标位置的线程
    input_thread = threading.Thread(target=input_target_position)
    input_thread.start()

    rospy.spin()

    # 关闭串口
    ser.close()

if __name__ == '__main__':
    try:
        # 打开串口
        if not ser.is_open:
            ser.open()
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("发生错误: {}".format(e))
    finally:
        if ser.is_open:
            ser.close()

