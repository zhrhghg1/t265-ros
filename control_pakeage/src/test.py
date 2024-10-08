import serial
import random
import time

# 串口配置
COM_PORT = '/dev/ttyUSB0'  # 你的串口端口
BAUD_RATE = 115200  # 波特率

def generate_random_speed():
    """生成随机的左右电机速度值，范围在 -330 到 330 之间"""
    left_speed =10
    right_speed = 10
    return left_speed, right_speed

def send_motor_speeds(ser, left_speed, right_speed):
    """以 `xx,xx` 格式发送电机速度到串口"""
    data = f"{left_speed},{right_speed}\n"
    ser.write(data.encode())  # 发送数据
    print(f"Sent: {data.strip()}")  # 打印发送数据

def main():
    # 连接串口
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {COM_PORT} at {BAUD_RATE} baud.")
    except Exception as e:
        print(f"Failed to connect to {COM_PORT}: {e}")
        return

    # 每隔2秒发送随机速度
    try:
        while True:
            left_speed, right_speed = generate_random_speed()  # 生成随机速度
            send_motor_speeds(ser, left_speed, right_speed)  # 发送到串口
            time.sleep(0.3)  # 等待2秒
    except KeyboardInterrupt:
        print("Test stopped by user.")
    finally:
        ser.close()  # 关闭串口
        print("Serial connection closed.")

if __name__ == "__main__":
    main()

