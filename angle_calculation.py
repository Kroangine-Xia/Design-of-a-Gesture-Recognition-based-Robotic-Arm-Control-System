import cv2
import mediapipe as mp
import numpy as np
import math

# 初始化 MediaPipe 手部跟踪器
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

# 获取手部关键点索引
HAND_CONNECTIONS = mp_hands.HAND_CONNECTIONS

# 定义手指关键点的索引
#FINGER_MCP_INDEX = [2, 5, 9, 13, 17]  # 每根手指的第一个关节索引
FINGER_MCP_INDEX = [2, 5, 9, 13, 17]  # 每根手指的第一个关节索引

# 计算两点之间的距离
def calculate_distance(point1, point2):
    return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2)

# 计算手指的弯曲角度
def calculate_finger_angle(finger_mcp, finger_pip, finger_dip):
    a = calculate_distance(finger_mcp, finger_pip)
    b = calculate_distance(finger_pip, finger_dip)
    c = calculate_distance(finger_mcp, finger_dip)

    angle = math.degrees(math.acos((a ** 2 + b ** 2 - c ** 2) / (2 * a * b)))
    return angle

# 处理图像并计算手指弯曲角度的函数
def process_frame(frame):
    # 调整图像以匹配 MediaPipe 的输入要求
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # 运行手部追踪器
    results = hands.process(rgb_frame)

    # 获取手部关键点
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # 计算每根手指的弯曲角度
            finger_angles = []
            for i in range(len(FINGER_MCP_INDEX)):
                mcp_index = FINGER_MCP_INDEX[i]
                pip_index = mcp_index + 1
                dip_index = mcp_index + 2

                finger_mcp = hand_landmarks.landmark[mcp_index]
                finger_pip = hand_landmarks.landmark[pip_index]
                finger_dip = hand_landmarks.landmark[dip_index]

                angle = calculate_finger_angle(finger_mcp, finger_pip, finger_dip)
                finger_angles.append(angle)

            # 输出手指的第一个指关节的角度
            print("Finger MCP Angles:", finger_angles)

# 打开摄像头并进行实时处理
cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 处理图像并计算手指弯曲角度
    process_frame(frame)

    # 在主线程中显示原始图像
    cv2.imshow('Hand Tracking', frame)

    # 按下 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
