from typing import Any

import cv2
import numpy as np
import mediapipe as mp
import serial
import math
from filterpy.kalman import KalmanFilter

#创建两个KalmanFilter实例，kf和kf2，用于对舵机角度数据进行滤波，以减少噪声和提高控制精度。
# 创建卡尔曼滤波器，指定状态向量维度为4，观测向量维度也为4
kf = KalmanFilter(dim_x=5, dim_z=5)

#设置卡尔曼滤波器参数
# 设置状态转移矩阵，这里我们假设角度信息在预测过程中不发生变化
kf.F = np.array([[1, 0, 0, 0, 0],   # 状态转移矩阵描述了当前状态与下一时刻状态之间的关系
                 [0, 1, 0, 0, 0],   # 在这个示例中，我们假设角度信息在预测过程中不发生变化
                 [0, 0, 1, 0, 0],   # 因此，状态转移矩阵中角度信息的变化率为0
                 [0, 0, 0, 1, 0],
                 [0, 0, 0, 0, 1]])

# 设置测量矩阵，这里假设我们可以直接观测到角度信息
kf.H = np.array([[1, 0, 0, 0, 0],   # 测量矩阵描述了状态变量与观测变量之间的关系
                 [0, 1, 0, 0, 0],   # 在这个示例中，我们可以直接观测到角度信息
                 [0, 0, 1, 0, 0],   # 因此，测量矩阵中角度信息直接映射到状态向量
                 [0, 0, 0, 1, 0],
                 [0, 0, 0, 0, 1]])

# 设置过程噪声协方差矩阵，表示系统在预测过程中的不确定性
kf.Q *= 0.01  # 这里简单地假设过程噪声是恒定的，并且是相互独立的

# 设置测量噪声协方差矩阵，表示观测值的不确定性
kf.R *= 0.1   # 这里简单地假设测量噪声是恒定的，并且是相互独立的

# 设置初始状态向量和初始协方差矩阵
kf.x = np.array([0, 0, 0, 0, 0])  # 初始状态向量，这里假设初始角度信息都是0度
kf.P *= 10  # 初始协方差矩阵，表示初始状态的不确定性

FINGER_MCP_INDEX = [5, 9, 13, 17, 1]  # 每根手指的第一个关节索引
FINGER_PIP_INDEX = [6, 10, 14, 18, 2]  # 每根手指的第二个关节索引
FINGER_DIP_INDEX = [7, 11, 15, 19, 4]   # 每根手指的第三个关节索引


# 创建卡尔曼滤波器，指定状态向量维度为4，观测向量维度也为4
kf2 = KalmanFilter(dim_x=5, dim_z=5)

# 设置状态转移矩阵，这里我们假设角度信息在预测过程中不发生变化
kf2.F = np.array([[1, 0, 0, 0, 0],   # 状态转移矩阵描述了当前状态与下一时刻状态之间的关系
                 [0, 1, 0, 0, 0],   # 在这个示例中，我们假设角度信息在预测过程中不发生变化
                 [0, 0, 1, 0, 0],   # 因此，状态转移矩阵中角度信息的变化率为0
                 [0, 0, 0, 1, 0],
                 [0, 0, 0, 0, 1]])

# 设置测量矩阵，这里假设我们可以直接观测到角度信息
kf2.H = np.array([[1, 0, 0, 0, 0],   # 测量矩阵描述了状态变量与观测变量之间的关系
                 [0, 1, 0, 0, 0],   # 在这个示例中，我们可以直接观测到角度信息
                 [0, 0, 1, 0, 0],   # 因此，测量矩阵中角度信息直接映射到状态向量
                 [0, 0, 0, 1, 0],
                 [0, 0, 0, 0, 1]])

# 设置过程噪声协方差矩阵，表示系统在预测过程中的不确定性
kf2.Q *= 0.01  # 这里简单地假设过程噪声是恒定的，并且是相互独立的=

# 设置测量噪声协方差矩阵，表示观测值的不确定性
kf2.R *= 0.1   # 这里简单地假设测量噪声是恒定的，并且是相互独立的

# 设置初始状态向量和初始协方差矩阵
kf2.x = np.array([0, 0, 0, 0, 0])  # 初始状态向量，这里假设初始角度信息都是0度
kf2.P *= 10  # 初始协方差矩阵，表示初始状态的不确定性


# 计算两点之间的三维距离
def calculate_distance(point1, point2):
    return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2)

# 计算手指的弯曲角度
def calculate_finger_angle(finger_mcp, finger_pip, finger_dip):
    a = calculate_distance(finger_mcp, finger_pip)
    b = calculate_distance(finger_pip, finger_dip)
    c = calculate_distance(finger_mcp, finger_dip)

    angle = math.degrees(math.acos((a ** 2 + b ** 2 - c ** 2) / (2 * a * b)))
    return angle


def calculate_distance2dxy(point1, point2):
    return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

# 计算手指的弯曲角度
def calculate_pose_angle2dxy(finger_mcp, finger_pip, finger_dip):
    a = calculate_distance2dxy(finger_mcp, finger_pip)
    b = calculate_distance2dxy(finger_pip, finger_dip)
    c = calculate_distance2dxy(finger_mcp, finger_dip)

    angle = math.degrees(math.acos((a ** 2 + b ** 2 - c ** 2) / (2 * a * b)))
    return angle


def calculate_distance2dxz(point1, point2):
    return math.sqrt((point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2)

# 计算手指的弯曲角度
def calculate_pose_angle2dxz(finger_mcp, finger_pip, finger_dip):
    a = calculate_distance2dxz(finger_mcp, finger_pip)
    b = calculate_distance2dxz(finger_pip, finger_dip)
    c = calculate_distance2dxz(finger_mcp, finger_dip)

    angle = math.degrees(math.acos((a ** 2 + b ** 2 - c ** 2) / (2 * a * b)))
    return angle

def hand_control(ser, a=900, b=900, c=900, d=900, e=900):
    hex_string = hex(a)[2:].zfill(4)  # [2:] 去掉前缀"0x"，zfill(4) 用于补齐到4位
    # 提取高八位和低八位
    ah = int(hex_string[:2], 16)  # 将前两位十六进制转换为整数
    al = int(hex_string[2:], 16)  # 将后两位十六进制转换为整数
    ser.write([0xff, 0x02, 0x00, al, ah])
    hex_string = hex(b)[2:].zfill(4)  # [2:] 去掉前缀"0x"，zfill(4) 用于补齐到4位
    # 提取高八位和低八位
    bh = int(hex_string[:2], 16)  # 将前两位十六进制转换为整数
    bl = int(hex_string[2:], 16)  # 将后两位十六进制转换为整数
    #ser.write([0xff, 0x01, 0x01, 0x28, 0x00])
    ser.write([0xff, 0x02, 0x01, bl, bh])
    hex_string = hex(c)[2:].zfill(4)  # [2:] 去掉前缀"0x"，zfill(4) 用于补齐到4位
    # 提取高八位和低八位
    ch = int(hex_string[:2], 16)  # 将前两位十六进制转换为整数
    cl = int(hex_string[2:], 16)  # 将后两位十六进制转换为整数
    ser.write([0xff, 0x02, 0x02, cl, ch])
    hex_string = hex(d)[2:].zfill(4)  # [2:] 去掉前缀"0x"，zfill(4) 用于补齐到4位
    # 提取高八位和低八位
    dh = int(hex_string[:2], 16)  # 将前两位十六进制转换为整数
    dl = int(hex_string[2:], 16)  # 将后两位十六进制转换为整数
    ser.write([0xff, 0x02, 0x03, dl, dh])
    hex_string = hex(e)[2:].zfill(4)  # [2:] 去掉前缀"0x"，zfill(4) 用于补齐到4位
    # 提取高八位和低八位
    eh = int(hex_string[:2], 16)  # 将前两位十六进制转换为整数
    el = int(hex_string[2:], 16)  # 将后两位十六进制转换为整数
    ser.write([0xff, 0x02, 0x04, el, eh])


mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_holistic: Any = mp.solutions.holistic

joint_list = [[7, 6, 5], [11, 10, 9], [15, 14, 13], [19, 18, 17], [3, 2, 1]]  # 手指关节序列
#joint_list = [[6, 5, 0], [10, 9, 0], [14, 13, 0], [18, 17, 0], [3, 2, 1]]  # 手指关节序列

ser = serial.Serial('COM25', 9600, timeout=1)
#serdis = serial.Serial('COM22', 9600, timeout=1)  # 根据你的串口号和波特率进行更改

cap = cv2.VideoCapture(0)
#使用mediapipe.Holistic进行手势跟踪，获取手势的关键点。
with mp_holistic.Holistic(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        model_complexity=1) as holistic:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            break
        image.flags.writeable = False  # 将图像设为不可写，为了在处理过程中不修改原始图像数据
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # 转换图像颜色空间为 RGB
        results = holistic.process(image) # 使用 MediaPipe Holistic 模型检测图像中的人体姿态和手部关键点
        image.flags.writeable = True# 将图像设为可写，以便在图像上绘制关键点
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) # 将图像颜色空间转换回 BGR，以便在 OpenCV 中显示图像

        # 渲染
        '''mp_drawing.draw_landmarks(
            image,
            results.face_landmarks,
            mp_holistic.FACEMESH_CONTOURS,
            landmark_drawing_spec=None,
            connection_drawing_spec=mp_drawing_styles
                .get_default_face_mesh_tesselation_style())'''
        #使用mp_drawing.draw_landmarks在图像上绘制手势关键点。
        mp_drawing.draw_landmarks(
            image,
            results.pose_landmarks,
            mp_holistic.POSE_CONNECTIONS, landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

        mp_drawing.draw_landmarks(image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                                  landmark_drawing_spec=mp_drawing_styles.get_default_hand_landmarks_style())
        mp_drawing.draw_landmarks(image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                                  landmark_drawing_spec=mp_drawing_styles.get_default_hand_landmarks_style())
        # 监测到姿势，执行
        if results.pose_landmarks:
            RHL = results.pose_landmarks

            # 计算角度
            pose_angle = [0, 0, 0, 0, 0]
            mcp = RHL.landmark[11]
            pip = RHL.landmark[12]
            dip = RHL.landmark[14]

            angle = calculate_pose_angle2dxy(mcp, pip, dip)
            anglez = calculate_pose_angle2dxz(mcp, pip, dip)
            anglez = anglez if anglez >= 90 else 90.0

            angle_leg = (calculate_pose_angle2dxy(RHL.landmark[12], RHL.landmark[14], RHL.landmark[16]))

            pose_angle[0] = angle
            pose_angle[1] = anglez
            pose_angle[2] = angle_leg

            cv2.putText(image, str(round(angle, 2)), tuple(np.multiply(np.array([RHL.landmark[14].x, RHL.landmark[14].y]), [640, 480]).astype(int)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            kf2.predict()  # 预测下一时刻状态
            kf2.update(pose_angle)  # 更新当前时刻状态，利用观测值进行校正
            pose_angle = kf2.x

            a = int(2000 - (pose_angle[0] - 90) / 90 * 1500)
            hex_string = hex(a)[2:].zfill(4)  # [2:] 去掉前缀"0x"，zfill(4) 用于补齐到4位
            # 提取高八位和低八位
            ah = int(hex_string[:2], 16)  # 将前两位十六进制转换为整数
            al = int(hex_string[2:], 16)  # 将后两位十六进制转换为整数
            ser.write([0xff, 0x02, 0x0a, al, ah])

            a = int(800 + (pose_angle[1] - 90) / 70 * 1300)
            hex_string = hex(a)[2:].zfill(4)  # [2:] 去掉前缀"0x"，zfill(4) 用于补齐到4位
            # 提取高八位和低八位
            ah = int(hex_string[:2], 16)  # 将前两位十六进制转换为整数
            al = int(hex_string[2:], 16)  # 将后两位十六进制转换为整数
            ser.write([0xff, 0x02, 0x07, al, ah])

            a = int(900 + (pose_angle[2]) / 180 * 1400)
            #print(a)
            hex_string = hex(a)[2:].zfill(4)  # [2:] 去掉前缀"0x"，zfill(4) 用于补齐到4位
            # 提取高八位和低八位
            ah = int(hex_string[:2], 16)  # 将前两位十六进制转换为整数
            al = int(hex_string[2:], 16)  # 将后两位十六进制转换为整数
            ser.write([0xff, 0x02, 0x09, al, ah])

            #print(a, anglez)

        if results.right_hand_landmarks:
            RHL = results.right_hand_landmarks
            #global fig17
            #global fig15
            fig15 = RHL.landmark[15]
            fig17 = RHL.landmark[17]
            print(1000*(fig15.z - fig17.z))

            # 计算角度
            hand_angle = []
            for i in range(len(FINGER_MCP_INDEX)):
                #print(RHL.landmark)

                mcp_index = FINGER_MCP_INDEX[i]
                pip_index = FINGER_PIP_INDEX[i]
                dip_index = FINGER_DIP_INDEX[i]

                finger_mcp = RHL.landmark[mcp_index]
                finger_pip = RHL.landmark[pip_index]
                finger_dip = RHL.landmark[dip_index]

                angle = calculate_finger_angle(finger_mcp, finger_pip, finger_dip)

                '''a = np.array([RHL.landmark[joint[0]].x, RHL.landmark[joint[0]].y, RHL.landmark[joint[0]].z])
                b = np.array([RHL.landmark[joint[1]].x, RHL.landmark[joint[1]].y, RHL.landmark[joint[1]].z])
                c = np.array([RHL.landmark[joint[2]].x, RHL.landmark[joint[2]].y, RHL.landmark[joint[2]].z])
                alpha = b - a
                beta = c - b
                # 计算弧度
                chacheng = np.cross(alpha, beta) #叉乘
                #print(chacheng)
                diancheng = np.dot(alpha, beta) #点乘
                #print(diancheng)
                chachengmo = np.linalg.norm(chacheng) #叉乘的模
                radians_fingers = np.arctan(chachengmo / diancheng)
                angle = math.degrees(radians_fingers) + 90 #np.abs(radians_fingers * 180.0 / np.pi)  # 弧度转角度
                #print(angle)

                if angle > 180.0:
                    angle = 360 - angle'''

                hand_angle.append(angle)
                #cv2.putText(image, str(round(angle, 2)), tuple(np.multiply(b, [640, 480]).astype(int)),
                #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

            #hand_control(ser, 2000, int(hand_angle[0]/180*1100 + 900), int(hand_angle[1]/180*1100 + 900), int(hand_angle[2]/180*1100 + 900), int(hand_angle[3]/180*1100 + 900))
                # 进行卡尔曼滤波
            #print(hand_angle[4])
            kf.predict()  # 预测下一时刻状态
            kf.update(hand_angle)  # 更新当前时刻状态，利用观测值进行校正
            #serdis.write('f1={:.2f}, f2={:.2f}\r\n'.format(hand_angle[0], kf.x[0]).encode('utf-8'))
            hand_angle = kf.x
            hand_control(ser,
                         int(2500 - (hand_angle[4] - 100) / 80 * 1600), #大拇指转动角度
                         int(900 + (hand_angle[0] - 70) / 100 * 1100),  #食指
                         int(900 + (hand_angle[1] - 80) / 100 * 1100), #中指
                         int(900 + (hand_angle[2] - 80) / 100 * 1100),  #无名指
                         int(900 + (hand_angle[3] - 85) / 100 * 1100)) #小指
            hand_control2(ser,
                         int(2000 - (pose_angle[0] - 90) / 90 * 1500),
                         int(800 + (pose_angle[1] - 90) / 70 * 1300),
                         int(900 + (pose_angle[2]) / 180 * 1400))
            #print(hand_angle)
            #ser.write([0xff, 0x02, 0x07, 0xa4, 0x06])
        # cv2.imshow('MediaPipe Holistic', cv2.flip(image, 1))
        cv2.imshow('Mediapipe Holistic', image)  # 取消镜面翻转
        if cv2.waitKey(5) == ord('q'):
            break
cap.release()
