import roboticstoolbox as rtb
import numpy as np
from math import pi
from myhand import Myhand

robot = Myhand()
#robot = rtb.models.DH.Panda()
#机械臂信息
print(robot)


# 定义欧拉角
angles = np.radians([30, 90, 20])  # 将角度转换为弧度

# 分别计算绕各轴旋转的旋转矩阵
R_z = np.array([[np.cos(angles[0]), -np.sin(angles[0]), 0],
                [np.sin(angles[0]), np.cos(angles[0]), 0],
                [0, 0, 1]])

R_y = np.array([[np.cos(angles[1]), 0, np.sin(angles[1])],
                [0, 1, 0],
                [-np.sin(angles[1]), 0, np.cos(angles[1])]])

R_x = np.array([[1, 0, 0],
                [0, np.cos(angles[2]), -np.sin(angles[2])],
                [0, np.sin(angles[2]), np.cos(angles[2])]])

# 计算总的旋转矩阵
R = np.dot(np.dot(R_z, R_y), R_x)
# 定义平移向量 t
t = np.array([0.3, 0.1, 0.2])

# 构建位姿齐次矩阵 T
T = np.eye(4)  # 创建单位矩阵
#T[:3, :3] = R  # 将旋转矩阵填充到左上角的3x3子矩阵
T[:3, 3] = t   # 将平移向量填充到右侧的一列

T = robot.fkine(robot.qr)
res = robot.ikine_LM(T)

print(res)


print(T)

qt = rtb.tools.trajectory.jtraj(robot.qz, res.q, 50)
#qt2 = rtb.tools.trajectory.jtraj(robot.qr, robot.qr2, 50)
robot.plot(qt.q, block=True)
#robot.plot(qt2.q, loop=True)



'''


import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH
import numpy as np

# 创建舵机参数
L1 = 1.0  # 舵机1的长度
L2 = 1.0  # 舵机2的长度
L3 = 1.0  # 舵机3的长度
L4 = 1.0  # 舵机4的长度

# 定义每个关节的DH参数
dh_params = [
    [0, 0, 0, np.pi / 2],    # 关节1
    [0, 0, L2, 0],           # 关节2
    [0, 0, L3, 0],           # 关节3
    [0, 0, L4, 0]            # 关节4
]

# 创建RevoluteDH对象，代表旋转关节
joints = [RevoluteDH(dh[0], dh[2], dh[1], dh[3]) for dh in dh_params]

# 创建机械臂模型
robot = DHRobot(joints)

robot.addconfiguration('qr', np.array([0, np.pi / 4, 0, 0]))

#qt = rtb.tools.trajectory.jtraj(robot.qr, robot.qr, 50)


robot.plot(robot.q, loop=True)

# 定义目标末端执行器位姿
T_target = np.array([[1, 0, 0, 1],  # 位置
                     [0, 1, 0, 1],  # 位置
                     [0, 0, 1, 1],  # 位置
                     [0, 0, 0, 1]])  # 旋转

# 使用逆运动学求解末端执行器的关节位置
q_initial_guess = np.zeros(4)  # 设置初始猜测值为0
q_solution = robot.ikine_LM(T_target, q0=q_initial_guess)

print("逆运动学求解结果：", q_solution)

'''
