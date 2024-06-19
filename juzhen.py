import numpy as np

# 定义欧拉角
angles = np.radians([30, 45, 90])  # 将角度转换为弧度

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
t = np.array([0.2, 0.2, 0.1])

# 构建位姿齐次矩阵 T
T = np.eye(4)  # 创建单位矩阵
T[:3, :3] = R  # 将旋转矩阵填充到左上角的3x3子矩阵
T[:3, 3] = t   # 将平移向量填充到右侧的一列

print("旋转矩阵为：")
print(R)

print("齐次矩阵：")
print(T)
