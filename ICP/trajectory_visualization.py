import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 变换矩阵信息
transform_matrices = [
    np.array([[1., 0., 0., 0.],
              [0., 1., 0., 0.],
              [0., 0., 1., 0.],
              [0., 0., 0., 1.]]),
    np.array([[ 0.999475,  0.032402,  0.      ,  0.044232],
              [-0.032402,  0.999475,  0.      ,  1.085925],
              [ 0.      ,  0.      ,  1.      ,  0.      ],
              [ 0.      ,  0.      ,  0.      ,  1.      ]]),
    np.array([[ 0.994699,  0.102826,  0.      ,  0.19022 ],
              [-0.102826,  0.994699,  0.      ,  2.181384],
              [ 0.      ,  0.      ,  1.      ,  0.      ],
              [ 0.      ,  0.      ,  0.      ,  1.      ]]),
    np.array([[ 0.988222 , 0.153027,  0,      0.335301],
                [-0.153027,  0.988222,  0,       3.389808],
                [ 0,       0,        1,        0.      ],
                [ 0,        0,        0,      1.      ]]),
    np.array([[ 0.999648 , 0.02654,   0.  ,      0.409749],
            [-0.02654  , 0.999648 , 0.     ,   4.32209 ],
            [ 0.  ,      0.  ,      1.   ,     0.      ],
            [ 0.   ,     0.   ,     0.    ,    1.      ]]),
    np.array([[ 0.99818 , -0.060303 , 0.    ,    0.343737],
            [ 0.060303 , 0.99818 ,  0.   ,     5.192588],
            [ 0.  ,      0.    ,    1.    ,    0.      ],
            [ 0.  ,      0.    ,    0.     ,   1.      ]]),
    np.array([[ 0.999069, -0.043141 , 0.   ,     0.273041],
            [ 0.043141 , 0.999069 , 0.     ,   6.135863],
            [ 0.  ,      0.    ,    1.    ,    0.      ],
            [ 0.  ,      0.    ,    0.    ,    1.      ]]),
    np.array([[ 0.999718, -0.023767,  0.   ,     0.239555],
        [ 0.023767,  0.999718 , 0.    ,    7.053531],
        [ 0.     ,   0.    ,    1.    ,    0.      ],
        [ 0.     ,   0.   ,     0.   ,     1.      ]]),
    np.array([[ 0.997749,  0.067057 , 0.  ,      0.196093],
        [-0.067057 , 0.997749 , 0.    ,    8.137574],
        [ 0.     ,   0.   ,     1.    ,    0.      ],
        [ 0.     ,   0.   ,     0.    ,    1.      ]]),
    np.array([[ 0.999487 , 0.032017 , 0.  ,      0.259797],
            [-0.032017 , 0.999487 , 0.    ,    9.170733],
            [ 0.   ,     0.      ,  1.    ,    0.      ],
            [ 0.   ,     0.      ,  0.     ,   1.      ]]),
]


# 初始化位置和姿态信息
positions = [(0., 0., 0.)]
orientations = [np.eye(3)]

# 逐步计算位置和姿态信息
for i in range(1, len(transform_matrices)):
    prev_position = positions[-1]
    prev_orientation = orientations[-1]

    # 计算新的位置
    new_position = np.dot(transform_matrices[i], np.array([prev_position[0], prev_position[1], prev_position[2], 1]))[:3]

    # 计算新的姿态
    new_orientation = np.dot(prev_orientation, transform_matrices[i][:3, :3])

    positions.append(new_position)
    orientations.append(new_orientation)

# 可视化机器人运动轨迹
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 提取位置信息
x = [pos[0] for pos in positions]
y = [pos[1] for pos in positions]
z = [pos[2] for pos in positions]

# 绘制轨迹
ax.plot(x, y, z, marker='o')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
