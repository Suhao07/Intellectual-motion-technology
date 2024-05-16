## 智能移动技术EKF定位作业

### 一、作业要求与程序说明

在matlab中，利用ekf_localization.m文件，实现移动机器人的EKF定位算法

-  本任务中的观测模型输入为 GPS 全局绝对观测，运动模型的输入为固定的速度和角速度。运动模型的输入在 robotControl()函数中已定义，u包含 Vx,Vy 和角速度 w，不需要修改。
-  机器人的状态量为三维: xTruth, xOdom, xEkf=(x,y,yaw)。
- 在给定的prepare()函数中，本程序首先根据机器人速度和运动模型计算定位 真值。然后在速度上添加噪声，计算有误差的里程计定位。然后在定位真值上添加噪声，形成 GPS 观测。噪声在文件开始已给定，该函数不需要修改。
- Amination部分和finalPlot()函数亦包含绘图部分，不需要修改。
-  如果需要对变量或者函数功能调整，可自行修改或者优化。

### 二、具体实现

1. #### 协方差矩阵convR和convQ，以及其他EKF初始化。

```matlab
 % Simulation parameter
    global noiseQ
    noiseQ = diag([0.1 0 degreeToRadian(10)]).^2; %[Vx Vy yawrate]

    global noiseR
    noiseR = diag([0.5 0.5 degreeToRadian(5)]).^2;%[x y yaw]
    
    % Covariance Matrix for motion
    convQ=noiseQ;

    % Covariance Matrix for observation
    convR=noiseR;
```

使用两个高斯分布噪声模型分别给运动模型和观测模型的协方差矩阵赋初值。

#### 2.运动模型doMotion()，以及雅各比矩阵jacobF()。输入为上一时刻位姿和速度角速度，以及时间间隔，输出为推算得到的当前位姿。

首先实现了一个简单的二维运动模型

给定当前状态向量$  x = [x, y, \theta]^T $ 和控制输入向量$  u = [v_x, v_y, \dot{\theta}]^T $，其中 $v_x $ 和 $ v_y $ 分别是机器人在x和y方向上的线速度，$ \dot{\theta} $是机器人的偏航角速率。

更新机器人的位置和姿态的过程可以用下面的数学公式表示：

$
\begin{align*}
x' &= x + v_x \cos(\theta) \cdot dt \\
y' &= y + v_x \sin(\theta) \cdot dt \\
\theta' &= \theta + \dot{\theta} \cdot dt
\end{align*}
$

其中 $ (x', y')  $是更新后的机器人位置，$\theta' $ 是更新后的偏航角，$dt$ 是时间步长。

```matlab
% Motion Model
function x = doMotion(x, u)
global dt;
%     % x: [x, y, yaw]'
%    % u: [vx, vy, yawrate]'
     x(1) = x(1) + u(1)*cos(x(3))*dt;
     x(2) = x(2) + u(1)*sin(x(3))*dt;
     x(3) = x(3) + u(3)*dt;
 end
```

对运动模型进行更精准的预测，假设运动模型为非线性模型

1. 首先，计算转向半径 $ \text{radius} $。当$  \dot{\theta}  $的绝对值大于一个很小的阈值时（这里设为0.001），转向半径为 $ \text{radius} = \frac{v}{\dot{\theta}} $，否则$  \text{radius} = 0 $。

2. 使用转向半径$ \text{radius} $，根据当前偏航角$  \theta  $和偏航角速率 $ \dot{\theta} $，以及时间步长 $ dt $，计算机器人在x和y方向上的位移：

$
\begin{align*}
\Delta x &= \text{radius} \cdot (\sin(\theta + \dot{\theta} \cdot dt) - \sin(\theta)) \\
\Delta y &= \text{radius} \cdot (-\cos(\theta + \dot{\theta} \cdot dt) + \cos(\theta))
\end{align*}
$

3. 更新机器人的位置和偏航角：

$
\begin{align*}
x' &= x + \Delta x \\
y' &= y + \Delta y \\
\theta' &= \theta + \dot{\theta} \cdot dt
\end{align*}
$

```matlab
function x = doMotion(x, u)
    global dt;
    % x: [x, y, yaw]'
    % u: [vx, vy, yawrate]'
    
    v = u(1);          % 速度
    yawrate = u(3);    % 偏航角速率
    yaw = x(3);        % 当前偏航角
    
    % 计算转向半径（避免除以零）
    if abs(yawrate) > 0.001
        radius = v / yawrate;
    else
        radius = 0;
    end
    
    % 更新机器人的位置
    x(1) = x(1) + radius * (sin(yaw + yawrate * dt) - sin(yaw));
    x(2) = x(2) + radius * (-cos(yaw + yawrate * dt) + cos(yaw));
    x(3) = x(3) + yawrate * dt;  % 更新偏航角
end
```

雅可比矩阵的表达式如下：

$
J_F = \begin{bmatrix}
\frac{\partial x}{\partial x} & \frac{\partial x}{\partial y} & \frac{\partial x}{\partial \theta} \\
\frac{\partial y}{\partial x} & \frac{\partial y}{\partial y} & \frac{\partial y}{\partial \theta} \\
\frac{\partial \theta}{\partial x} & \frac{\partial \theta}{\partial y} & \frac{\partial \theta}{\partial \theta}
\end{bmatrix}
$

根据机器人的运动模型，可以计算出每个元素的偏导数：

$
\begin{align*}
\frac{\partial x}{\partial x} &= 1, & \frac{\partial x}{\partial y} &= 0, & \frac{\partial x}{\partial \theta} &= -v \sin(\theta) \cdot dt \\
\frac{\partial y}{\partial x} &= 0, & \frac{\partial y}{\partial y} &= 1, & \frac{\partial y}{\partial \theta} &= v \cos(\theta) \cdot dt \\
\frac{\partial \theta}{\partial x} &= 0, & \frac{\partial \theta}{\partial y} &= 0, & \frac{\partial \theta}{\partial \theta} &= 1
\end{align*}
$

因此，雅可比矩阵 \( J_F \) 可以表示为：

$
J_F = \begin{bmatrix}
1 & 0 & -v_x \sin(\theta) \cdot dt \\
0 & 1 & v_x \cos(\theta) \cdot dt \\
0 & 0 & 1
\end{bmatrix}
$

```matlab
% Jacobian of Motion Model
function jF = jacobF(x, u)
    global dt;
    % x: [x, y, yaw]'
    % u: [vx, vy, yawrate]'
    jF = [1, 0, -u(1)*sin(x(3))*dt;
          0, 1, u(1)*cos(x(3))*dt;
          0, 0, 1];
end
```

#### 3.观测模型doObservation()，以及雅各比矩阵jacobH()。输入为GPS观测和运动模型的预测位姿。

观测模型将预测的状态向量$  x_{\text{pred}} $ 转换为观测向量 $ z_{\text{pred}} $，其形式与状态向量相同，即：

$
z_{\text{pred}} = [x_{\text{pred}}, y_{\text{pred}}, \theta_{\text{pred}}]^T
$

直接将预测的状态向量作为观测向量输出。

```matlab
%Observation Model
function zPred = doObservation(xPred)
    % xPred: predicted state [x, y, yaw]'
    % Convert the predicted state to the form of the observation
    zPred = [xPred(1), xPred(2), xPred(3)]';
end
```

观测模型雅克比为单位矩阵。

#### 4.在for循环中，实现拓展卡尔曼滤波的过程，输出Ekf位姿变量x

1. **预测步骤（ekf_predict）**：
   - 根据状态转移方程对当前状态进行预测，得到预测的状态向量$ X_{pred}$。
   - 利用状态转移方程的雅可比矩阵计算状态协方差矩阵的预测值 $𝑃_{pred}$。
   - 考虑系统的过程噪声，更新状态协方差矩阵的预测值。

```matlab
function [xPred, PPred] = ekf_predict(x, P, u)
    % x: state vector
    % P: state covariance matrix
    % u: control inputs
    global dt;
    global noiseQ;

    % Predict state
    xPred = doMotion(x, u);

    % Predict state covariance
    F = jacobF(x, u);
    PPred = F * P * F' + noiseQ;
end
```

1. **更新步骤（ekf_update）**：
   - 利用观测模型将预测的状态向量转换为观测空间中的预测值 $Z_{pred}$。
   - 计算预测残差（观测值与预测值之差），即创新向量。
   - 利用观测模型的雅可比矩阵，计算卡尔曼增益。
   - 利用卡尔曼增益将预测值与观测值进行融合，得到更新后的状态向量。
   - 更新状态协方差矩阵，考虑观测噪声的影响。

```matlab
function [xEkf, PxEkf] = ekf_update(xPred, PPred, z)
    % xPred: predicted state vector
    % PPred: predicted state covariance matrix
    % z: measurements
    global noiseR;

    % Measurement update
    H = jacobH(xPred);
    K = PPred * H' / (H * PPred * H' + noiseR); % Kalman gain

    % Update state
    % Update
    zPred = doObservation(xPred);
    innovation = z - zPred;  % observation residual
    xEkf = xPred + K * innovation;


    % Update state covariance
    PxEkf = (eye(size(xEkf,1)) - K * H) * PPred;
end
```

#### 5.在最后的 finalPlot()函数中，根据定位真值，计算纯里程计误差，以及你的EKF定位误差，用disp()函数在控制台输出

计算估计值与真实状态的欧氏距离，求所有时刻的误差平均值，作为误差表达。

```matlab
 % calculate error
    % ?
    odomError = sqrt(sum((estimation.xOdom - estimation.xTruth).^2, 2));
    ekfError = sqrt(sum((estimation.xEkf - estimation.xTruth).^2, 2));

    % display errors
    disp(['Odometry error: ', num2str(mean(odomError))]);
    disp(['EKF error: ', num2str(mean(ekfError))]);
```

### 三、结果展示与分析

#### 全局显示结果

简单二维运动模型

 ![image-20240428181134041](C:\Users\admin\AppData\Roaming\Typora\typora-user-images\image-20240428181134041.png)

Odometry error: 0.94333
EKF error: 0.066181

非线性运动模型

![image-20240428181603899](C:\Users\admin\AppData\Roaming\Typora\typora-user-images\image-20240428181603899.png)

Odometry error: 0.54888
EKF error: 0.068307

#### 局部放大结果

简单二维运动模型

![image-20240428181815304](C:\Users\admin\AppData\Roaming\Typora\typora-user-images\image-20240428181815304.png)

非线性运动模型

![image-20240428181707034](C:\Users\admin\AppData\Roaming\Typora\typora-user-images\image-20240428181707034.png)

考虑误差来源可能有：

**过程噪声的影响：** EKF 在预测状态时考虑了过程噪声，即系统动态的不确定性。过程噪声的大小会影响状态的预测精度，通常由噪声协方差矩阵来表示。

**初始状态和初始协方差的准确性：** EKF 需要初始状态和初始状态协方差来开始估计。如果初始状态或初始状态协方差存在误差，那么预测的状态也会有误差，并且这种误差可能会在滤波过程中累积。

**运动模型的准确性：** EKF 中的预测步骤使用了运动模型来预测下一个时刻的状态。如果运动模型与实际系统的运动行为不匹配或者存在误差，那么预测的状态也会有误差。

