function [] = ekf_localization()
 
% Homework for ekf localization
% Modified by YH on 09/09/2019, thanks to the original open source
% Any questions please contact: zjuyinhuan@gmail.com

    close all;
    clear all;

    disp('EKF Start!')

    time = 0;
    global endTime; % [sec]
    endTime = 60;
    global dt;
    dt = 0.1; % [sec]

    removeStep = 5;

    nSteps = ceil((endTime - time)/dt);

    estimation.time=[];
    estimation.u=[];
    estimation.GPS=[];
    estimation.xOdom=[];
    estimation.xEkf=[];
    estimation.xTruth=[];

    % State Vector [x y yaw]'
    xEkf=[0 0 0]';
    PxEkf = eye(3);

    % Ground True State
    xTruth=xEkf;

    % Odometry Only
    xOdom=xTruth;

    % Observation vector [x y yaw]'
    z=[0 0 0]';

    % Simulation parameter
    global noiseQ
    noiseQ = diag([0.1 0 degreeToRadian(10)]).^2; %[Vx Vy yawrate]

    global noiseR
    noiseR = diag([0.1 0.1 degreeToRadian(5)]).^2;%[x y yaw]
    
    % Covariance Matrix for motion
    convQ=noiseQ;

    % Covariance Matrix for observation
    convR=noiseR;

    % Other Intial
    % ?
    


    % Main loop
    for i=1 : nSteps
        time = time + dt;
        % Input
        u=robotControl(time);
        % Observation
        [z,xTruth,xOdom,u]=prepare(xTruth, xOdom, u);

        % ------ Kalman Filter --------
        % Predict
        %?
        [xPredict, PPred] = ekf_predict(xEkf, PxEkf, u);
        % Update
        %?
        [xEkf, PxEkf] = ekf_update(xPredict, PPred, z);
        % -----------------------------

        % Simulation estimation
        estimation.time=[estimation.time; time];
        estimation.xTruth=[estimation.xTruth; xTruth'];
        estimation.xOdom=[estimation.xOdom; xOdom'];
        estimation.xEkf=[estimation.xEkf;xEkf'];
        estimation.GPS=[estimation.GPS; z'];
        estimation.u=[estimation.u; u'];

        % Plot in real time
        % Animation (remove some flames)
        if rem(i,removeStep)==0
            %hold off;
            plot(estimation.GPS(:,1),estimation.GPS(:,2),'*m', 'MarkerSize', 5);hold on;
            plot(estimation.xOdom(:,1),estimation.xOdom(:,2),'.k', 'MarkerSize', 10);hold on;
            plot(estimation.xEkf(:,1),estimation.xEkf(:,2),'.r','MarkerSize', 10);hold on;
            plot(estimation.xTruth(:,1),estimation.xTruth(:,2),'.b', 'MarkerSize', 10);hold on;
            axis equal;
            grid on;
            drawnow;
            %movcount=movcount+1;
            %mov(movcount) = getframe(gcf);
        end 
    end
    close
    
    finalPlot(estimation);
 
end

% control
function u = robotControl(time)
    global endTime;

    T = 10; % sec
    Vx = 1.0; % m/s
    Vy = 0.2; % m/s
    yawrate = 5; % deg/s
    
    % half
    if time > (endTime/2)
        yawrate = -5;
    end
    
    u =[ Vx*(1-exp(-time/T)) Vy*(1-exp(-time/T)) degreeToRadian(yawrate)*(1-exp(-time/T))]';
    
end

% all observations for 
function [z, xTruth, xOdom, u] = prepare(xTruth, xOdom, u)
    global noiseQ;
    global noiseR;

    % Ground Truth
    xTruth=doMotion(xTruth, u);
    % add Motion Noises
    u=u+noiseQ*randn(3,1);
    % Odometry Only
    xOdom=doMotion(xOdom, u);
    % add Observation Noises
    z=xTruth+noiseR*randn(3,1);
end


% Motion Model
function x = doMotion(x, u)
    global dt;
    % x: [x, y, yaw]'
    % u: [vx, vy, yawrate]'
    x(1) = x(1) + u(1)*cos(x(3))*dt;
    x(2) = x(2) + u(1)*sin(x(3))*dt;
    x(3) = x(3) + u(3)*dt;
end
% function x = doMotion(x, u)
%     global dt;
%     % x: [x, y, yaw]'
%     % u: [vx, vy, yawrate]'
%     
%     v = u(1);          % 速度
%     yawrate = u(3);    % 偏航角速率
%     yaw = x(3);        % 当前偏航角
%     
%     % 计算转向半径（避免除以零）
%     if abs(yawrate) > 0.001
%         radius = v / yawrate;
%     else
%         radius = 0;
%     end
%     
%     % 更新机器人的位置
%     x(1) = x(1) + radius * (sin(yaw + yawrate * dt) - sin(yaw));
%     x(2) = x(2) + radius * (-cos(yaw + yawrate * dt) + cos(yaw));
%     x(3) = x(3) + yawrate * dt;  % 更新偏航角
% end

% Jacobian of Motion Model
function jF = jacobF(x, u)
    global dt;
    % x: [x, y, yaw]'
    % u: [vx, vy, yawrate]'
    jF = [1, 0, -u(1)*sin(x(3))*dt;
          0, 1, u(1)*cos(x(3))*dt;
          0, 0, 1];
end

%Observation Model
function zPred = doObservation(xPred)
    % xPred: predicted state [x, y, yaw]'
    % Convert the predicted state to the form of the observation
    zPred = [xPred(1), xPred(2), xPred(3)]';
end
function zPred = doObservationNonlinear(xPred, sensorPos)
    % xPred: predicted state [x, y, yaw]'
    % sensorPos: sensor position [x_sensor, y_sensor]
    
    % 计算机器人位置与传感器位置之间的距离
    deltaX = xPred(1) - sensorPos(1);
    deltaY = xPred(2) - sensorPos(2);
    distance = sqrt(deltaX^2 + deltaY^2);
    
    % 将距离作为观测值
    zPred = [distance, xPred(3)]';  % 观测值为距离和偏航角
end

%Jacobian of Observation Model
function jH = jacobH(x)
    % x: [x, y, yaw]'
    jH = eye(3);
end

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



% finally plot the results
function []=finalPlot(estimation)
    figure;
    
    plot(estimation.GPS(:,1),estimation.GPS(:,2),'*m', 'MarkerSize', 5);hold on;
    plot(estimation.xOdom(:,1), estimation.xOdom(:,2),'.k','MarkerSize', 10); hold on;
    plot(estimation.xEkf(:,1), estimation.xEkf(:,2),'.r','MarkerSize', 10); hold on;
    plot(estimation.xTruth(:,1), estimation.xTruth(:,2),'.b','MarkerSize', 10); hold on;
    legend('GPS Observations','Odometry Only','EKF Localization', 'Ground Truth');

    xlabel('X (meter)', 'fontsize', 12);
    ylabel('Y (meter)', 'fontsize', 12);
    grid on;
    axis equal;
    
    % calculate error
    % ?
    odomError = sqrt(sum((estimation.xOdom - estimation.xTruth).^2, 2));
    ekfError = sqrt(sum((estimation.xEkf - estimation.xTruth).^2, 2));

    % display errors
    disp(['Odometry error: ', num2str(mean(odomError))]);
    disp(['EKF error: ', num2str(mean(ekfError))]);


end

function radian = degreeToRadian(degree)
    radian = degree/180*pi;
end