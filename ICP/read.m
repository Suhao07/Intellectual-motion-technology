clear;
clc;
% 读取PLY文件
laser_map = pcread('merged_point_cloud.ply');

% 可视化点云
pcshow(laser_map);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('merged_point_cloud.ply');
