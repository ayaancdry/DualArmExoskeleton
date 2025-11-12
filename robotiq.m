%% ABB YuMi Dual Arm - Forward Kinematics (Fully Compatible with R2024a)
clc; clear; close all;

disp('Loading ABB YuMi Robot...');
yumi = loadrobot('abbYumi', 'DataFormat', 'row', 'Gravity', [0 0 -9.81]);
show(yumi);
title('ABB YuMi Dual Arm Robot');
axis equal;
drawnow;

% Check DOF
nDOF = numel(homeConfiguration(yumi));
disp(['Total DOF detected: ' num2str(nDOF)]);

%% Create Joint Configuration Vector
qHome = homeConfiguration(yumi); % numeric array
q = qHome + (rand(1, nDOF) - 0.5) * (pi/6);  % random angles around home position

%% Visualize new configuration
show(yumi, q, 'PreservePlot', false, 'Frames', 'off');
title('ABB YuMi - Custom Joint Angles');
axis equal;
drawnow;

%% Compute Forward Kinematics
leftEE = 'yumi_link_7_l';
rightEE = 'yumi_link_7_r';

T_left = getTransform(yumi, q, leftEE);
T_right = getTransform(yumi, q, rightEE);

pos_left = tform2trvec(T_left);
pos_right = tform2trvec(T_right);

disp('------------------------------------------');
disp('Left Arm End Effector Position (m):');
disp(pos_left);
disp('Right Arm End Effector Position (m):');
disp(pos_right);

disp('Left Arm Orientation (Rotation Matrix):');
disp(T_left(1:3,1:3));
disp('Right Arm Orientation (Rotation Matrix):');
disp(T_right(1:3,1:3));

%% Plot end effector frames manually
hold on;

% Draw simple 3D coordinate frames for visualization
quiver3(pos_left(1), pos_left(2), pos_left(3), T_left(1,1)*0.1, T_left(2,1)*0.1, T_left(3,1)*0.1, 'r', 'LineWidth', 2); % X-axis
quiver3(pos_left(1), pos_left(2), pos_left(3), T_left(1,2)*0.1, T_left(2,2)*0.1, T_left(3,2)*0.1, 'g', 'LineWidth', 2); % Y-axis
quiver3(pos_left(1), pos_left(2), pos_left(3), T_left(1,3)*0.1, T_left(2,3)*0.1, T_left(3,3)*0.1, 'b', 'LineWidth', 2); % Z-axis

quiver3(pos_right(1), pos_right(2), pos_right(3), T_right(1,1)*0.1, T_right(2,1)*0.1, T_right(3,1)*0.1, 'r', 'LineWidth', 2);
quiver3(pos_right(1), pos_right(2), pos_right(3), T_right(1,2)*0.1, T_right(2,2)*0.1, T_right(3,2)*0.1, 'g', 'LineWidth', 2);
quiver3(pos_right(1), pos_right(2), pos_right(3), T_right(1,3)*0.1, T_right(2,3)*0.1, T_right(3,3)*0.1, 'b', 'LineWidth', 2);

text(pos_left(1), pos_left(2), pos_left(3)+0.05, 'Left EE', 'Color', 'r');
text(pos_right(1), pos_right(2), pos_right(3)+0.05, 'Right EE', 'Color', 'b');

title('ABB YuMi End Effector Frames');
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on; view(130, 25);