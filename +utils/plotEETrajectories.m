function plotEETrajectories(path_left, path_right, object_center)
% plotEETrajectories Plots the 3D paths of the end-effectors
%   utils.plotEETrajectories(path_left, path_right, object_center)
%
%   Inputs:
%       path_left:     [Nx3] matrix of left EE (x,y,z) positions
%       path_right:    [Nx3] matrix of right EE (x,y,z) positions
%       object_center: [1x3] position of the virtual object

fig = figure('Name', 'End-Effector Trajectories');
ax = axes('Parent', fig);
hold(ax, 'on');

% Plot paths
plot3(ax, path_left(:,1), path_left(:,2), path_left(:,3), 'b-', 'LineWidth', 2, 'DisplayName', 'Left EE Path');
plot3(ax, path_right(:,1), path_right(:,2), path_right(:,3), 'r-', 'LineWidth', 2, 'DisplayName', 'Right EE Path');

% Plot start points
plot3(ax, path_left(1,1), path_left(1,2), path_left(1,3), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Start');
plot3(ax, path_right(1,1), path_right(1,2), path_right(1,3), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Start');

% Plot end points (grasp)
plot3(ax, path_left(end,1), path_left(end,2), path_left(end,3), 'bx', 'MarkerSize', 10, 'LineWidth', 3, 'DisplayName', 'Grasp');
plot3(ax, path_right(end,1), path_right(end,2), path_right(end,3), 'rx', 'MarkerSize', 10, 'LineWidth', 3, 'DisplayName', 'Grasp');

% Plot object
plot3(ax, object_center(1), object_center(2), object_center(3), 'ks', 'MarkerSize', 20, 'LineWidth', 3, 'DisplayName', 'Virtual Object');

% Formatting
title('Figure 2: End-Effector Trajectories for Bi-Manual Grasp');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis equal;
grid on;
legend show;
view(60, 15);

% Save the figure
saveas(fig, 'report/Figure2_EE_Trajectories.png');

end