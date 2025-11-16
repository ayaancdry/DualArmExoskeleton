% plotJointTrajectories.m — load jointData.mat and plot Figures 3 & 4
clear; clc;
dataFile = fullfile('report','jointData.mat');
if ~exist(dataFile,'file')
    error('jointData.mat not found in report/. Run animateDualArmGrasp first.');
end
load(dataFile, 't_vec', 'allQ', 'qd_traj');

n = size(allQ,2)/2;
% Joint angles 
fig1 = figure('Name','Joint Angles vs Time');
for i=1:n
    subplot(n,2,2*i-1);
    plot(t_vec, allQ(:,i),'b-'); grid on;
    title(sprintf('Left Joint %d',i));
    xlabel('Time (s)'); ylabel('Angle (rad)');
    subplot(n,2,2*i);
    plot(t_vec, allQ(:,i+n),'r-'); grid on;
    title(sprintf('Right Joint %d',i));
    xlabel('Time (s)'); ylabel('Angle (rad)');
end
sgtitle('Joint Angles vs. Time');
saveas(fig1, fullfile('report','Figure3_Joint_Angles_manual.png'));

% Joint velocities
fig2 = figure('Name','Joint Velocities vs Time');
for i=1:n
    subplot(n,2,2*i-1);
    plot(t_vec, qd_traj(:,i),'b-'); grid on;
    title(sprintf('Left Joint %d Vel',i));
    xlabel('Time (s)'); ylabel('Vel (rad/s)');
    subplot(n,2,2*i);
    plot(t_vec, qd_traj(:,i+n),'r-'); grid on;
    title(sprintf('Right Joint %d Vel',i));
    xlabel('Time (s)'); ylabel('Vel (rad/s)');
end
sgtitle('Joint Velocities vs. Time');
saveas(fig2, fullfile('report','Figure4_Joint_Velocities_manual.png'));

fprintf('Saved manual joint plots to report/ as PNGs\n');
