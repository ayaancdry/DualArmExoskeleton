function plotWorkspace()
% simulation.plotWorkspace  Random sampling workspace (YuMi)

robot  = kinematics.createDualArmExo();
nSamp  = 5000;

leftP  = zeros(nSamp,3);
rightP = zeros(nSamp,3);

fprintf('Sampling workspace with %d configs...\n', nSamp);
wb = waitbar(0,'Sampling...');
for i=1:nSamp
    q = randomConfiguration(robot);      
    [~,~, pL, pR] = kinematics.getDualArmPose(robot, q);
    leftP(i,:)  = pL';
    rightP(i,:) = pR';
    if mod(i,100)==0, waitbar(i/nSamp, wb); end
end
close(wb);

fig = figure('Name','Workspace (YuMi)'); ax = axes('Parent',fig); hold(ax,'on');
plot3(ax, leftP(:,1),  leftP(:,2),  leftP(:,3),  'b.', 'MarkerSize', 2);
plot3(ax, rightP(:,1), rightP(:,2), rightP(:,3), 'r.', 'MarkerSize', 2);
show(robot, homeConfiguration(robot), 'Parent', ax, 'PreservePlot', false, 'Frames','off');
title(ax,'Figure 1: Dual-Arm Reachable Workspace (YuMi)');
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
legend(ax,'Left Reach','Right Reach','Robot');
axis(ax,'equal'); grid(ax,'on'); view(140,25);

if ~exist('report','dir'), mkdir report; end
saveas(fig, 'report/Figure1_Workspace.png');
end
