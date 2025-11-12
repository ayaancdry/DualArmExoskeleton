function [T_left, T_right, p_left, p_right, Tall] = getDualArmPose(robot, q)
% kinematics.getDualArmPose  (YuMi, row config)
leftEE  = 'yumi_link_7_l';
rightEE = 'yumi_link_7_r';

% per-joint FK list for Part 2 (Tall{k} is 4x4 from base to body k)
allBodies = robot.BodyNames;
Tall = cell(numel(allBodies),1);
for i = 1:numel(allBodies)
    Tall{i} = getTransform(robot, q, allBodies{i});
end

T_left  = getTransform(robot, q, leftEE);
T_right = getTransform(robot, q, rightEE);

if nargout > 2
    p_left  = T_left(1:3,4);
    p_right = T_right(1:3,4);
end
end
