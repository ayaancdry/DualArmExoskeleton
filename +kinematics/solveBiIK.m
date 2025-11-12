function [q_sol, info] = solveBiIK(robot, q_seed, T_left, T_right)
% kinematics.solveBiIK  Simultaneous IK for left/right YuMi EEs
gik = generalizedInverseKinematics( ...
    'RigidBodyTree', robot, ...
    'ConstraintInputs', {'pose','pose'});

leftEE  = 'yumi_link_7_l';
rightEE = 'yumi_link_7_r';

cL = constraintPoseTarget(leftEE);   cL.TargetTransform  = T_left;
cR = constraintPoseTarget(rightEE);  cR.TargetTransform  = T_right;

cL.OrientationTolerance = deg2rad(1);  cL.PositionTolerance = 1e-3;
cR.OrientationTolerance = deg2rad(1);  cR.PositionTolerance = 1e-3;

[q_sol, info] = gik(q_seed, cL, cR);
end
