% Sample bi-manual IK runs
clc; clear;
robot = loadrobot('abbYumi','DataFormat','row');

% Define three sample object centers in workspace:
targets = [ 0.5,  0.2, 0.3;
            0.4, -0.1, 0.4;
            0.6,  0.0, 0.2 ];

fprintf('Bi-manual IK Demo\n');
for idx = 1:size(targets,1)
  obj = targets(idx,:)';
  side = 0.05;
  pL = obj + [ side;  0.11; 0];
  pR = obj + [-side; -0.11; 0];

  R_L = eul2rotm([0 pi/2 deg2rad(15)]);
  R_R = eul2rotm([0 -pi/2 deg2rad(-15)]);
  T_L = [R_L, pL; 0 0 0 1];
  T_R = [R_R, pR; 0 0 0 1];

  [q_sol, info] = kinematics.solveBiIK(robot, homeConfiguration(robot), T_L, T_R);
  fprintf('Target #%d: center = [%0.2f %0.2f %0.2f]\n', idx, obj);
  fprintf('  ExitFlag = %d\n', info.ExitFlag);
  fprintf('  q_sol = [%s]\n\n', sprintf('%0.3f ', q_sol));
end
