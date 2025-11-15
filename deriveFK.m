function deriveFK()
% kinematics.deriveFK  — Build DH transforms and compute T0_7 for YuMi arm

  %--- DH parameters from your table (i = 1..7) ---
  a     = [-0.03,  0.03,   0.0405, 0.0405, 0.027,  -0.027, 0];
  alpha = deg2rad([-90,    90,    -90,    -90,    -90,    90,     0]);
  d     = [ 0.166,  0,      0.2515, 0,      0.265,  0,      0.036];
  % θ ranges are irrelevant here; use homeConfiguration(robot) instead:
  q_home = homeConfiguration(loadrobot('abbYumi','DataFormat','row'));

  %--- Compute successive transforms ---
  T = eye(4);
  fprintf('---- Individual DH Transforms (i = 1..7) ----\n');
  for i = 1:7
    theta = q_home(i);
    A_i = dhTransform(a(i), alpha(i), d(i), theta);
    fprintf('T_%d_%d =\n', i-1, i);
    disp(A_i);
    T = T * A_i;
  end

  fprintf('---- Composite T_0_7 (end-effector) ----\n');
  disp(T);

end

function A = dhTransform(a, alpha, d, theta)
% Standard DH: A = [ Rot(z,θ) Trans(z,d) Trans(x,a) Rot(x,α) ]
  A = [ ...
    cos(theta),           -sin(theta),           0,            a;
    sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;
    sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d;
    0,                     0,                     0,            1 ];
end
