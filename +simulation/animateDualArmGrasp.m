function animateDualArmGrasp(opts)
% simulation.animateDualArmGrasp(opts)
% opts.saveVideo=false|true, opts.duration=seconds

if nargin<1, opts=struct; end
if ~isfield(opts,'saveVideo'), opts.saveVideo=false; end
if ~isfield(opts,'duration'),  opts.duration=20;     end

robot  = kinematics.createDualArmExo();
q_home = homeConfiguration(robot);

% ---------- Virtual object (bigger, vivid color, proper box) ----------
objCenter = [0.55; 0; 0.25];    % push farther and a bit higher
objW = 0.22;                    % width (y)
objL = 0.14;                    % length (x)
objH = 0.10;                    % height (z)

% ---------- Target poses: side approach + small roll to avoid collision ----------
sideOffsetX = 0.05;               % meters
pL = objCenter + [ sideOffsetX;  objW/2; 0];
pR = objCenter + [-sideOffsetX; -objW/2; 0];

roll = deg2rad(15);
R_L  = eul2rotm([0,  pi/2,  roll]);    % ZYX: yaw=0, pitch=+90°, roll=+15°
R_R  = eul2rotm([0, -pi/2, -roll]);    % yaw=0, pitch=-90°, roll=-15°

T_L = eye(4); T_L(1:3,1:3)=R_L; T_L(1:3,4)=pL;
T_R = eye(4); T_R(1:3,1:3)=R_R; T_R(1:3,4)=pR;

% ---------- Simultaneous IK ----------
fprintf('Bi-Manual IK...\n');
[q_goal, info] = kinematics.solveBiIK(robot, q_home, T_L, T_R);
if info.ExitFlag <= 0
    warning('gIK ExitFlag=%d (continuing)', info.ExitFlag);
end

% ---------- Trajectory params ----------
segmentT   = 5; 
fps        = 30; 
steps      = segmentT*fps;
t_samples  = linspace(0, segmentT, steps);

% ---------- Prepare video if requested ----------
[writer, rec] = openVideoIfPossible('report/Animation2_Grasp_YuMi', fps, opts.saveVideo);

% ---------- Figure & object drawing ----------
fig = figure('Name','Animation 2: Bi-Manual Grasp (YuMi)');
ax  = axes('Parent',fig); cla(ax); hold(ax,'on');
show(robot, q_home, 'Parent', ax, 'PreservePlot', false, 'Frames','off');
drawnow;
title(ax,'Animation 2: Bi-Manual Grasp (YuMi)');
axis(ax, [-0.2 1.0 -0.6 0.6 -0.1 0.8]); axis(ax,'equal'); grid(ax,'on'); view(ax,60,15);
camlight(ax,'headlight'); lighting(ax,'gouraud');

drawBox(ax, objCenter, [objL objW objH], [0.1 0.7 0.2], 0.6, [0 0 0]);

% ---------- Pre-allocate storage ----------
pathL = zeros(0,3);
pathR = zeros(0,3);
allQ  = zeros(0,14);

t0 = tic;
while toc(t0) < opts.duration
    %% A) home -> goal
    [qAB, ~, ~] = quinticpolytraj([q_home' q_goal'], [0 segmentT], t_samples);
    qAB = qAB.';                      % [steps x 14]
    allQ = [allQ; qAB];              %#ok<AGROW>
    for i = 1:size(qAB,1)
        show(robot, qAB(i,:), 'Parent', ax, 'PreservePlot', false, 'Frames','off', 'FastUpdate', true);
        drawnow;  % <-- force screen update
        [~,~, pLnow, pRnow] = kinematics.getDualArmPose(robot, qAB(i,:));
        pathL(end+1,:) = pLnow';     %#ok<AGROW>
        pathR(end+1,:) = pRnow';     %#ok<AGROW>
        if rec, writeVideo(writer, getframe(fig)); end
        if toc(t0)>=opts.duration, break; end
    end
    if toc(t0)>=opts.duration, break; end

    %% B) goal -> home
    [qBA, ~, ~] = quinticpolytraj([q_goal' q_home'], [0 segmentT], t_samples);
    qBA = qBA.';                      % [steps x 14]
    allQ = [allQ; qBA];              %#ok<AGROW>
    for i = 1:size(qBA,1)
        show(robot, qBA(i,:), 'Parent', ax, 'PreservePlot', false, 'Frames','off', 'FastUpdate', true);
        drawnow;  % <-- force screen update
        [~,~, pLnow, pRnow] = kinematics.getDualArmPose(robot, qBA(i,:));
        pathL(end+1,:) = pLnow';     %#ok<AGROW>
        pathR(end+1,:) = pRnow';     %#ok<AGROW>
        if rec, writeVideo(writer, getframe(fig)); end
        if toc(t0)>=opts.duration, break; end
    end
end

% ---------- Close video writer ----------
if rec, close(writer); end
hold(ax,'off');

% ---------- Deliverable plots ----------
utils.plotEETrajectories(pathL, pathR, objCenter);

% Build time vector & compute velocities
t_vec   = (0:size(allQ,1)-1)'/fps;
qd_traj = [zeros(1,14); diff(allQ)] * fps;

utils.plotJointsVsTime(t_vec, allQ, qd_traj);

end


%% Helper to draw the object box
function drawBox(ax, center, dims, faceColor, alpha, edgeColor)
  Lx=dims(1); Ly=dims(2); Lz=dims(3);
  cx=center(1); cy=center(2); cz=center(3);
  x = cx + 0.5*[-Lx -Lx -Lx -Lx  Lx  Lx  Lx  Lx];
  y = cy + 0.5*[-Ly -Ly  Ly  Ly -Ly -Ly  Ly  Ly];
  z = cz + 0.5*[-Lz  Lz  Lz -Lz -Lz  Lz  Lz -Lz];
  V = [x(:) y(:) z(:)];
  F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
  patch('Parent',ax,'Faces',F,'Vertices',V,'FaceColor',faceColor,...
        'FaceAlpha',alpha,'EdgeColor',edgeColor,'LineWidth',1.5);
end
