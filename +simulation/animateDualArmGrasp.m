function animateDualArmGrasp(opts)

if nargin<1, opts=struct; end
if ~isfield(opts,'saveVideo'), opts.saveVideo=false; end
if ~isfield(opts,'duration'),  opts.duration=20;     end

robot  = kinematics.createDualArmExo();
q_home = homeConfiguration(robot);

% Virtual object
objCenter = [0.55; 0; 0.25];
objW = 0.22; objL = 0.14; objH = 0.10;

% Target poses
sideOffsetX = 0.05;
pL = objCenter + [ sideOffsetX;  objW/2; 0];
pR = objCenter + [-sideOffsetX; -objW/2; 0];
roll = deg2rad(15);
R_L = eul2rotm([0, pi/2, roll]);
R_R = eul2rotm([0, -pi/2, -roll]);
T_L = [R_L, pL; 0 0 0 1];
T_R = [R_R, pR; 0 0 0 1];

% IK
fprintf('Bi-Manual IK...\n');
[q_goal, info] = kinematics.solveBiIK(robot, q_home, T_L, T_R);
if info.ExitFlag <= 0
    warning('gIK ExitFlag=%d (continuing)', info.ExitFlag);
end

% Trajectory params
segmentT  = 5;
fps       = 30;
steps     = segmentT * fps;
t_samples = linspace(0, segmentT, steps);

% Video writer helper now included below
[writer, rec] = openVideoIfPossible('report/Animation2_Grasp_YuMi', fps, opts.saveVideo);

% Initial figure
fig = figure('Name','Animation 2: Bi-Manual Grasp (YuMi)');
ax  = axes('Parent',fig); cla(ax); hold(ax,'on');
show(robot, q_home, 'Parent', ax, 'PreservePlot', false, 'Frames','off');
drawnow;
title(ax,'Animation 2: Bi-Manual Grasp (YuMi)');
axis(ax, [-0.2 1.0 -0.6 0.6 -0.1 0.8]);
axis(ax,'equal'); grid(ax,'on'); view(ax,60,15);
camlight(ax,'headlight'); lighting(ax,'gouraud');
drawBox(ax, objCenter, [objL objW objH], [0.1 0.7 0.2], 0.6, [0 0 0]);

% Pre-allocate
pathL = zeros(0,3);
pathR = zeros(0,3);
allQ  = zeros(0,14);

% Animation loop with debug prints
t0 = tic;
fprintf('Starting grasp animation loop (duration = %0.1f s)...\n', opts.duration);
while toc(t0) < opts.duration
    % Segment A
    fprintf('  Segment A: generating %d steps...\n', steps);
    [qAB, ~, ~] = quinticpolytraj([q_home' q_goal'], [0 segmentT], t_samples);
    qAB = qAB.'; allQ = [allQ; qAB];
    for i = 1:size(qAB,1)
        show(robot, qAB(i,:), 'Parent', ax, 'PreservePlot', false, 'Frames','off', 'FastUpdate', true);
        drawnow;
        [~,~,pLnow,pRnow] = kinematics.getDualArmPose(robot, qAB(i,:));
        pathL(end+1,:) = pLnow';
        pathR(end+1,:) = pRnow';
        if rec, writeVideo(writer, getframe(fig)); end
        if toc(t0)>=opts.duration, break; end
    end
    fprintf('    After Segment A: total frames = %d\n', size(allQ,1));
    if toc(t0)>=opts.duration, break; end

    % Segment B
    fprintf('  Segment B: generating %d steps...\n', steps);
    [qBA, ~, ~] = quinticpolytraj([q_goal' q_home'], [0 segmentT], t_samples);
    qBA = qBA.'; allQ = [allQ; qBA];
    for i = 1:size(qBA,1)
        show(robot, qBA(i,:), 'Parent', ax, 'PreservePlot', false, 'Frames','off', 'FastUpdate', true);
        drawnow;
        [~,~,pLnow,pRnow] = kinematics.getDualArmPose(robot, qBA(i,:));
        pathL(end+1,:) = pLnow';
        pathR(end+1,:) = pRnow';
        if rec, writeVideo(writer, getframe(fig)); end
        if toc(t0)>=opts.duration, break; end
    end
    fprintf('    After Segment B: total frames = %d\n', size(allQ,1));
end
fprintf('Finished animation loop: collected %d EE poses and %d joint samples\n', size(pathL,1), size(allQ,1));

% Close video writer
if rec, close(writer); end
hold(ax,'off');

% Plots
utils.plotEETrajectories(pathL, pathR, objCenter);
t_vec   = (0:size(allQ,1)-1)'/fps;
qd_traj = [zeros(1,14); diff(allQ)] * fps;
utils.plotJointsVsTime(t_vec, allQ, qd_traj);

end

%% Helper: drawBox
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

%% Helper: openVideoIfPossible
function [vw, recording] = openVideoIfPossible(baseName, fps, wantSave)
vw = []; recording = false;
if ~wantSave, return; end
if ~exist('report','dir'), mkdir('report'); end
try
    profs = string({VideoWriter.getProfiles.Name});
    if any(profs == "MPEG-4")
        vw = VideoWriter(fullfile('report',[baseName,'.mp4']),'MPEG-4');
    elseif any(profs == "Motion JPEG AVI")
        vw = VideoWriter(fullfile('report',[baseName,'.avi']),'Motion JPEG AVI');
    else
        warning('No video profile; running without recording.');
        return;
    end
    vw.FrameRate = fps;
    open(vw);
    recording = true;
catch ME
    warning('Video off: %s', ME.message);
% Save joint history so we can plot later
if ~exist('report','dir'), mkdir('report'); end
save(fullfile('report','jointData.mat'), 't_vec', 'allQ', 'qd_traj');
fprintf('Saved jointData.mat with %d samples\n', size(allQ,1));

end
end