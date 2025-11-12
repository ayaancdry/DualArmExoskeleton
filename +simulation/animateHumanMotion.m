function animateHumanMotion(opts)
% simulation.animateHumanMotion(opts)
% opts.saveVideo=false|true, opts.duration=seconds

if nargin<1, opts=struct; end
if ~isfield(opts,'saveVideo'), opts.saveVideo=false; end
if ~isfield(opts,'duration'),  opts.duration=20;     end

robot  = kinematics.createDualArmExo();
q_home = homeConfiguration(robot); % 1x14 (row)

% Waypoints
q0 = q_home;
q_up   = q0;  q_up(2)=q0(2)-pi/2;  q_up(9)=q0(9)-pi/2;  q_up(4)=q0(4)+pi/2;  q_up(11)=q0(11)+pi/2;
q_wide = q0;  q_wide(1)=q0(1)+pi/2; q_wide(8)=q0(8)-pi/2; q_wide(4)=q0(4)+pi/4; q_wide(11)=q0(11)+pi/4;
keyframes = [q0; q_up; q_wide; q0];

segmentT=5; fps=30; stepsPerSeg=segmentT*fps;

[writer, rec] = openVideoIfPossible('report/Animation1_HumanLike_YuMi', fps, opts.saveVideo);

% Figure (clean)
fig = figure('Name','Animation 1: Human-Like Motion (YuMi)');
ax  = axes('Parent',fig);
cla(ax); hold(ax,'off');
show(robot, q0, 'Parent', ax, 'PreservePlot', false, 'Frames','off');
title(ax,'Animation 1: Human-Like Motion (YuMi)');
axis(ax, [-1 1 -1.5 1.5 -0.5 1.5]); axis(ax,'equal'); grid(ax,'on'); view(ax,140,25);
camlight(ax,'headlight'); lighting(ax,'gouraud'); drawnow;

t0=tic;
while toc(t0) < opts.duration
    for s=1:size(keyframes,1)-1
        [qseg, ~, ~] = quinticpolytraj([keyframes(s,:).', keyframes(s+1,:).'], ...
                               [0 segmentT], linspace(0,segmentT,stepsPerSeg));

        qseg=qseg.'; % [N x 14]
        for i=1:size(qseg,1)
            show(robot, qseg(i,:), 'Parent', ax, 'PreservePlot', false, 'Frames','off', 'FastUpdate', true);
            drawnow;
            if rec, writeVideo(writer, getframe(fig)); end
            if toc(t0)>=opts.duration, break; end
        end
        if toc(t0)>=opts.duration, break; end
    end
end
if rec, close(writer); end
end

function [vw, recording] = openVideoIfPossible(baseName, fps, wantSave)
vw=[]; recording=false; if ~wantSave, return; end
if ~exist('report','dir'), mkdir report; end
try
    profs=string({VideoWriter.getProfiles.Name});
    if any(profs=="MPEG-4"), vw=VideoWriter([baseName '.mp4'],'MPEG-4');
    elseif any(profs=="Motion JPEG AVI"), vw=VideoWriter([baseName '.avi'],'Motion JPEG AVI');
    else, warning('No video profile; running without recording.'); return;
    end
    vw.FrameRate=fps; open(vw); recording=true;
catch ME
    warning('Video off: %s', ME.message);
end
end
