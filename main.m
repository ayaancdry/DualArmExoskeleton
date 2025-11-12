% =========================================================================
% main.m  — YuMi-based assignment runner (continuous animations)
% =========================================================================
clear; clc; close all;
addpath(genpath(pwd));

fprintf('Starting Dual-Arm Exoskeleton (YuMi) Assignment...\n\n');

% --- Part 2: Workspace ---
fprintf('Running Part 2: Workspace Analysis...\n');
try
    simulation.plotWorkspace;
    fprintf('Workspace figure saved in /report.\n\n');
catch e
    fprintf('ERROR in Workspace: %s\n\n', e.message);
end

% --- Part 3A: Human-like motion (continuous; 20s; save video true/false) ---
fprintf('Running Part 3A: Human-like Motion Animation...\n');
try
    simulation.animateHumanMotion(struct('saveVideo',true,'duration',20));
    fprintf('Animation 1 completed. Video (if enabled) in /report.\n\n');
catch e
    fprintf('ERROR in Human-like Motion: %s\n\n', e.message);
end

% --- Part 3B: Bi-manual grasp (continuous; 20s; save video true/false) ---
fprintf('Running Part 3B: Bi-Manual Grasp Animation...\n');
try
    simulation.animateDualArmGrasp(struct('saveVideo',true,'duration',20));
    fprintf('Animation 2 completed. Video (if enabled) in /report.\n\n');
catch e
    fprintf('ERROR in Grasping: %s\n\n', e.message);
end

fprintf('All tasks executed.\n');
