clear; clc; close all;
addpath(genpath(pwd));  

fprintf('Starting Dual-Arm Exoskeleton (YuMi) Assignment...\n\n');

% FK derivation demo 
fprintf('Running FK derivation demo...\n');
deriveFK();   

% Workspace 
fprintf('Running Part 2: Workspace Analysis...\n');
try
    simulation.plotWorkspace;
    fprintf('Workspace figure saved in /report.\n\n');
catch e
    fprintf('ERROR in Workspace: %s\n\n', e.message);
end

% Human-like motion 
fprintf('Running Part 3A: Human-like Motion Animation...\n');
try
    fprintf('About to call simulation.animateHumanMotion at %s\n', datestr(now,'HH:MM:SS'));
    simulation.animateHumanMotion(struct('saveVideo',true,'duration',20));
    fprintf('Returned from simulation.animateHumanMotion at %s\n', datestr(now,'HH:MM:SS'));
    fprintf('Animation 1 completed. Video (if enabled) in /report.\n\n');
catch e
    fprintf('ERROR in Human-like Motion: %s\n\n', e.message);
end

% Bi-manual grasp 
fprintf('Running Part 3B: Bi-Manual Grasp Animation...\n');
try
    fprintf('About to call simulation.animateDualArmGrasp at %s\n', datestr(now,'HH:MM:SS'));
    simulation.animateDualArmGrasp(struct('saveVideo',true,'duration',20));
    fprintf('Returned from simulation.animateDualArmGrasp at %s\n', datestr(now,'HH:MM:SS'));
    fprintf('Animation 2 completed. Video (if enabled) in /report.\n\n');
catch e
    fprintf('ERROR in Grasping: %s\n\n', e.message);
end

% IK demo
fprintf('Running bi-manual IK demo...\n');
demoIK();

fprintf('All tasks executed.\n');
