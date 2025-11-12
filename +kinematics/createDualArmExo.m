function robot = createDualArmExo()
% kinematics.createDualArmExo -> ABB YuMi with numeric row configs
robot = loadrobot('abbYumi', 'DataFormat', 'row', 'Gravity', [0 0 -9.81]);
end
