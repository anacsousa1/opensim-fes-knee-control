% Import Java Library
clear; clc;
import org.opensim.modeling.*
 
%% Open the model
osimModel = Model('leg6dof9stand_ankleLockedNewForces.osim');
osimModel.setName('knee_torque100');
 
%% Utilities
zeroVec3 = ArrayDouble.createVec3(0);
 
%% Add Torque Control
% Create Torque
femur = osimModel.updBodySet().get('femur_r');
tibia = osimModel.updBodySet().get('tibia_r');
 
knee_Actuator = TorqueActuator (femur, tibia, Vec3(0,0,-1), true);
knee_Actuator.setName('knee_torque');
knee_Actuator.setOptimalForce(100.0);

% Add the force to the model
osimModel.addForce(knee_Actuator);
 
% Print a new model file
osimModel.print('hip_torque100.osim');

