%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by Ana de Sousa (anacsousa@lara.unb.br)
% October 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic; clear; clc; close;
import org.opensim.modeling.*
addpath('OpensimLibrary');
addpath('GaitLibrary');

%% load gait 01 or 03
global refVel

%% GLOBAL VARIABLES
% timing
global Tf
global Ts
global n_samples
global n_cycles

% general variables
global ZERO
global refKnee
global measKnee
global measHip
global refHip
global musclesNames
global statesNames

% pertubation variables
global tQUADactivated
global tHAMSactivated

% general control variables
global controlTypeEXO
global controlTypeFES

global controlActionQUAD
global controlActionHAMS
global controlActionStim
global controlActionTorque

global controlErrorKnee
global controlErrorHip
global controlTime
global controlIteration

global integralErrorKnee
global integralErrorHip
global PIDparam_vector
global jcost_vector
global Kp_hat_vector
global Ki_hat_vector
global Kd_hat_vector
global PIDparam

global ilc_memory
global ilc_i

% flags
global flagSensorNoise
global flagSTIM
global flagKneeTorque
global flagKneeStart
global flagKneeRange
global flagFAT_R

%% INITIAL CONFIGURATION
if refVel == 03
    load('Dataset_hipsuit/gait03');
else
    load('Dataset_hipsuit/gait01');
end

Tf = gait_period*n_cycles; Ts = 1/50; n_samples = floor(Tf/Ts);
flagSensorNoise = 0;

tQUADactivated = 0;
tHAMSactivated = 0;

% do not change:
ZERO = 1e-3;
ilc_i = 1;
PIDparam_vector = PIDparam'.*ones(3,n_samples);% PIDparam_vector(:,1) = PIDparam;
measKnee = zeros(1,n_samples);
measHip = zeros(1,n_samples);
jcost_vector = zeros(1,n_samples);
Kp_hat_vector = PIDparam(1)*ones(1,n_samples);
Ki_hat_vector = PIDparam(2)*ones(1,n_samples);
Kd_hat_vector = PIDparam(3)*ones(1,n_samples);
controlActionQUAD = ZERO * ones(1,n_samples);
controlActionHAMS = ZERO * ones(1,n_samples);
controlActionStim = ZERO * ones(1,n_samples);
controlActionTorque = ZERO * ones(1,n_samples);
controlErrorHip = zeros(1,n_samples);
controlErrorKnee = zeros(1,n_samples);
controlTime = zeros(1,n_samples);
controlIteration = 1;
integralErrorHip = 0;
integralErrorKnee = 0;

%% INITIALIZE MODEL
disp('> Initialize simulation')
osimModel_ref_filepath   = 'Model/hip_torque100.osim';
osimModel_control_filepath   = 'Model/leg69_Forward_Default_Controls.xml';

gaitModel = Model(osimModel_ref_filepath);  % open model
gaitState = gaitModel.initSystem();         % initializing the model
gaitModel.equilibrateMuscles(gaitState);    % "solve for equilibrium actuator states"

% control function handle
controlFunctionHandle = @gait_control;      % control function handle

% get muscles and states names
musclesNames = get_muscles_names(gaitModel);
statesNames = get_states_names(gaitModel);

% define initial pose
gaitState = init_states( gaitModel, gaitState );

%% REFERENCES
perc_avg_gait = perc_avg_gait*gait_period/perc_avg_gait(end);
new_t = 0:Ts:gait_period;

hip_interp = interp1(perc_avg_gait,fitted_avg_gait_cycle_upperleg,new_t);
knee_interp = interp1(perc_avg_gait,fitted_avg_gait_cycle_lowerleg,new_t);

refHip = repmat(hip_interp,1,n_cycles);
refKnee = repmat(knee_interp,1,n_cycles);

ilc_memory = zeros(2,length(knee_interp));
ilc_memory(1,:) = repmat(knee_interp,1,1);

%% TORQUE ANALYSIS
global analyzer
analyzer = ForceReporter(gaitModel);
analyzer.setName('torque_analysis');
analyzer.setModel(gaitModel);
analyzer.includeConstraintForces(true);
analyzer.begin(gaitState);

%% CONFIG & RUN SIMULATION
disp('> Run simulation')

% integrate plant using Matlab integration
timeSpan = [0 Tf]; integratorName = 'ode15s'; 
integratorOptions = odeset('AbsTol', 1E-5','MaxStep',.1*Ts);

% run simulation using function from Dynamic Walking example
motionData = IntegrateOpenSimPlant(gaitModel, controlFunctionHandle, timeSpan, ...
    integratorName, integratorOptions);

analyzer.end(gaitState);
analyzer.printResults('reporter');

%% SHOW RESULTS
n_samples = controlIteration;

jcost_vector = jcost_vector(1:n_samples);
refKnee = refKnee(1:n_samples);
refHip = refHip(1:n_samples);
measKnee = measKnee(1:n_samples);
measHip = measHip(1:n_samples);
controlActionStim = controlActionStim(1:n_samples);
controlActionHAMS = controlActionHAMS(1:n_samples);
controlActionQUAD = controlActionQUAD(1:n_samples);
controlActionTorque = controlActionTorque(1:n_samples);
controlErrorHip = controlErrorHip(1:n_samples);
controlErrorKnee = controlErrorKnee(1:n_samples);
controlTime = controlTime(1:n_samples);
PIDparam_vector = PIDparam_vector(:,1:n_samples);
Kp_hat_vector = Kp_hat_vector(1:n_samples);
Ki_hat_vector = Ki_hat_vector(1:n_samples);
Kd_hat_vector = Kd_hat_vector(1:n_samples);

%% CREATE .STO FILE FOR VISUALIZATION, SAVE WORKSPACE AND SAVE FIGURE
gait_plot;
Nsamples = length(motionData.data(:,1)); Nstates = length(statesNames);
% name of the file
str_name = 'gait_';
str_name = strcat(str_name,'_Tf',num2str(Tf),'_CE_',controlTypeEXO,'_CF_',controlTypeFES,'_STIM',num2str(flagSTIM),...
    '_F', num2str(flagFAT_R), '_R', num2str(refVel),...
    '_Kmax',num2str(flagKneeTorque),'_s',num2str(flagKneeStart),'_r',num2str(flagKneeRange));

    % create .STO
    str = strjoin(statesNames,'\t');
    header = ['gait_simulation \nversion=1 \nnRows=' num2str(Nsamples) ' \nnColumns=' num2str(Nstates+1) '\ninDegrees=no \nendheader \ntime	' str '\n'];

    fid = fopen(strcat('Results/',str_name,'.sto'),'wt');
    fprintf(fid,header); fclose(fid);

    fid = fopen(strcat('Results/',str_name,'.sto'),'a+');
    for i = 1:Nsamples
        fprintf(fid,'\t%f',motionData.data(i,:)); fprintf(fid,'\n');
    end
    fclose(fid);

    % save .MAT
    varlist = {'gaitModel','gaitState','analyzer'}; clear(varlist{:});  % clear opensim var
    save(strcat('Results\',str_name,'.mat'));                 % save data

%% The end!
fprintf('\n'); toc; disp('> THE END');


