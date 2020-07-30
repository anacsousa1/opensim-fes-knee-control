function modelControls = gait_control(osimModel, osimState)
    import org.opensim.modeling.*;
    
    %% GLOBAL VARIABLES
    global analyzer
    
    % timing
    global Ts
    global n_samples
    
    % general variables
    global ZERO
    ZERO = 1e-3;
    global refKnee
    global refHip
    global measKnee
    global measHip
    global musclesNames
    
    % general control variables
    global controlTypeEXO
    global controlTypeFES
    global flagSTIM

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
    
    global PIDparam
    global ESParam
    global jcost_vector
    global PIDparam_vector
    global Kp_hat_vector
    global Ki_hat_vector
    global Kd_hat_vector
    
    global ilc_memory
    global ILCParam
    global ilc_i
    
    % pertubation variables
    global tQUADactivated
    global tHAMSactivated
    global flagFAT_R
    
    % flags
    global flagSensorNoise

    %% GET INFO
    modelControls = osimModel.updControls(osimState);
    thisTime = osimState.getTime();
    
    thisStateArray = osimModel.getStateValues(osimState);    
    
    hip = thisStateArray.get(3); % knee
    knee = thisStateArray.get(5); % knee

    %% UPDATE CONTROL (only update if sampling period has passed)
    if (thisTime - controlTime(controlIteration)) >= (Ts-.02*Ts)
        
        % add noise
        if flagSensorNoise
            sigma = deg2rad(5);
            knee = knee + sigma*randn(1);
            hip = hip + sigma*randn(1);
        end
        
        controlIteration = controlIteration + 1;
        controlTime(controlIteration) = thisTime;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % CONTROLLER CODE HERE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % define reference and compute error
        thisRefKnee = -refKnee(controlIteration+1);
        thisRefKnee = deg2rad(thisRefKnee);
        thisErrorKnee = thisRefKnee - knee; % compute error
        jcost = Jfunction(thisErrorKnee,Ts);  % calculate jcost 
        
        thisRefHip = -refHip(controlIteration+1);
        thisRefHip = deg2rad(thisRefHip);
        thisErrorHip = thisRefHip - hip; % compute error 
        
        Kp_hat = 0;Ki_hat = 0;Kd_hat = 0;
        
        % print simulation evolution        
        fprintf('%d/%d: knee: %f, u_q: %f, u_h: %f, e: %f\n',controlIteration, n_samples, rad2deg(thisRefKnee), controlActionQUAD(controlIteration-1), controlActionHAMS(controlIteration-1),rad2deg(controlErrorKnee(controlIteration-1)));

        % compute stim control
        if strcmp(controlTypeFES,'PID')
            [thisU_stim, integralErrorKnee] = knee_FES_pid(thisRefKnee, knee, controlActionStim(controlIteration-1), ...
                controlErrorKnee(controlIteration-1), integralErrorKnee, Ts, PIDparam);
        elseif strcmp(controlTypeFES,'Open-loop')
            if controlIteration <= 0.1/Ts % start open-loop step excitation at this instant
                thisU_stim = 0;
            else
                thisU_stim = knee_FES_openloop( thisErrorKnee, flagSTIM );
            end
        elseif strcmp(controlTypeFES,'PID_ES')

            ESC_now.K = ESParam.K;
            ESC_now.RC = ESParam.RC;
            ESC_now.omega = ESParam.omega;
            
            ESC_now.A = ESParam.A;
            ESC_now.phase = ESParam.phase;
            
            [newKp, Kp_hat] = knee_FES_pid_es(jcost, jcost_vector(controlIteration-1), PIDparam_vector(1,controlIteration-1), Kp_hat_vector(controlIteration-1), Ts, controlTime(controlIteration), ESC_now);
            
            ESC_now.A = ESParam.A/2;
            ESC_now.phase = ESParam.phase+0.1745; %+10 graus
            ESC_now.omega = ESParam.omega-2; 
            
            [newKi, Ki_hat] = knee_FES_pid_es(jcost, jcost_vector(controlIteration-1), PIDparam_vector(2,controlIteration-1), Ki_hat_vector(controlIteration-1), Ts, controlTime(controlIteration), ESC_now);
            
            ESC_now.A = ESParam.A/4;
            ESC_now.phase = ESParam.phase+0.5236;  %+20 graus
            ESC_now.omega = ESParam.omega-4; 
            
            [newKd, Kd_hat] = knee_FES_pid_es(jcost, jcost_vector(controlIteration-1), PIDparam_vector(3,controlIteration-1), Kd_hat_vector(controlIteration-1), Ts, controlTime(controlIteration), ESC_now);

            PIDparam = [newKp,newKi,newKd];

            [thisU_stim, integralErrorKnee] = knee_FES_pid(thisRefKnee, knee, controlActionStim(controlIteration-1), ...
                controlErrorKnee(controlIteration-1), integralErrorKnee, Ts, PIDparam);
            
        elseif strcmp(controlTypeFES,'PID_ILC')
            [thisU_PID, integralErrorKnee] = knee_FES_pid(thisRefKnee, knee, controlActionStim(controlIteration-1), ...
                controlErrorKnee(controlIteration-1), integralErrorKnee, Ts, PIDparam);
            
            % after second cycle
            ilc_send(1,1) = ilc_memory(1,ilc_i);
            ilc_send(1,2) = ilc_memory(2,ilc_i);
            
            if ilc_memory(2,ilc_i) ~= 0
                [thisU_stim] = knee_FES_pid_ilc(ilc_send,ILCParam,thisU_PID);
            else
                thisU_stim = thisU_PID;
            end
                        
            %update ilc_memory
            ilc_memory(1,ilc_i) = thisU_stim;
            ilc_memory(2,ilc_i) = thisErrorKnee;
            
            % update ilc iteration
            ilc_i = ilc_i+1;
            if ilc_i > length(ilc_memory)
                ilc_i = 1;
            end
        end
                
        % compute torque control
        if strcmp(controlTypeEXO,'PID')
            [thisU_torque, integralErrorHip] = hip_EXO_pid(thisRefHip, hip, controlActionTorque(controlIteration-1), ...
                controlErrorHip(controlIteration-1), integralErrorHip, Ts);
                        
        elseif strcmp(controlTypeEXO,'Open-loop')
            if controlIteration <= 0.1/Ts % start open-loop step excitation at this instant
                thisU_torque = 0;
            else
                thisU_torque = hip_EXO_openloop( thisTime );
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % END OF CONTROLLER CODE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        if thisU_stim >= 0
            stimQ = abs(thisU_stim);
            stimH = 0;
        else
            stimQ = 0;
            stimH = 0.5*abs(thisU_stim);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % FATIGUE HERE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if flagFAT_R > 0
            % QUAD FATIGUE
            if stimQ > 0
                tQUADactivated = tQUADactivated + Ts;
                if flagFAT_R > 0 
                    fQUAD = fatigue( tQUADactivated, flagFAT_R );
                else
                    fQUAD = 0;
                end
                stimQ = stimQ - fQUAD;
                if stimQ < 0
                    stimQ = 0;
                end
            end

            % HAMS FATIGUE
            if stimH > 0
                tHAMSactivated = tHAMSactivated + Ts;
                if flagFAT_R > 0 
                    fHAMS = fatigue( tHAMSactivated, flagFAT_R );
                else
                    fHAMS = 0;
                end
                stimH = stimH - fHAMS;
                if stimH < 0
                    stimH = 0;
                end
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % END OF FATIGUE HERE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % update variables
        controlActionQUAD(controlIteration) = stimQ;
        controlActionHAMS(controlIteration) = stimH;
        refKnee(controlIteration) = thisRefKnee;
        refHip(controlIteration) = thisRefHip;
        controlActionStim(controlIteration) = thisU_stim;
        controlActionTorque(controlIteration) = thisU_torque;
        controlErrorKnee(controlIteration) = thisErrorKnee;
        controlErrorHip(controlIteration) = thisErrorHip;
        measKnee(controlIteration) = knee;
        measHip(controlIteration) = hip;
        PIDparam_vector(:,controlIteration) = PIDparam';
        jcost_vector(controlIteration) = jcost;
        Kp_hat_vector(controlIteration) = Kp_hat;
        Ki_hat_vector(controlIteration) = Ki_hat;
        Kd_hat_vector(controlIteration) = Kd_hat;
        
        analyzer.step(osimState,controlIteration);
    end
            
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % END OF MUSCLE ENCODING
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % define muscle actuation
    rect_fem_r = controlActionQUAD(controlIteration);
    vas_int_r = controlActionQUAD(controlIteration);
    bifemlh_r = controlActionHAMS(controlIteration);
    
    % make sure excitation is between zero and 1
    if rect_fem_r < ZERO
        rect_fem_r = ZERO;
    end
    
    if vas_int_r < ZERO
        vas_int_r = ZERO;
    end
    
    if bifemlh_r < ZERO
        bifemlh_r = ZERO;
    end
        
    osimModel.updActuators().get('hip_torque').addInControls(Vector(1, controlActionTorque(controlIteration)), modelControls);

    %assing excitation to muscles
    for i = 1:length(musclesNames)
        if strcmp(musclesNames{i},'rect_fem_r')
            thisExcitation = Vector(1, rect_fem_r);            
        elseif strcmp(musclesNames{i},'vas_int_r')
            thisExcitation = Vector(1, vas_int_r);
        elseif strcmp(musclesNames{i},'bifemlh_r')
            thisExcitation = Vector(1, bifemlh_r);
        else
            thisExcitation = Vector(1, ZERO);
        end
        % update modelControls with the new values% TODO: not sure if it is working
        osimModel.updActuators().get(musclesNames{i}).addInControls(thisExcitation, modelControls);
    end

end

