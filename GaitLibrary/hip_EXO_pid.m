function [u, newIntegralError] = hip_EXO_pid( refHip, thisHip, lastControlAction, lastError, lastIntegralError, Ts )

    % PID parameters
    Kp = 0.6;
    Ki = 0.1;
    Kd = 0.0;
    
    % compute error 
    thisError = refHip - thisHip;

    % compute control action
    newIntegralError = lastIntegralError + thisError * Ts;
    %u = Kp * thisError; % P
    %u = Kp * thisError + Kd * (thisError - lastError) / Ts; % PD
    u = Kp * thisError + Kd * (thisError - lastError) / Ts + ...
        Ki * newIntegralError; % PID

    % apply control saturation & anti-windup
    if u > 1
        u = 1;
        newIntegralError = lastIntegralError;
    elseif u <= -1
        u = -1;
        newIntegralError = lastIntegralError;
    end

    % reset integral term for different muscle excitation
    if ~sign(lastControlAction*u)
        newIntegralError = 0;
    end

end

