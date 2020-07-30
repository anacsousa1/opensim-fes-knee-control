function [u, newIntegralError] = knee_FES_pid( refKnee, thisKnee, lastControlAction, lastError, lastIntegralError, Ts, PIDparam )

    % PID parameters
    Kp = PIDparam(1);
    Ki = PIDparam(2);
    Kd = PIDparam(3);

    % compute error 
    thisError = refKnee - thisKnee;

    % compute control action
    newIntegralError = lastIntegralError + thisError * Ts;
    u = Kp * thisError + Kd * (thisError - lastError) / Ts + ...
        Ki * newIntegralError; % PID

    % apply control saturation & anti-windup
    if u > 1
        u = 1;
        newIntegralError = lastIntegralError;
    elseif u < -1
        u = -1;
        newIntegralError = lastIntegralError;
    end

    % reset integral term for different muscle excitation
    if ~sign(lastControlAction*u)
        newIntegralError = 0;
    end

end

