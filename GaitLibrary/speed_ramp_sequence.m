%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function ref = speed_ramp_sequence( time )
% by Antonio Padilha L. Bo (antonio.plb@lara.unb.br)
% modified by Ana de Sousa (anacsousa@lara.unb.br)
% June 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ref = speed_ramp_sequence( controlIteration )

    global Ts
    
    % define times
    t1 = 20.0; n1 = round(t1/Ts);
    t2 = 23.0; n2 = round(t2/Ts);
    t = controlIteration*Ts;  

    % define speed
    low_vel = deg2rad(300); 
    high_vel = deg2rad(400);

    % ramp
    if t < t1
        ref = low_vel;
    elseif t < t2
        ref = (high_vel-low_vel)/(n2-n1)*(controlIteration-n1) + low_vel;
    else
        ref = high_vel;        
    end

end