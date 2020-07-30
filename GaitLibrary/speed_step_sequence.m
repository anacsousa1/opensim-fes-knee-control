%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function ref = speed_step_sequence( time )
% by Antonio Padilha L. Bo (antonio.plb@lara.unb.br)
% modified by Ana de Sousa (anacsousa@lara.unb.br)
% May 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ref = speed_step_sequence( controlIteration )

    global Ts
    t = 20.0;

    % define speed
    low_vel = deg2rad(200); 
    high_vel = deg2rad(300);
    
    % if time < 10 sec
    if controlIteration*Ts < t
        ref = low_vel;
    else
        ref = high_vel;
    end   

end