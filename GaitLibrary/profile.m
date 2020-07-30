%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [ quad_l, quad_r ] = profile( angle, speed , speed_ref, correction)
% by Ana de Sousa (anacsousa@lara.unb.br)
% June 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ quad_l, quad_r ] = profile( angle, speed , correction)
    
    speed = -speed;
    speed = rad2deg(speed);             
    speed_max = 500;   
    
    if correction == 1
        correction_factor = -30;
        if speed <= speed_max
            theta_shift = (speed / speed_max) * correction_factor;
        elseif speed < 0
            theta_shift = 0;
        else
            theta_shift = correction_factor;
        end
    else
        theta_shift = 0;
    end
    
    % right leg
    right_quad_start_ang = 300;
    right_quad_range =  100;
    
    % left leg
    left_quad_start_ang = right_quad_start_ang - 180;
    left_quad_range = right_quad_range;
    
    %% Get angle correctly    
    % gearToGround is counter-clockwise, take the absolute angle and speed
    angle = -angle;
    
    % gearToGround increases forever, wrap to 0 to 360 deg
    angle = wrapTo360(rad2deg(angle)); 
    
    %% Stimulate if it is in the stim zone  (right quad)
    start_ang = right_quad_start_ang - theta_shift;
    end_ang = wrapTo360(start_ang + right_quad_range);
    
    d1 = rem(angle-start_ang+180+360,360)-180;
    d2 = rem(end_ang-angle+180+360,360)-180;
    
    if (d1 >=0) && (d2 >= 0)
        quad_r = 1;
    else
        quad_r = 0;
    end
    
    %% Stimulate if it is in the stim zone (left quad)
    start_ang = left_quad_start_ang - theta_shift;
    end_ang = wrapTo360(start_ang + left_quad_range);
    
    d1 = rem(angle-start_ang+180+360,360)-180;
    d2 = rem(end_ang-angle+180+360,360)-180;
    
    if  (d1 >=0) && (d2 >= 0)
        quad_l = 1;
    else
        quad_l = 0;
    end
end