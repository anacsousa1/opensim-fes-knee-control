%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function ref = position_senoidal_sequence( time )
% by Ana de Sousa (anacsousa@lara.unb.br)
% January 2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ ref ] = position_senoidal_sequence( time )

    global Ts
    
    time = time*Ts;
    T = 4; f = 1/T;
    A = 50/2;
    B = -55;
    w = 2*pi*f;
    
    ref = A*cos(w*time+pi)+B;

end

