%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [ f_left, f_right ] = get_forces( osimModel, osimState )
% by Ana de Sousa (anacsousa@lara.unb.br)
% June 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ f_left, f_right ] = get_forces( osimModel, osimState )

    f_left = zeros(12,1);
    f_right = zeros(12,1);

    for i = 1:12
        f1 = osimModel.getForceSet().get('LPedalForce').getRecordValues(osimState);
        f2 = osimModel.getForceSet().get('RPedalForce').getRecordValues(osimState);
        
        f_left(i,1) = f1.get(i-1);
        f_right(i,1) = f2.get(i-1);
    end

end

