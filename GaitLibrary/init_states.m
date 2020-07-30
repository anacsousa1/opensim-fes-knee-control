%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function state = init_states( model, state )
% by Ana de Sousa (anacsousa@lara.unb.br)
% April 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function state = init_states( model_leg, state )

    % locking corresponding states and definint initial pose
    editableCoordSet = model_leg.updCoordinateSet();
    
%     editableCoordSet.get('pelvis_tx').setLocked(state, false);
%     editableCoordSet.get('pelvis_tx').setValue(state, deg2rad(0));
    
%     editableCoordSet.get('pelvis_ty').setLocked(state, false);
%     editableCoordSet.get('pelvis_ty').setValue(state, deg2rad(0));
    
    editableCoordSet.get('hip_flexion_r').setValue(state, deg2rad(20));
    editableCoordSet.get('hip_flexion_r').setLocked(state, false);
    
    editableCoordSet.get('knee_angle_pat_r').setLocked(state, false);
    
    editableCoordSet.get('ankle_angle_r').setValue(state, deg2rad(90));
    editableCoordSet.get('ankle_angle_r').setLocked(state, true);
    
    editableCoordSet.get('knee_angle_r').setValue(state, deg2rad(-33));
%     editableCoordSet.get('knee_angle_r').setValue(state, deg2rad(-34.6994)); % max
%     editableCoordSet.get('knee_angle_r').setValue(state, deg2rad(1.9533)); % min 
    editableCoordSet.get('knee_angle_r').setLocked(state, false);
    
    % recalculate the derivatives after the coordinate changes
    model_leg.computeStateVariableDerivatives(state);

end

