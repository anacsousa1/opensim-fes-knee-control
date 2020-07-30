function statesNames = get_states_names( osimModel )

    import org.opensim.modeling.*

    states_vector = osimModel.getStateVariableNames();
    n = states_vector.getSize;
    if n < 1
        disp('Issue getting states names from model!');
    end

    % Transfer the muscles to a matlab array so I can process it more easily
    statesNames = cell(1, n);
    for i = 1:n
        state = char(states_vector.get(i-1));
        statesNames{i} = state;
    end
    
    % Removes the '.excitation' and if it has any leading _r or _l parts and removes duplicate muscle names
%     statesNames = unique(strrep(strrep(statesNames, '.excitation', ''), '.excitation', ''));

end

