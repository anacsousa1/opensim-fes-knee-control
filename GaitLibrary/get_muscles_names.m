function muscleNames = get_muscles_names( osimModel )

    import org.opensim.modeling.*

    muscle_vector = osimModel.getMuscles();
    n = muscle_vector.getSize;
    if n < 1
        disp('Issue getting muscle names from model!');
    end

    % Transfer the muscles to a matlab array so I can process it more easily
    muscleNames = cell(1, n);
    for i = 1:n
        muscle = char(muscle_vector.get(i-1));
        muscleNames{i} = muscle;
    end
    
    % Removes the '.excitation' and if it has any leading _r or _l parts and removes duplicate muscle names
    muscleNames = unique(strrep(strrep(muscleNames, '.excitation', ''), '.excitation', ''));

end

