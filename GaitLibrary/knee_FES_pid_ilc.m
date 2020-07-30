function [thisU_stim] = knee_FES_pid_ilc(ilc_memory,ILCParam,thisU_PID)

    err_past = ilc_memory(2);
    u_past = ilc_memory(1);

    alpha = ILCParam.alpha;
    beta = ILCParam.beta;
    gama = ILCParam.gama;
    
    u_ilc = u_past + gama*err_past;
    u_pid = thisU_PID;
    
    if u_ilc > 1.0
        u_ilc = 1.0
    elseif u_ilc < -1.0
        u_ilc = -1.0
    end

    thisU_stim = alpha*(u_ilc) + beta*(u_pid);
    
end

