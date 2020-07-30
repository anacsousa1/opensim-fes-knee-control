function [u, uhat] = knee_FES_pid_es(j_cost, last_j_cost, last_y, last_uhat, dt, t, ESCparam)

    %% filter (1)
    hp_result = HP_filter_iterative(j_cost, last_j_cost, last_y, dt, ESCparam.RC);
    
    %% * a.sin(wt-phase) (2)
    xi = hp_result*sin(ESCparam.omega*t + ESCparam.phase);
    
    %% integrate (3)
    uhat = last_uhat + xi*ESCparam.K*dt;
    
    %% + a.sin(wt) (4)
    u = uhat + ESCparam.A*sin(ESCparam.omega*t + ESCparam.phase);
    
    %% parameters cannot be lower than zero
    u(u<0)=0;
    uhat(uhat<0)=0;
    
end

