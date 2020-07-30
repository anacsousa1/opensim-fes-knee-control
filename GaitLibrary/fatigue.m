function [f] = fatigue( t, F )
    f = 1 - exp(-t/F);
end