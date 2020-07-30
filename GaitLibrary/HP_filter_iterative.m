function [y] = HP_filter_iterative(x, last_x, last_y, dt, a)
%     a = cut_freq/(cut_freq+dt)
%     cut_freq = 0.00099;
    y = a*(last_y + x - last_x);
end