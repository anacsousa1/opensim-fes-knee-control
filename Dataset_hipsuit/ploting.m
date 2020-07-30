
clear; clc; close all;
figure();

load('gait01.mat');
t = linspace(0,gait_period,length(fitted_avg_gait_cycle_upperleg));

subplot(2,2,1); hold on; title('Reference 0.1m/s');
    plot(t,fitted_avg_gait_cycle_upperleg, 'Color',[37/255,52/255,148/255]);
    xlim([0,t(end)]); ylim([-20,10]);
    ylabel('Hip trajectory [°]');
    
hold off; subplot(2,2,3); hold on; 
    plot(t,-fitted_avg_gait_cycle_lowerleg,'Color',[148/255,52/255,37/255]);
    xlim([0,t(end)]); ylim([-40,10]);
    ylabel('Knee trajectory [°]');
    xlabel('Time [s]');


load('gait03.mat');
t = linspace(0,gait_period,length(fitted_avg_gait_cycle_upperleg));

hold off; subplot(2,2,2); hold on;  title('Reference 0.3m/s');
    plot(t,fitted_avg_gait_cycle_upperleg, 'Color',[37/255,52/255,148/255]);
    xlim([0,t(end)]); ylim([-20,10]);
    
hold off; subplot(2,2,4); hold on;  
    plot(t,-fitted_avg_gait_cycle_lowerleg,'Color',[148/255,52/255,37/255]);
    xlim([0,t(end)]);  ylim([-40,10]);
    xlabel('Time [s]');
    
    
% set(gcf, 'Position', [0,-10,750,1500]);
suptitle({'Hip and knee trajectories for g_s = 0.1 and g_s = 0.3m/s.'});
savefig('trajectories.fig');