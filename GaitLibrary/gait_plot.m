
h(1) = figure;
ax1 = subplot(4,1,1);
hold on;
plot(controlTime, controlActionQUAD);
plot(controlTime, controlActionHAMS);
plot(controlTime, controlActionTorque);
hold off;
title('Control Action'); xlabel('time [s]'); ylabel('Control Action');
legend('quad','hams','hip');
axis([controlTime(1) controlTime(end) -1 1]);
% axis([controlTime(1) controlTime(end)]);

ax2 = subplot(4,1,2);
hold on;
plot(controlTime, rad2deg(measKnee));
plot(controlTime, rad2deg(refKnee));
hold off;
title('Knee Angle'); xlabel('time [s]'); ylabel('Knee Angle [deg]');
axis([controlTime(2) controlTime(end) -40 10]);
legend('meas','ref');
rad2deg(mean(controlErrorKnee))

ax3 = subplot(4,1,3);
hold on;
plot(controlTime, rad2deg(measHip));
plot(controlTime, rad2deg(refHip));
hold off;
title('Hip Angle'); xlabel('time [s]'); ylabel('Hip Angle [deg]');
axis([controlTime(2) controlTime(end) -100 100]);
rad2deg(mean(controlErrorHip))
legend('meas','ref');

ax4 = subplot(4,1,4);
    hold on;
    plot(controlTime, PIDparam_vector(1,:));
    plot(controlTime, PIDparam_vector(2,:));
    plot(controlTime, PIDparam_vector(3,:));
    hold off;
    legend('p','i','d');


linkaxes([ax1,ax2,ax3,ax4],'x');
