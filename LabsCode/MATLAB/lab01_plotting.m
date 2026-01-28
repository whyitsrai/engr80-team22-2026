%% Initialize X, Y, & Z Datasets

%% Acceleration variables initialization
accelZ_grav = accelZ3;

%% Acceleration Conversion (m/s^2)
zbar = mean(accelZ_grav);
accelConversion = 9.8/zbar;
%%
accelX = accelX .* accelConversion;
accelY = accelY .* accelConversion;
% accelZ below is the accleration data for the z-direction for experimental
% data
accelZ = accelZ .* accelConversion;

disp("Conversion ratio: " + accelConversion)


%% Plotting Zero Acceleration Graphs
hold on
subplot(3, 1, 1)
plot(accelX1)
plot(accelX2)
ylabel("Acceleration (m/s^2)")
title("X-direction")

subplot(3, 1, 2)
plot(accelY1)
plot(accelY2)
ylabel("Acceleration (m/s^2)")
title("Y-direction")

subplot(3, 1, 3)
plot(accelZ1)
plot(accelZ2)
xlabel("Sample #")
ylabel("Acceleration (m/s^2)")
title("Z-direction")

%% Plotting Accleration due to Gravity (Z-direction)
figure;
hold on
plot(accelZ3)
xlabel("Sample #")
ylabel("Acceleration (m/s^2)")
title("Z-direciton (due to gravity)")

%disp(mean(accelY))
hold off

%% Obstacle Course Plot

figure;
hold on
subplot(1, 1, 1)
plot(accelX)
plot(accelY)
plot(accelZ)
%labelling
xlabel("Sample Number")
ylabel("Acceleration (m/s^2)")
title("Obstacle Course Acceleration Plot")
% Setting domain boundaries to crop critical data
% xlim([x1 x2])
% ylim([y1 y2])
legend('accelX', 'accelY', 'accelZ', 'Peak Accel Sample #', 'Peak Acceleration')
[Max, Index] = max(accelZ);
disp([Max, Index])
xline(Index, 'y--')
yline(Max, 'y--')

