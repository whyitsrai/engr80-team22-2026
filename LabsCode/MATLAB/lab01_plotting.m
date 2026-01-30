%% Initialize X, Y, & Z Datasets

accelX_Total = [accelX1; accelX2];
accelY_Total = [accelY1; accelY3];
accelZ_Total = [accelZ2; accelZ3];

%% Statistics for Each Direction

% X-direcion
disp("Stats for X-Direciton");
data = accelX_Total;
confLev = 0.95;
xbar = mean(data); % Arithmetic mean
disp("Sample Mean is: " + xbar);
S = std(data); % Standard Deviation
disp("Standard Deviation is: " + S);
N = length(data); % Count
disp("Number of samples: " + N);
ESE = S/sqrt(N); % Estimated Standard Error
disp("Standard Error: " + ESE);
% tinv is for 1-tailed, for 2-tailed we need to halve the range
StdT = tinv((1-confLev)/2,N-1); % The Student t value
disp("t value is: " + StdT);
confBounds = StdT*(S/sqrt(N));
disp("Confidence Bounds are: " + confBounds);
fprintf('\n');

% Y-direcion
disp("Stats for Y-Direciton");
data = accelY_Total;
confLev = 0.95;
ybar = mean(data); % Arithmetic mean
disp("Sample Mean is: " + ybar);
S = std(data); % Standard Deviation
disp("Standard Deviation is: " + S)
N = length(data); % Count
disp("Number of samples: " + N)
ESE = S/sqrt(N); % Estimated Standard Error
disp("Standard Error: " + ESE)
% tinv is for 1-tailed, for 2-tailed we need to halve the range
StdT = tinv((1-confLev)/2,N-1); % The Student t value
disp("t value is: " + StdT)
confBounds = StdT*(S/sqrt(N));
disp("Confidence Bounds are: " + confBounds)
fprintf('\n');

% Z-direcion
disp("Stats for Z-Direciton")
data = accelZ_Total;
confLev = 0.95;
zbar = mean(data); % Arithmetic mean
disp("Sample Mean is: " + zbar)
S = std(data); % Standard Deviation
disp("Standard Deviation is: " + S)
N = length(data); % Count
disp("Number of samples: " + N)
ESE = S/sqrt(N); % Estimated Standard Error
disp("Standard Error: " + ESE)
% tinv is for 1-tailed, for 2-tailed we need to halve the range
StdT = tinv((1-confLev/2),N-1); % The Student t value
disp("t value is: " + StdT)
confBounds = StdT*(S/sqrt(N));
disp("Confidence Bounds are: " + confBounds)
fprintf('\n');

%% Plotting Zero Acceleration Graphs
figure;
hold on
subplot(3, 1, 1)
plot(accelX_Total)
xlim([0 200])
ylabel("Acceleration (Teensy Units)")
title("X-direction")
yline(xbar, 'y--')


subplot(3, 1, 2)
plot(accelY_Total)
xlim([0 200])
ylabel("Acceleration (Teensy Units)")
title("Y-direction")
yline(ybar, 'y--')

subplot(3, 1, 3)
plot(accelZ_Total)
xlim([0 200])
xlabel("Sample #")
ylabel("Acceleration (Teensy Units)")
title("Z-direction")
yline(zbar, 'y--')

%% Statistics for Accleration due to Gravity (Z-direction)
% Z-direcion
disp("Stats for Z-Direciton due to Gravity")
data = accelZ1;
confLev = 0.95;
z_gravbar = mean(data); % Arithmetic mean
disp("Sample Mean is: " + z_gravbar)
S = std(data); % Standard Deviation
disp("Standard Deviation is: " + S)
N = length(data); % Count
disp("Number of samples: " + N)
ESE = S/sqrt(N); % Estimated Standard Error
disp("Standard Error: " + ESE)
% tinv is for 1-tailed, for 2-tailed we need to halve the range
StdT = tinv((1-confLev)/2,N-1); % The Student t value
disp("t value is: " + StdT)
confBounds = StdT*(S/sqrt(N));
disp("Confidence Bounds are: " + confBounds)
fprintf('\n');

%% Plotting Accleration due to Gravity (Z-direction)
figure;
hold on
plot(accelZ1)
xlim([0 200])
xlabel("Sample #")
ylabel("Acceleration (Teensy units)")
title("Z-direciton (due to gravity)")
yline(z_gravbar, 'y--')
hold off
