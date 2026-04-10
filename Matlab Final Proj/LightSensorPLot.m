% Light sensor overlay vs Time/Depth Curve

% We will first define the values from the teensy to our varaibles (****
% CURRENT TEST DUMMY VALUES)
violet = [1,2,3,4,5];
blue = [1,2,2,2,2];
green = [1,3,3,3,3];
yellow = [1,4,4,4,4];
orange = [1,5,6,7,6];;
red = [5,5,5,5,5];
x = [1,2,3,4,5]; %linspace(0, length(y)*0.99,length(y)); % can change to depth later, currently time

% Now we will plot the values using hold on to overlay
plot(x, violet, 'Color', [0.56 0.00 1.00]);
hold on;
plot(x, blue,   'Color', [0.00 0.45 0.74]);
plot(x, green,  'Color', [0.00 0.60 0.00]);
plot(x, yellow, 'Color', [1.00 0.84 0.00]);
plot(x, orange, 'Color', [1.00 0.65 0.00]);
plot(x, red,    'Color', [0.85 0.10 0.10]);
title("Light (V) vs. Time (s) Temperature Calibration Curve");
xlabel('Time (s)');
ylabel('Light (V)');
legend('violet', 'blue', 'green', 'yellow', 'orange', 'red');
