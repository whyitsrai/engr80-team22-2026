% Depth vs Time/Depth Curve [Want pressure or depth]???

b = -0.00949    ;% Assign slope from calibration
y0 = 32.18
0  ;% Assign intercept from calibration
y = b.*depth + y0; % Need to convert from teensy unit to pressure
x = linspace(0, length(y)*0.99,length(y)); % can change to depth later
plot(x,y);
title("Depth vs. Time Graph");
ylabel('Depth [m]');
xlabel('Time (s)');


