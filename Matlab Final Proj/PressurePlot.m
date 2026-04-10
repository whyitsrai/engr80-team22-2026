% Depth vs Time/Depth Curve [Want pressure or depth]???

b = -0.0187    ;% Assign slope from calibration
y0 =  16.3400  ;% Assign intercept from calibration
y = b.*A10 + y0; % Need to convert from teensy unit to pressure
x = linspace(0, length(y)*0.99,length(y)); % can change to depth later
plot(x,y);
title("Depth vs. Time Graph");
ylabel('Depth [m]');
xlabel('Time (s)');


