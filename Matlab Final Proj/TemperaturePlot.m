% Temperature vs Time/Depth Curve

b = -0.0293 ; % Assign slope from calibration
y0 =  29.8353;% Assign intercept from calibration
y = b.*A01 + y0; % Need to convert from teensy unit to temperature
x = linspace(0, length(y)*0.99,length(y)); % can change to depth later, currently time
plot(x,y);
title("Temperature vs. Time Graph");
ylabel('Temperature (°C)');
xlabel('Time (s)');