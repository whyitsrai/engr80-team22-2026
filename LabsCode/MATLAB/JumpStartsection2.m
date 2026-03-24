% Code Navigation Jump Start


% Define our variable
% ax = accelX;
% ay = accelY;

% We will define sampling rate from the Loop Period
dt = 0.0099; % CONFIRM

% Now we will make a time vector
t = 0:dt:(length(ay)*dt);

% Now we will integrate for velocity and position
vx = cumtrapz(t,ax); % Integrate the true acceleration to get the true velocity
rx = cumtrapz(t,vx); % Integrate the true velocity to get the true position.
vy = cumtrapz(t,ay); % Integrate the true acceleration to get the true velocity
ry = cumtrapz(t,vy); % Integrate the true velocity to get the true position.


% Bounds
sigma = std(ry); % The standard deviation of the noise in the accel.
confLev = 0.95; % The confidence level for bounds
preie = sqrt(2)*erfinv(confLev)*sigma*sqrt(dt); % the prefix to the sqrt(t)
preiie = 2/3*preie; % The prefix to t^3/2a = 1 + sin( pi*t - pi/2);
plusie=preie*t.^0.5; % The positive noise bound for one integration
plusiie = preiie*t.^1.5; % The positive noise bound for double integration
rnp = ry + plusiie; % Position plus confidence bound
rnm = ry - plusiie; % Position minus confidence bound

% Ideal
ideal = zeros(length(ax));

% Plot x and y
plot(rx, ry);
hold on;
plot([0,0.5,0], [0,0,0])
legend('Calculated Path','Ideal Path','location','southeast')
xlabel("x (m)")
ylabel("y (m)")
title("X, Y Coordinates of Board Stack Overlaid on Ideal 0.5 m Path")

% Now we can plot our y vs time
plot(t, ideal, t, ry, t, rnp,'-.', t, rnm,'-.')
xlabel("time (s)")
ylabel("posY (m)")
title("Y Coordinate of Board Stack vs. Time")
legend('True Path','Calculated Path','Upper Confidence Bound',...
    'Lower Confidence Bound','location','southeast')

