%% Author: Team 22
%% Date: 25th Feb 2026
%% Purpose: Measured Calibration Curve for Wind Tunnel Fan Speed vs. Air Velocity


% We will create x vector as the RPM and y vector as the air velocity
x = [2.22 2.02 1.82 1.61 1.45]; % Wind Tunnel Fan Speed [RPM]
y = [0 0.1 0.2 0.3 0.4]; % Air Velocity [m/s]

%% The follow code was credited to Prof. Spencer code given in the E80
%% Website
confLev = 0.95; % The confidence level
N = length(y); % The number of data points
xbar = mean(x);
ybar = mean(y);
Sxx = dot((x-xbar),(x-xbar));
%Sxx = (x-xbar)*transpose(x-xbar);
% beta1 is the estimated best slope of the best-fit line
beta1 = dot((x-xbar),(y-ybar))/Sxx
% beta1 = ((x-xbar)*transpose(y-ybar))/Sxx
% beta0 is the estimated best-fit y-intercept of the best fit line
beta0 = ybar - beta1*xbar
yfit = beta0 + beta1*x;
SSE = dot((y - yfit),(y - yfit)) % Sum of the squared residuals
% SSE = (y - yfit)*transpose(y - yfit) % Sum of the squared residuals
Se = sqrt(SSE/(N-2)) % The Root Mean Square Residual
Sbeta0 = Se*sqrt(1/N + xbar^2/Sxx)
Sbeta1 = Se/sqrt(Sxx)
% tinv defaults to 1-sided test. We need 2-sises, hence:(1-0.5*(1-confLev))
StdT = tinv((1-0.5*(1-confLev)),N-2) % The Student's t factor
lambdaBeta1 = StdT*Sbeta1 % The 1/2 confidence interval on beta1
lambdaBeta0 = StdT*Sbeta0 % The 1/2 confidence interval on beta0
range = max(x) - min(x);
xplot = min(x):range/30:max(x); % Generate array for plotting
yplot = beta0 + beta1*xplot; % Generate array for plotting
Syhat = Se*sqrt(1/N + (xplot - xbar).*(xplot - xbar)/Sxx);
lambdayhat = StdT*Syhat;
Sy = Se*sqrt(1+1/N + (xplot - xbar).*(xplot - xbar)/Sxx);
lambday = StdT*Sy;
figure(1)
plot(x,y,'x')
hold on
plot(xplot,yplot)
plot(xplot,yplot+lambdayhat,'-.b',xplot,yplot-lambdayhat,'-.b')
plot(xplot,yplot+lambday,'--m',xplot,yplot-lambday,'--m')
xlabel('Wind Tunnel Fan Speed [RPM]')
ylabel('Air Velocity [m/s]')
if beta1 > 0 % Fix this
    location = 'northwest';
else
    location = 'northeast';
end
legend('Data Points','Best Fit Line','Upper Func. Bound',...
    'Lower Func. Bound', 'Upper Obs. Bound', 'Lower Obs. Bound',...
    'Location', location)
title("Measured Calibration Curve for Wind Tunnel Fan Speed vs. Air Velocity")
hold off