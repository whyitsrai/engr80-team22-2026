%% Non-linear data fitting. This script demonstrates several techniques
%  for fitting data sets with curves other than lines, calculating the
%  confidence intervals on the fitted parameters, and calculating and
%  displaying the functional confidence bounds and the observational
%  bounds. The data set is take from the data sheet for the TCS10K5 NTC
%  thermistor, which lists the resistance as a function of temperature
%  in degrees C. Since we are fitting the Steinhart-Hart equation to the
%  data, we first converted the temperature to Kelvin. Next, because the
%  Steinhart-Hart equation has T as a function of R, we use T or 1/T as y
%  and R or ln(R) as x. Finally, because the data are so clean, we add
%  noise of ±5% to the T data for some of the fits.

% The temperatures.
T = [228.15, 229.15, 230.15, 231.15, 232.15, 233.15, 234.15, 235.15, ...
    236.15, 237.15, 238.15, 239.15, 240.15, 241.15, 242.15, 243.15, ...
    244.15, 245.15, 246.15, 247.15, 248.15, 249.15, 250.15, 251.15, ...
    252.15, 253.15, 254.15, 255.15, 256.15, 257.15, 258.15, 259.15, ...
    260.15, 261.15, 262.15, 263.15, 264.15, 265.15, 266.15, 267.15, ...
    268.15, 269.15, 270.15, 271.15, 272.15, 273.15, 274.15, 275.15, ...
    276.15, 277.15, 278.15, 279.15, 280.15, 281.15, 282.15, 283.15, ...
    284.15, 285.15, 286.15, 287.15, 288.15, 289.15, 290.15, 291.15, ...
    292.15, 293.15, 294.15, 295.15, 296.15, 297.15, 298.15, 299.15, ...
    300.15, 301.15, 302.15, 303.15, 304.15, 305.15, 306.15, 307.15, ...
    308.15, 309.15, 310.15, 311.15, 312.15, 313.15, 314.15, 315.15, ...
    316.15, 317.15, 318.15, 319.15, 320.15, 321.15, 322.15, 323.15];

% The resistances.
R = [473200, 441800, 412600, 385600, 360500, 337200, 315500, 295400, ...
    276700, 259300, 243100, 228000, 213900, 200800, 188600, 177200, ...
    166500, 156600, 147300, 138600, 130500, 122900, 115800, 109200, ...
    103000, 97130, 91660, 86540, 81730, 77220, 72980, 69000, 65260, ...
    61750, 58450, 55340, 52420, 49670, 47080, 44640, 42340, 40170, ...
    38120, 36200, 34380, 32660, 31040, 29510, 28060, 26690, 25400, ...
    24180, 23020, 21920, 20890, 19900, 18970, 18090, 17260, 16470, ...
    15710, 15000, 14320, 13680, 13070, 12490, 11940, 11420, 10920, ...
    10450, 10000, 9572, 9165, 8777, 8408, 8056, 7721, 7402, 7098, ...
    6808, 6531, 6267, 6015, 5774, 5545, 5326, 5116, 4916, 4725, 4543, ...
    4368, 4201, 4041, 3888, 3742, 3602];

%The noisy temperatures.
TN = [237.89, 222.29, 220.44, 229.68, 223.80, 240.13, 224.09, 236.63, ...
    239.87, 236.67, 241.33, 230.53, 244.87, 243.76, 253.48, 248.04, ...
    245.11, 241.58, 250.99, 251.27, 254.79, 247.07, 244.51, 240.73, ...
    259.03, 247.82, 253.96, 246.75, 249.61, 260.59, 245.99, 270.61, ...
    262.16, 251.45, 256.72, 250.46, 259.38, 277.45, 267.59, 270.90, ...
    258.47, 257.87, 268.08, 267.12, 266.16, 283.41, 268.84, 273.66, ...
    263.59, 274.50, 291.04, 281.83, 267.99, 277.94, 278.47, 275.93, ...
    297.04, 273.29, 296.58, 283.65, 275.19, 284.87, 290.91, 302.05, ...
    283.67, 290.39, 299.32, 308.30, 300.72, 295.69, 298.54, 289.95, ...
    289.15, 313.87, 307.66, 316.51, 304.86, 308.64, 292.66, 293.79, ...
    312.59, 299.79, 322.10, 298.11, 317.17, 306.48, 326.64, 303.52, ...
    320.89, 308.95, 325.71, 315.08, 317.71, 317.80, 333.34, 337.38];

confLev = 0.95; % We set the confidence level for the data fits here.

%% Plot Original Data
% Plot the original temperature and noisy temperature data.
figure(1) 
plot(R,T)
hold on
plot (R,TN,'x')
xlabel('Resistance (\Omega)')
ylabel('Temperature (K)')
title('Original and Noisy Data')
legend('Original Temperatures','Noisy Temperatures')
hold off

% Since a plot of 1/T vs ln(R) should be close to linear, we will convert
% the data to the correct forms and do linear and polynomial fits with
% them.
ooT = 1./T;
ooTN = 1./TN;
lnR = log(R);

%% Plot Transformed Data
% Plot the transformed original temperature and noisy temperature data.
figure(2) 
plot(lnR,ooT)
hold on
plot (lnR,ooTN,'x')
xlabel('ln[Resistance (\Omega)]')
ylabel('Reciprocal Temperature (1/K)')
title('Original and Noisy Data Transformed')
legend('Original Temperatures','Noisy Temperatures')
hold off

% Start by working with the original transformed data. We will fit a 1st,
% 2nd, and 3rd order polynomial to the data and look at the fit and the
% residuals.
range = max(lnR) - min(lnR); % Get range for our xplot data
xplot = min(lnR):range/30:max(lnR); % Generate x data for some of our plots.
% The fitting routine 'fit' is particular about the form of the data.
% Use the line below to get your data into the correct format for 'fit'.
[Xout,Yout] = prepareCurveData(lnR, ooT); 
[f1,stat1] = fit(Xout,Yout,'poly1') % 1st-order fit with statistics.
[f2,stat2] = fit(Xout,Yout,'poly2') % 2nd-order fit with statistics.
[f3,stat3] = fit(Xout,Yout,'poly3') % 3rd-order fit with statistics.
% The default for confidence level is 0.95. If we wanted the confidence
% intervals on the parameters for a value of confLev other than 0.95, we
% would uncomment the following lines:
% format short e
% ci1 = confint(f1,confLev)
% ci2 = confint(f2,confLev)
% ci3 = confint(f3,confLev)
% format

%% Plot 1st-order Fit
% Plot data with 1st-order fit. Since the data have very little noise (just
% round-off or truncation error), the data and the curve will vary but
% little.
figure(3)
plot(f1,Xout,Yout) % Notice that the fit doesn't need both x and y values.
% It will plot with just the f1 object.
xlabel('ln[Resistance (\Omega)]')
ylabel('Reciprocal Temperature (1/K)')
title('1st-Order Fit to Transformed Data')

%% Plot Residuals
% Since the fit is so close, we need a better way to distinguish how well
% 1st-, 2nd-, and 3rd-order fit. We'll plot the residuals to compare.
figure(4)
subplot(3,1,1)
plot(f1,Xout,Yout,'residuals')
% xlabel('Ln[Resistance (\Omega)]')
ylabel('Residuals (1/K)')
title('1st Order Polynomial Fit')
% The linear residuals look parabolic, so we need at least 2nd order.
subplot(3,1,2)
plot(f2,Xout,Yout,'residuals')
% xlabel('Ln[Resistance (\Omega)]')
ylabel('Residuals (1/K)')
title('2nd Order Polynomial Fit')
% The 2nd-order residuals look cubic so we need at least 3rd order.
subplot(3,1,3)
plot(f3,Xout,Yout,'residuals')
xlabel('Ln[Resistance (\Omega)]')
ylabel('Residuals (1/K)')
title('3rd Order Polynomial Fit')
% The 3rd-order residuals look pretty random, so we're done.

%% Plot 3rd-order Fit and Bounds
% Let's plot the 3rd order fit with the data and the functional and
% observational bounds. Note, since there is so little noise in the data,
% we won't be able to distinguish which is which.
p11 = predint(f3,xplot,confLev,'observation','off'); % Gen conf bounds
p21 = predint(f3,xplot,confLev,'functional','off'); % Gen conf bounds
figure(5)
plot(f3,Xout,Yout) % Notice that the fit doesn't need both x and y.
hold on
plot(xplot, p21, '-.b') % Upper and lower functional confidence limits
plot(xplot, p11, '--m') % Upper and lower observational confidence limits
xlabel('Ln[Resistance (\Omega)]')
ylabel('Reciprocal Temperature (1/K)')
title('3rd Order Polynomial Fit')
legend('Data Points','Best Fit Line','Upper Func. Bound',...
    'Lower Func. Bound', 'Upper Obs. Bound', 'Lower Obs. Bound',...
    'Location', 'northwest')
hold off

%% 3rd-order with Noise
% Since the original data have so little noise, let's repeat the 3rd order
% plot for the noisy data. We'll reuse some of the variables
[Xout,Yout] = prepareCurveData(lnR, ooTN); 
[f3,stat1] = fit(Xout,Yout,'poly3') % 1st-order fit with statistics.
p11 = predint(f3,xplot,confLev,'observation','off'); % Gen conf bounds
p21 = predint(f3,xplot,confLev,'functional','off'); % Gen conf bounds
figure(6)
plot(f3,Xout,Yout) % Notice that the fit doesn't need both x and y.
hold on
plot(xplot, p21, '-.b') % Upper and lower functional confidence limits
plot(xplot, p11, '--m') % Upper and lower observational confidence limits
xlabel('Ln[Resistance (\Omega)]')
ylabel('Reciprocal Temperature (1/K)')
title('3rd Order Polynomial Fit, Noisy Data')
legend('Data Points','Best Fit Line','Upper Func. Bound',...
    'Lower Func. Bound', 'Upper Obs. Bound', 'Lower Obs. Bound',...
    'Location', 'northwest')
hold off

%% Transform Fit Back
% Let's untransform the best fit and confidence bounds back into the
% original space with T vs R and see how good we feel about things.
yplot = f3(xplot);
figure(7)
plot(R,TN,'x')
hold on
plot(exp(xplot), 1./yplot)
plot(exp(xplot), 1./p21, '-.b') % Upper and lower functional confidence limits
plot(exp(xplot), 1./p11, '--m') % Upper and lower observational confidence limits
legend('Data Points','Best Fit Line','Upper Func. Bound',...
    'Lower Func. Bound', 'Upper Obs. Bound', 'Lower Obs. Bound',...
    'Location', 'northeast')
xlabel('Resistance (\Omega)')
ylabel('Temperature (K)')
title('Retransformed 3rd-Order with Fit Lines')
hold off

%% Nonlinear Fit
% To compare this transformed linear fit of a polynomial with non-linear, let's
% do a non-linear fit using the Steinhart-Hart equation (but we'll include
% the 2nd-order term.
range = max(R) - min(R); % Get range for our xplot data
xplot = min(R):range/30:max(R); % Generate x data for some of our plots.
% First we have to define the function we will fit.
% Things work better if we have starting points for a, b, c, and d. We'll
% use our values from above and 'fitoptions'
fo = fitoptions('Method','NonlinearLeastSquares',...
    'StartPoint',[-0.002894 0.001339 -9.963e-05 3.053e-06]);
ft = fittype('1/(a+b*log(R)+c*(log(R)^2)+d*(log(R)^3))','independent',...
    'R','options',fo);
% Next, we have to get our data into the correct format for 'fit'.
[Xout,Yout] = prepareCurveData(R, TN);
% Now we'll do our fit.
[f4,stat4] = fit(Xout,Yout,ft)
p11 = predint(f4,xplot,confLev,'observation','off'); % Gen conf bounds
p21 = predint(f4,xplot,confLev,'functional','off'); % Gen conf bounds
figure(8)
plot(f4,Xout,Yout) % Notice that the fit doesn't need both x and y.
hold on
plot(xplot, p21, '-.b') % Upper and lower functional confidence limits
plot(xplot, p11, '--m') % Upper and lower observational confidence limits
xlabel('Resistance (\Omega)')
ylabel('Temperature (K)')
title('Steinhart-Hart Fit, Noisy Data')
legend('Data Points','Best Fit Line','Upper Func. Bound',...
    'Lower Func. Bound', 'Upper Obs. Bound', 'Lower Obs. Bound',...
    'Location', 'northeast')
hold off
% In many cases, the non-linear least squares fit looks better than the
% transformed-fit-untransformed fit. This is a rare case when it doesn't

%% Nonlinear Residuals
% And finally, the residuals
figure(9)
plot(f4,Xout,Yout,'residuals')
xlabel('Resistance (\Omega)')
ylabel('Residuals (K)')
title('Steinhart-Hart Fit')