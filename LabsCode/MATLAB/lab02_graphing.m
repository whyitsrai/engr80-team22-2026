% logreader_lab01_obstacle.m
% Written by Luke Murphy (hmurphy@hmc.edu) 2026-01-28
% Last edited by Rai Wandeler (rwandeler@hmc.edu) 2026-02-03

clear;

readfile = true; %% TODO update when in lab to be true

if readfile
    load("10and20.mat"); % variable is teensyanalog
    arrsize = size(teensyanalog);
    disp(arrsize(1));
    samples = 1:arrsize(1);
    teensyanalog = double(teensyanalog);
    teensyanalog = teensyanalog';
else
    % For testing with sample data only
    xtesting = 0:1000;
    noise = 0.5-rand(1, length(xtesting));
    ytesting = 0.5 * sin(xtesting/25) + 0.4;
    ytesting = (ytesting*(2^10))/3.3; % testing teensy generators
    ytesting = ytesting+10*noise; % initial noise
    ytesting(ytesting<0) = 0;
    ytesting = ytesting+noise; % add extra noise arund 0v

    teensyanalog = ytesting;
    samples = xtesting;
end

%% Data Processing


%%  Plot

% Plot of Raw measurements
f = figure;
hold on

plot(samples, teensyanalog, '.', 'MarkerSize',1)

yu = max(teensyanalog);
yl = min(teensyanalog);
yr = abs((yu-yl));
ym = mean(teensyanalog);
[pks, locs] = findpeaks(teensyanalog,samples,'MinPeakHeight',yu*0.95,'MinPeakProminence',yr/5); % find sin wave peaks of measured function
period_samples = diff(locs);
predicted_period = mean(period_samples);
model = fittype('a*sin(b*x+c)+d',dependent="y",independent="x",coefficients=["a" "b" "c" "d"]);
excludedPoints = find(teensyanalog <= yl+(0.01*yr)); % exclude all points within 1% of zero from fit
fittedmodel = fit(samples',teensyanalog',model,'start',[yr/2,(2*pi)/predicted_period,rand()*100,ym],'Exclude', excludedPoints);
plot(fittedmodel)
model_coeffs = coeffvalues(fittedmodel);
legend("Teensy ADC Measurement", "Predicted Voltage Signal")
xlabel("Teensy Sample Number")
ylabel("Teensy ADC measurement")
hold off

%% Plot of measurements with voltage frequency conversion
%% TODO change below variables
signal_frequency = 100; % in Hz
signal_amplitude = 1; % Vp2p
sample_rate = predicted_period*signal_frequency; % sample rate in Hz

f = figure;
hold on
xlabel("Time [s]")
ylabel("Voltage [V]")

time = (samples/predicted_period)*(1/signal_frequency);
voltage = (teensyanalog/(2*model_coeffs(1)))*signal_amplitude;

plot(time, voltage, '.', 'MarkerSize',1)

% a sin(bx + c) + d
sin_a = model_coeffs(1)/(2*model_coeffs(1))*signal_amplitude;
sin_b = 2*pi*signal_frequency;
sin_c = (model_coeffs(2)/predicted_period)*(1/signal_frequency);
sin_d = (model_coeffs(4)/(2*model_coeffs(1)))*signal_amplitude;
y=sin_a*sin(sin_b*time+sin_c) + sin_d;
plot(time, y);
legend("Teensy ADC Measurement", "Predicted Voltage Signal")
hold off

disp("Measured Values:");
disp("Sample rate: " + sample_rate + " samples/sec")
disp("P2P voltage: " + 2*sin_a)
disp("RMS voltage: " + sin_a / sqrt(2))
