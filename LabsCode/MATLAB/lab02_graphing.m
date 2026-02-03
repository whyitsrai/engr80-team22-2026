% logreader_lab01_obstacle.m
% Written by Luke Murphy (hmurphy@hmc.edu) 2026-01-28
% Last edited by Rai Wandeler (rwandeler@hmc.edu) 2026-02-03

clear;

readfile = false; %% TODO update when in lab to be true

if readfile
    filenum = '008'; % file number for the data you want to read
    infofile = strcat('INF', filenum, '.TXT');
    datafile = strcat('LOG', filenum, '.BIN');

    % map from datatype to length in bytes
    dataSizes.('float') = 4;
    dataSizes.('ulong') = 4;
    dataSizes.('int') = 4;
    dataSizes.('int32') = 4;
    dataSizes.('uint8') = 1;
    dataSizes.('uint16') = 2;
    dataSizes.('char') = 1;
    dataSizes.('bool') = 1;

    % read from info file to get log file structure
    fileID = fopen(infofile);
    items = textscan(fileID,'%s','Delimiter',',','EndOfLine','\r\n');
    fclose(fileID);
    [ncols,~] = size(items{1});
    ncols = ncols/2;
    varNames = items{1}(1:ncols)';
    varTypes = items{1}(ncols+1:end)';
    varLengths = zeros(size(varTypes));
    colLength = 256;
    for i = 1:numel(varTypes)
        varLengths(i) = dataSizes.(varTypes{i});
    end
    R = cell(1,numel(varNames));

    % read column-by-column from datafile
    fid = fopen(datafile,'rb');
    for i=1:numel(varTypes)
        %# seek to the first field of the first record
        fseek(fid, sum(varLengths(1:i-1)), 'bof');

        %# % read column with specified format, skipping required number of bytes
        R{i} = fread(fid, Inf, ['*' varTypes{i}], colLength-varLengths(i));
        eval(strcat(varNames{i},'=','R{',num2str(i),'};'));
    end
    fclose(fid);
else
    % For testing with sample data only
    xtesting = 0:1000;
    noise = 0.5-rand(1, length(xtesting));
    ytesting = 0.5 * sin(xtesting/25) + 0.4;
    ytesting = (ytesting*(2^10))/3.3; % testing teensy generators
    ytesting = ytesting+10*noise; % initial noise
    ytesting(ytesting<0) = 0;
    ytesting = ytesting+noise; % add extra noise arund 0v
end

%% Data Processing


%%  Plot

% Plot of Raw measurements
f = figure;
hold on
xlabel("Teensy Sample Number")
ylabel("Teensy ADC measurement")

plot(xtesting, ytesting, '.', 'MarkerSize',1)
yu = max(ytesting);
yl = min(ytesting);
yr = abs((yu-yl));
ym = mean(ytesting);
[pks, locs] = findpeaks(ytesting,xtesting,'MinPeakHeight',yu*0.95,'MinPeakProminence',yr/5); % find sin wave peaks of measured function
period_samples = diff(locs);
predicted_period = mean(period_samples);
model = fittype('a*sin(b*x+c)+d',dependent="y",independent="x",coefficients=["a" "b" "c" "d"]);
excludedPoints = find(ytesting <= yl+(0.01*yr)); % exclude all points within 1% of zero from fit
fittedmodel = fit(xtesting',ytesting',model,'start',[yr/2,(2*pi)/predicted_period,rand(),ym],'Exclude', excludedPoints);
plot(fittedmodel)
model_coeffs = coeffvalues(fittedmodel);
legend("Teensy ADC Measurement", "Predicted Voltage Signal")
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

time = (xtesting/predicted_period)*(1/signal_frequency);
voltage = (ytesting/(2*model_coeffs(1)))*signal_amplitude;

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
