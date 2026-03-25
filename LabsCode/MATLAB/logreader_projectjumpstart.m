% logreader.m
% Use this script to read data from your micro SD card

clear;
clf;

filenum = '000'; % file number for the data you want to read
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

% items included by Logger:
% imu, gps, xy_state_estimator, z_state_estimator, depth_control, motor_driver, adc, ef, button_sampler

% Yields the variables:
% accelX accelY accelZ magX magY magZ roll pitch heading
% lat lon age hdop num_sat
% x y yaw
% z
% depth_des depth depth_error Kp uV diveState surfaceState atDepth atSurface complete totalWayPoints wayPoints
    % this one is strange as there is no associated struct, so the variables themselves need to be changed maybe


%% Header from log.txt file showing available variables
%rollIMU,pitchIMU,headingIMU,accelX,accelY,accelZ,magX,magY,magZ,lat,lon,nsats,x,y,u,uL,uR,yaw,yaw_des,motorA,motorB,motorC,Current_Sense,A00,A01,A02,A03,A10,A11,A12,A13,ErrorFlagA,ErrorFlagB,ErrorFlagC,Button
%float,float,float,float,float,float,float,float,float,float,float,uint8,float,float,float,float,float,float,float,int,int,int,int,int,int,int,int,int,int,int,int,bool,bool,bool,bool


sample_period = 0.0099; % TODO double-check
times = 0:sample_period:size(items{1});

ctrl_eff = uV ./ 255 .* 100; % convert to percentage


%% Obstacle Course Plot

figure(1);
subplot(2,1,1);
plot(times, depth);
hold on;
plot(times, depth_des);
hold off;
xlabel("Time [s]");
ylabel("Depth [m]");
title("Deployment Trajectory");
% Setting domain boundaries to crop in critical data
% xlim([x, y])
% ylim([x y])
legend('Current Depth', 'Desired Depth');

subplot(2,1,2);
plot(times, ctrl_eff);
xlabel("Time [s]");
ylabel("Control Effort [percent]");
title("Deployment Control Effort");
% Setting domain boundaries to crop in critical data
% xlim([x, y])
% ylim([x y])
