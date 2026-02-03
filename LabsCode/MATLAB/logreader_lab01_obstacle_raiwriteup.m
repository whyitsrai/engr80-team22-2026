% logreader_lab01_obstacle.m
% Written by Luke Murphy (hmurphy@hmc.edu) 2026-01-28
% Last edited by Rai Wandeler (rwandeler@hmc.edu) 2026-01-30

clear;

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

%% Data Processing

accelConversion = 0.010035; % correction factor from previous script

% Converting to m/s^2
accelX = accelX .* accelConversion;
accelY = accelY .* accelConversion;
accelZ = accelZ .* accelConversion;
totAccel = sqrt(accelX.^2 + accelY.^2 + accelZ.^2);

%%  Plot
f = figure;
subplot(2, 1, 1)
hold on
xlabel("Sample Number (0.1 second increments)")
ylabel("Acceleration (m/s^2)")
% Setting domain boundaries to crop critical data
xlim([890 1210])
ylim([-6 12])
[Max, Index] = max(accelZ(890: 1210));
disp([Max, Index])
plot(accelX)
plot(accelY)
plot(accelZ)
plot(totAccel)
plot(1125,11.706,'o')
legend("accel. x", "accel. y", "accel. z", "mag. tot. accel.", "max. tot. accel.")
xline(1125, 'm:')
yline(11.706, 'm:')
hold off
%
subplot(2, 1, 2)
xlim([890 1210])
% plot motor values
hold on
plot(motorA/255, '--')
plot(motorB/255, ':')
plot(motorC/255)
legend("motor A (x-dir.)", "motor B (x-dir.)", "motor c (z-dir.)")
xlabel("Sample Number (0.1 second increments)")
ylabel("Motor Power")
hold off

exportgraphics(f,'Lab1Plots.svg','Padding', 20)
