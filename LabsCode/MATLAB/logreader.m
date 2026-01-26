% logreader.m
% Use this script to read data from your micro SD card

clear;
%clf;

filenum = '004'; % file number for the data you want to read
infofile = strcat('INF', filenum, '.TXT');
datafile = strcat('LOG', filenum, '.BIN');

%% map from datatype to length in bytes
dataSizes.('float') = 4;
dataSizes.('ulong') = 4;
dataSizes.('int') = 4;
dataSizes.('int32') = 4;
dataSizes.('uint8') = 1;
dataSizes.('uint16') = 2;
dataSizes.('char') = 1;
dataSizes.('bool') = 1;

%% read from info file to get log file structure
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

%% read column-by-column from datafile
fid = fopen(datafile,'rb');
for i=1:numel(varTypes)
    %# seek to the first field of the first record
    fseek(fid, sum(varLengths(1:i-1)), 'bof');
    
    %# % read column with specified format, skipping required number of bytes
    R{i} = fread(fid, Inf, ['*' varTypes{i}], colLength-varLengths(i));
    eval(strcat(varNames{i},'=','R{',num2str(i),'};'));
end
fclose(fid);

%% Process your data here
data = accelZ;
confLev = 0.95;
xbar = mean(data); % Arithmetic mean
S = std(data); % Standard Deviation
N = length(data); % Count
ESE = S/sqrt(N); % Estimated Standard Error
% tinv is for 1-tailed, for 2-tailed we need to halve the range
StdT = tinv((1-0.5*(1-confLev)),N-1); % The Student t value

accelUnit = xbar/9.81;
disp(accelUnit)
%% Plotting 4 Acceleration Graphs
subplot(2, 2, 1)
plot(accelX)
ylabel("Acceleration (m/s^2)")
title("X-direction")
subplot(2, 2, 2)
plot(accelY)
ylabel("Acceleration (m/s^2)")
title("Y-direction")
subplot(2, 2, 3)
plot(accelZ)
ylabel("Acceleration (m/s^2)")
title("Z-direction")
subplot(2, 2, 4)
plot(accelZ)
ylabel("Acceleration (m/s^2)")
title("Z-direciton (due to gravity)")

%% Obstacle Course

hold on
plot(accelX)
plot(accelY)
plot(accelZ)
xlabel("Sample Number")
ylabel("Acceleration")
hold off

