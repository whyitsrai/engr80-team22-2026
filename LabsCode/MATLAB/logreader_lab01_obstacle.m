% logreader.m
% Use this script to read data from your micro SD card

clear;
%clf;

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

%% Process your data here!!!

% Acceleration variables initialization
% accelZ_grav = accelZ1;

%% Acceleration Conversion (m/s^2)
% removing bias from combined datasets
% accelX = accelX - xbar;
% accelY = accelY - ybar;
% accelZ = accelZ - zbar;
% 
% % Conversion Unit
% z = mean(accelZ_grav);
% accelConversion = 9.8/z;
% disp("Conversion ratio: " + accelConversion)
accelConversion = 0.010035;
%% Converting to m/s^2
accelX = accelX .* accelConversion;
accelY = accelY .* accelConversion;
accelZ = accelZ .* accelConversion;
%% Obstacle Course Plot
totAccel = sqrt((accelX).^2 + (accelY).^2 + (accelZ).^2);
figure;
hold on
subplot(1, 1, 1)
plot(accelX)
plot(accelY)
plot(accelZ)
plot(totAccel)
%labelling
xlabel("Sample Number")
ylabel("Acceleration (m/s^2)")
title("Obstacle Course Acceleration Plot")
% Setting domain boundaries to crop critical data
xlim([890 1210])
legend('accelX', 'accelY', 'accelZ', 'Magnitude of Acceleration')
[Max, Index] = max(accelZ(890: 1210));
disp([Max, Index])
xline(1125, 'm--')
yline(11.706, 'm--')
%%
hold on
plot(accelX)
plot(accelY)
plot(accelZ)
legend("accelX", "accelY", "accelZ")
%%

