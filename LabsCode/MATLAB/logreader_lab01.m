% logreader.m
% Use this script to read data from your micro SD card

clear;
%clf;

filenum = '004'; % file number for the data you want to read
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

% Process your data here!!!
%% Acceleration variables initialization
accelZ_grav = accelZ;

%% Acceleration Conversion (m/s^2)
zbar = mean(accelZ_grav);
accelConversion = 9.8/zbar;
%%
accelX = accelX .* accelConversion;
accelY = accelY .* accelConversion;
% accelZ below is the accleration data for the z-direction for experimental
% data
accelZ = accelZ .* accelConversion;

disp("Conversion ratio: " + accelConversion)

%% T-test Instructions
% x-y, y-z, x-z
differenceXY = accelY - accelX;
differenceYZ = accelZ - accelY;
differenceXZ = accelZ - accelX;
%%
% find the mean of each
dbar_XY = mean(differenceXY);
dbar_YZ = mean(differenceYZ);
dbar_XZ = mean(differenceXZ);
%%
% calculate Standard error
xy_SE = std(differenceXY)/sqrt(length(differenceXY));
yz_SE = std(differenceYZ)/sqrt(length(differenceYZ));
xz_SE = std(differenceXZ)/sqrt(length(differenceXZ));
%%
% find the t-value and compare it to the critical value from the table
xy_t = dbar_XY/xy_SE;
yz_t = dbar_YZ/yz_SE;
xz_t = dbar_XZ/xz_SE;
xy_tinv = tinv(0.05, (length(differenceXY) - 1));
yz_tinv = tinv(0.05, (length(differenceYZ) - 1));
xz_tinv = tinv(0.05, (length(differenceXZ) - 1));
disp("xy theoretical t-value: " + xy_tinv)
disp("yz theoretical t-value: " + yz_tinv)
disp("xz theoretical t-value: " + xz_tinv)
%%
if xy_t > tinv(0.05, (length(differenceXY) - 1))
    disp("x and y difference is statistically significant: " + xy_t)
else
    disp("x and y difference is not statistically signifiacnt: " + xy_t)
end
if yz_t > tinv(0.05, (length(differenceYZ) - 1))
    disp("y and z difference is statistically significant: " + yz_t)
else
    disp("y and z difference is not statistically signifiacnt: " + yz_t)
end
if xy_t > tinv(0.05, (length(differenceXZ) - 1))
    disp("x and z difference is statistically significant: " + xz_t)
else
    disp("x and z difference is not statistically signifiacnt: " + xz_t)
end

%% Z-direction Acceleration Data Calculations
data = accelZ;
confLev = 0.95;
xbar = mean(data); % Arithmetic mean
S = std(data); % Standard Deviation
N = length(data); % Count
ESE = S/sqrt(N); % Estimated Standard Error

% tinv is for 1-tailed, for 2-tailed we need to halve the range
StdT = tinv((1-confLev),N-1); % The Student t value
confBounds = StdT*(S/sqrt(N));

disp("Z-direction mean is: " + xbar);
disp("Standard Deviation is: " + S);
disp("Sample Size is: " + N)
disp("Standard Error is: " + ESE);
disp("Confidence Bounds are: " + abs(confBounds));


%% Plotting Zero Acceleration Graphs
subplot(3, 1, 1)
plot(accelX)
ylabel("Acceleration (m/s^2)")
title("X-direction")

subplot(3, 1, 2)
plot(accelY)
ylabel("Acceleration (m/s^2)")
title("Y-direction")

subplot(3, 1, 3)
plot(accelZ)
xlabel("Sample #")
ylabel("Acceleration (m/s^2)")
title("Z-direction")
%% Plotting Accleration due to Gravity (Z-direction)
figure;
hold on
plot(accelZ)
xlabel("Sample #")
ylabel("Acceleration (m/s^2)")
title("Z-direciton (due to gravity)")

%disp(mean(accelY))
hold off

%% Obstacle Course Plot

figure;
hold on
subplot(1, 1, 1)
plot(accelX)
plot(accelY)
plot(accelZ)
%labelling
xlabel("Sample Number")
ylabel("Acceleration (m/s^2)")
title("Obstacle Course Acceleration Plot")
% Setting domain boundaries to crop critical data
% xlim([x, y])
% ylim([x y])
legend('accelX', 'accelY', 'accelZ')

