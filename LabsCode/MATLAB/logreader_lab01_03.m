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

%% Process your data here!!!
% make note of which experiences gravity
%
accelX3 = accelX;

%
accelY3 = accelY;

%
accelZ3 = accelZ;
%% T-test Instructions (X&Z)

disp("Pairded Dependent T-test (X&Z)")
differenceXZ = accelZ - accelX;
% find the mean of each
dbar_XZ = mean(differenceXZ); 
disp("Mean of XZ difference: " + dbar_XZ)
% calculate Estimated Standard Error
xz_SE = std(differenceXZ)/sqrt(length(differenceXZ));
disp("Standard Deviation is: " + std(differenceXZ))
disp("Estimated Standard Error is: " + xz_SE)
% find the t-value and compare it to the critical value from the table
xz_t = dbar_XZ/xz_SE;
% critical value from the table
xz_tinv = tinv(0.05, (length(differenceXZ) - 1));
disp("xz theoretical t-value: " + xz_tinv)
if xy_t > tinv(0.05, (length(differenceXZ) - 1))
    disp("x and z difference is statistically significant: " + xz_t)
else
    disp("x and z difference is not statistically signifiacnt: " + xz_t)
end
fprintf('\n');