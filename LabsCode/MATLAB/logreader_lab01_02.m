% logreader.m
% Use this script to read data from your micro SD card

clear;
%clf;

filenum = '005'; % file number for the data you want to read
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

%% Adding trial data to overall datasets
accelX2 = accelX;
accelY2 = accelY;
accelZ2 = accelZ;

differenceYZ = accelZ - accelY;
dbar_YZ = mean(differenceYZ);
yz_SE = std(differenceYZ)/sqrt(length(differenceYZ));
yz_t = dbar_YZ/yz_SE;
yz_tinv = tinv(0.05, (length(differenceYZ) - 1));
disp("yz theoretical t-value: " + yz_tinv)
if yz_t > tinv(0.05, (length(differenceYZ) - 1))
    disp("y and z difference is statistically significant: " + yz_t)
else
    disp("y and z difference is not statistically signifiacnt: " + yz_t)
end