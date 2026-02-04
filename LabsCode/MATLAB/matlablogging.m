% matlablogging
% reads from Teensy data stream

function teensyanalog=matlablogging(length)
    length = 5000;  % 5000 is hardcoded buffer size on Teensy
    s = serial('/dev/tty.usbmodem175850901','BaudRate',115200); % TODO edit serial port once teensy is plugged in
    set(s,'InputBufferSize',2*length)
    fopen(s);
    fprintf(s,'%d',2*length)         % Send length to Teensy
    dat = fread(s,2*length,'uint8');
    fclose(s);
    teensyanalog = uint8(dat);
    teensyanalog = typecast(teensyanalog,'uint16');
    save('10mand20m.mat') % added this for redundancy backup
end


%str = fscanf(s);
%teensyanalog = str2num(str);

%[teensyanalog, count] = fscanf(s,['%d']);
