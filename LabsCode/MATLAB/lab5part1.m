%% Lab 5 part a
clf;
figure(7);
% Load in our csv
data = readmatrix('scope_56.csv'); 

N = length(data(:,1));

time = rmmissing(data(:, 1))';
voltage = rmmissing(data(:, 2))';
%disp(voltage);

% Set our sampling frequency and N
fs = 1 / (time(2)-time(1));
N = length(voltage);

han = hann(N);
X = fft((voltage.*han'));

%disp(abs(X(1:N/2)));
fnew = (0 : N/2-1) .* (fs/N);


plot(fnew, abs(X(1:N/2)));

ylabel('Magnitude')
xlabel('Frequency (Hz) ')
title('Matlab plot of FFT magnitude for signal 3 with Hanning window');
xlim([0,100000]);
