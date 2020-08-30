% Patrick Corley - 2020
% Analysis script for accelerometer data captured from rowing sessions
pkg load signal
close all;

aX = importdata('x_accel_small.txt');
aX_negated = aX .* -1;
aY = importdata('y_accel.txt');
aY_negated = aY .* -1;
aZ = importdata('z_accel.txt');
aZ_negated = aZ .* -1;
time_ms = importdata('time_ms_small.txt');

##all_axes_sum =  aX + aY + aZ;
arr_size = length(aX);

aX_sum = 0;
aY_sum = 0;
aZ_sum = 0;
for i = 1:arr_size
  aX_sum = aX_sum + aX(i);
  aY_sum = aY_sum + aY(i);
  aZ_sum = aZ_sum + aZ(i);
end
aX_avg = aX_sum / arr_size;
aX_avg_arr = aX_avg.* ones(arr_size,1);
aY_avg = aY_sum / arr_size;
aY_avg_arr = aY_avg.* ones(arr_size,1);
aZ_avg = aZ_sum / arr_size;
aZ_avg_arr = aZ_avg.* ones(arr_size,1);
 
x_points = linspace(0, arr_size - 1, arr_size);

% Compute and plot FFt of acceleration data
fs = 8; % sampling frequency
% Number of frequency pins must be the next power of two after the number of 
% samples we have 41633 -> 65536
fft_size=2^nextpow2(arr_size);

% Generate frequency domain data (double-sided)
X_double = fft(aX, fft_size);
figure();
subplot(3,2,1);
plot(abs(X_double));
title('X-Axis Two-sided Amplitude Spectrum');
% Generate singler-sided frequency data 
X_single = X_double(1:fft_size/2) * 2;
% Convert frequency bins to actual frequency values (Hz)       
f = fs*(0:fft_size/2 - 1)/fft_size;
subplot(3,2,2);
plot(f, abs(X_single));
title('X-Axis One-sided Amplitude Spectrum');

##% Generate frequency domain data (double-sided)
##Y_double = fft(aY, fft_size);
##subplot(3,2,3);
##plot(abs(Y_double));
##title('Y-Axis Two-sided Amplitude Spectrum');
##% Generate singler-sided frequency data 
##Y_single = Y_double(1:fft_size/2) * 2;
##% Convert frequency bins to actual frequency values (Hz)       
##f = fs*(0:fft_size/2 - 1)/fft_size;
##subplot(3,2,4);
##plot(f, abs(Y_single));
##title('Y-Axis One-sided Amplitude Spectrum');
##
##% Generate frequency domain data (double-sided)
##Z_double = fft(aZ, fft_size);
##subplot(3,2,5);
##plot(abs(Z_double));
##title('Z-Axis Two-sided Amplitude Spectrum');
##% Generate singler-sided frequency data 
##Z_single = Z_double(1:fft_size/2) * 2;
##% Convert frequency bins to actual frequency values (Hz)       
##f = fs*(0:fft_size/2 - 1)/fft_size;
##subplot(3,2,6);
##plot(f, abs(Z_single));
##title('Z-Axis One-sided Amplitude Spectrum');

% Low pass filter the data
fpass = 0.75;
n = 6;
[b,a] = butter(n,fpass/(fs/2));
aX_lpf = filter(b,a,aX);

figure();
X_double_lpf = fft(aX_lpf, fft_size);
% Generate singler-sided frequency data 
X_single_lpf = X_double_lpf(1:fft_size/2) * 2;
% Convert frequency bins to actual frequency values (Hz)       
f = fs*(0:fft_size/2 - 1)/fft_size;
plot(f, abs(X_single_lpf));
title('X-Axis One-sided Amplitude Spectrum (after LPF)');


figure();
plot(time_ms, aX);
title('X-Axis acceleration vs time');
hold;
plot(time_ms, aX_avg_arr);
figure();
plot(time_ms, aX_lpf);
title('X-Axis acceleration (low-pass filterted) vs time');
hold;
plot(time_ms, aX_avg_arr);











