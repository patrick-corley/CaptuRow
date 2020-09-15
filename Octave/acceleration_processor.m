% Patrick Corley 2020 - patrick.urban90@gmail.com
% CaptuRow post-processing script 
% Analysis script for analyzing accelerometer data captured from rowing sessions
% Accelerometer: MMA8452Q

pkg load signal
close all;

aX = importdata('../Reliable datasets/x_accel_SINGLE_3_SPINS_29-8-2020.txt');
aX_negated = aX .* -1;
aY = importdata('../Reliable datasets/y_accel.txt');
aY_negated = aY .* -1;
aZ = importdata('../Reliable datasets/z_accel.txt');
aZ_negated = aZ .* -1;
time_ms = importdata('../Reliable datasets/sample_num_SINGLE_3_SPINS_29-8-2020.txt');

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

##% Set anything below avg equal to avg
##for i = 1:arr_size
##  if (aX(i) < aX_avg)
##    aX(i) = aX_avg;
##  endif
##end

x_points = linspace(0, arr_size - 1, arr_size);

%-------------------------------------------------------------------------------
% Compute and plot FFT of acceleration data
% Setup Variables:
% Accelerometer sampling frequency = 8Hz but Fs = 16Hz seems to produce a spectral
% plot that seems to look like what it expected - need to investiagte this (?)

fs = 2; % sampling frequency
f = fs*(0:(arr_size/2))/arr_size;
% Number of frequency pins must be power of two... 41633 -> 65536
fft_size=2^nextpow2(arr_size);
%-------------------------------------------------------------------------------

figure();
% Generate frequency domain data (double-sided)
aX_fft = fft(aX);
aX_double= abs(aX_fft/arr_size);
% Generate frequency domain data (single-sided)
aX_single = aX_double(1:arr_size/2+1);
aX_single(2:end-1) = 2*aX_single(2:end-1);
plot(f, abs(aX_single));
title('X-Axis One-sided Amplitude Spectrum');
xlabel('f (Hz)');
ylabel('|P1(f)|');

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

% Low pass filter the data. Cutoff = 1Hz, 2nd order filter
fpass = 0.1;
n = 2;
[b,a] = butter(n,fpass/(fs/2),'low');
aX_lpf = filter(b,a,aX);

figure();
aX_lpf_fft = fft(aX_lpf);
aX_double_lpf= abs(aX_lpf_fft/arr_size);
aX_single_lpf = aX_double_lpf(1:arr_size/2+1);
aX_single_lpf(2:end-1) = 2*aX_single_lpf(2:end-1);
plot(f, abs(aX_single_lpf));
title('X-Axis One-sided Amplitude Spectrum (after LPF)');
xlabel('f (Hz)');
ylabel('|P1(f)|');

figure();
plot(time_ms, aX);
title('X-Axis acceleration vs time');
hold;
plot(time_ms, aX_avg_arr);
figure();
plot(time_ms, aX_lpf);
title('X-Axis acceleration (low-pass filtered) vs time');
hold;
plot(time_ms, aX_avg_arr);













