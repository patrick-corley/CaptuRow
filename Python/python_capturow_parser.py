import folium
import geopy.distance
import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter,filtfilt,sosfilt
import os
import sys
import time
import serial
from pathlib import Path

Path(__file__).parents[1]

cwd = os.getcwd()
input_file_path = str(Path(__file__).parents[1]) + '\Reliable datasets\GOOD_DATALOG_29-8-2020_MODIFIED.txt'

class datalog:
    def __init__(self):
        self.elap_milli = []
        self.current_sample = 0
        self.latitude = []
        self.longitude = []
        self.num_strokes = 0
        self.aX = []
        self.aX_avg = 0
        self.aY = []
        self.aY_avg = 0
        self.aZ = []
        self.aZ_avg = 0
        self.distance = 0
        self.num_samples = 0


def count_samples():
    with open(input_file_path, "r") as f:
        # Count samples in file
        dlog.num_samples = sum(1 for line in f) 


def plot_coords(sample_interval):
    m = folium.Map(location=[dlog.latitude[0], dlog.longitude[0]], max_zoom=19, zoom_start=19)
    for i in range (dlog.num_samples):
        if i % sample_interval == 0:
            folium.Marker([dlog.latitude[i], dlog.longitude[i]], icon=folium.Icon(color='red', icon='times', prefix='fa')).add_to(m)
    m.save('map.html')

def calc_total_distance():
    tot_distance = 0
    for i in range (dlog.num_samples):
        if i > 0:
            coords_1 = (dlog.latitude[i-1], dlog.longitude[i-1])
            coords_2 = (dlog.latitude[i], dlog.longitude[i])
            meter_distance = geopy.distance.distance(coords_1, coords_2).m
            tot_distance += meter_distance
    print('Total Distance = ' + str(round(tot_distance, 2)) + ' m')

def calc_max_speed(sample_interval):
    max_speed = 0 
    for i in range (dlog.num_samples):
        if ((i > 0) and (i % sample_interval == 0)):
            coords_1 = (dlog.latitude[i - sample_interval], dlog.longitude[i - sample_interval])
            coords_2 = (dlog.latitude[i], dlog.longitude[i])
            distance = geopy.distance.distance(coords_1, coords_2).m
            speed = distance / ((dlog.elap_milli[i] - (dlog.elap_milli[i - sample_interval])) / 1000)
            if (speed > max_speed):
                max_speed = speed
    print('Max Speed = ' + str(round(max_speed, 2)) + ' m/s')
    if max_speed != 0:
        factor = 500 / max_speed
        min_sec = factor / 60
        mins = math.modf(min_sec)[1]
        frac_mins = math.modf(min_sec)[0]
        secs = 60 * frac_mins
        print('Max Speed = ' + str(round(mins,0)) + ' mins ' + str(round(secs, 2)) + ' secs/500m')
    else:
        print('Max Speed could not be determined or distance travelled = 0')

# def calc_avg_speed(sample_interval):
#     avg_speed = 0
#     for i in range (dlog.num_samples):
#         if ((i > 0) and (i % sample_interval == 0)):
#             coords_1 = (dlog.latitude[i-sample_interval], dlog.longitude[i-sample_interval])
#             coords_2 = (dlog.latitude[i], dlog.longitude[i])
#             distance = geopy.distance.distance(coords_1, coords_2).m
#             speed = distance / (sample_interval)
#             if (speed > max_speed):
#                 max_speed = speed

def plot_accel(plot_all_axes_sum = False, round_precision = 2, x_axis_step = 20, plot_avg = True, start_point = 0, end_point = 10000):
    aX_rounded = []
    aY_rounded = []
    aZ_rounded = []
    for i in range(start_point, end_point):
        aX_rounded.append(round(float(dlog.aX[i]), round_precision))
        aY_rounded.append(round(float(dlog.aY[i]), round_precision))
        aZ_rounded.append(round(float(dlog.aZ[i]), round_precision))

    x_points = range(start_point, end_point)
    if  plot_all_axes_sum:
        
        plt.subplot(3,3,1)
        plt.plot(x_points, aX_rounded)
        plt.title('X-Axis Acceleration')

        plt.subplot(3,3,2)
        plt.plot(x_points, aY_rounded)
        plt.title('Y-Axis Acceleration')

        plt.subplot(3,3,3)
        plt.plot(x_points, aZ_rounded)
        plt.title('Z-Axis Acceleration')

        sum_rounded = []
        for i in range(end_point - start_point):
            sum_rounded.append((aX_rounded[i] + aY_rounded[i] + aZ_rounded[i]))
        plt.subplot(3,3,4)
        plt.plot(x_points, sum_rounded)
        plt.title('X-Axis + Y-Axis + Z-Axis Acceleration')

        sum_rounded = remove_offset(sum_rounded)

        plt.subplot(3,3,5)
        sum_rounded_lpf = lpf_data(data=sum_rounded, cutoff=1.67, sample_period=0.122)
        plt.plot(x_points, sum_rounded_lpf)
        plt.title('All Axes Sum Low-Pass Filtered')
        plt.xlabel('Sample')
        plt.ylabel('Acceleration (2G normalized)')
        
        plt.subplot(3,3,6)
        #sum_rounded_bpf = bpf_data(data=sum_rounded, cutoff_low=0.3, cutoff_high=1.67, sample_period=0.122)
        sum_rounded_bpf = hpf_data(data=sum_rounded_lpf, cutoff=0.0167,sample_period=0.122 )
        plt.plot(x_points, sum_rounded_bpf)
        plt.title('All Axes Sum Band-Pass Filtered')
        plt.xlabel('Sample')
        plt.ylabel('Acceleration (2G normalized)')

        [t, dummy_data] = gen_dummy_data(f=0.34, sample_period=0.122, start_time=0, end_time=60)
        plt.subplot(3,3,7)
        plt.plot(t, dummy_data)
        plt.title('Noisy Dummy Data')
        plt.xlabel('Time')
        plt.ylabel('Amplitude')

        #lpf_dummy_data = lpf_data(data=dummy_data, cutoff=1.67, sample_period=0.122)
        lpf_dummy_data = lpf_data(data=dummy_data, cutoff=0.5, sample_period=0.122)
        plt.subplot(3,3,8)
        plt.plot(t, lpf_dummy_data)
        plt.title('LPF Noisy Dummy Data')
        plt.xlabel('Time')
        plt.ylabel('Amplitude')
        plt.show()
        

    else:
        plt.subplot(3,1,1)
        plt.plot(x_points, aX_rounded)
        if plot_avg:
            plt.plot(x_points, np.full(end_point - start_point, dlog.aX_avg))
        plt.title('X-Axis Acceleration')
        plt.xlabel('Sample')
        plt.ylabel('Acceleration (2G normalized)')
        plt.xticks(np.arange(start_point, end_point, step=x_axis_step))
        plt.subplot(3,1,2)
        plt.plot(x_points, aY_rounded)
        if plot_avg:
            plt.plot(x_points, np.full(end_point - start_point, dlog.aY_avg))
        plt.title('Y-Axis Acceleration')
        plt.xlabel('Sample')
        plt.ylabel('Acceleration (2G normalized)')
        plt.xticks(np.arange(start_point, end_point, step=x_axis_step))
        plt.subplot(3,1,3)
        plt.plot(x_points, aZ_rounded)
        if plot_avg:
            plt.plot(x_points, np.full(end_point - start_point, dlog.aZ_avg))
        plt.title('Z-Axis Acceleration')
        plt.xlabel('Sample')
        plt.ylabel('Acceleration (2G normalized)')
        plt.xticks(np.arange(start_point, end_point, step=x_axis_step))
        plt.show()

def avg_accel_data():
    x_sum = 0
    y_sum = 0
    z_sum = 0
    for i in range(0, dlog.num_samples):
        x_sum += float(dlog.aX[i])
        y_sum += float(dlog.aY[i])
        z_sum += float(dlog.aZ[i])
    dlog.aX_avg = x_sum / dlog.num_samples
    dlog.aY_avg = y_sum / dlog.num_samples
    dlog.aZ_avg = z_sum / dlog.num_samples

def find_g_axis():
    # Determine which axis/axes gravity is more dominant on
    print('')

def remove_offset(data):
    # Calibrate-out offset in accel data
    sum = 0
    for value in data:
        sum += value
    mean = sum / len(data)
    if (mean > 0):
        for i, value in enumerate(data):
            data[i] = value + mean
    else:
        for i, value in enumerate(data):
            data[i] = value - mean
    return data

def gen_dummy_data(f, sample_period, start_time, end_time):
    # Generate dummy data for debug purposes.
    # Sine wave function (of freq (Hz)) = A*Sin(2*pi*f*t), we will take A as 1
    fs = 1/sample_period
    t = np.arange(start_time, end_time, 1/fs)
    sin_data = np.sin(2*math.pi*f*t)

    # Add HF noise
    noise_data = np.random.normal(0, 0.25, len(sin_data))

    # plt.figure()
    # plt.plot(t, sin_data + noise_data)   
    # plt.title('Dummy Sine Data')
    # plt.xlabel('time')
    # plt.ylabel('Amplitude')
    # plt.show()
    return [t, sin_data + noise_data]

def lpf_data(data, cutoff, sample_period):
    # Filter requirements.
    T = sample_period       # Sample Period
    fs = 1/T      # sample rate, Hz
    nyq = 0.5 * fs  # Nyquist Frequency
    order = 2       # sin wave can be approx represented as quadratic
    n = int(T * fs) # total number of samples
    
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients 
    filt = butter(order, normal_cutoff, btype='lowpass', analog=False, output='sos')
    filtered_sig = sosfilt(filt, data)
    return filtered_sig

def hpf_data(data, cutoff, sample_period):
     # Filter requirements.
    T = sample_period       # Sample Period
    fs = 1/T      # sample rate, Hz
    nyq = 0.5 * fs  # Nyquist Frequency
    order = 2       # sin wave can be approx represented as quadratic
    n = int(T * fs) # total number of samples
    
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients 
    filt = butter(order, normal_cutoff, btype='highpass', analog=False, output='sos')
    filtered_sig = sosfilt(filt, data)
    return filtered_sig

def bpf_data(data, cutoff_low, cutoff_high, sample_period):
    # Filter requirements.
    T = sample_period       # Sample Period
    fs = 1/T      # sample rate, Hz
    nyq = 0.5 * fs  # Nyquist Frequency
    order = 2       # sin wave can be approx represented as quadratic
    n = int(T * fs) # total number of samples
    
    normal_cutoff_low = cutoff_low / nyq
    normal_cutoff_high = cutoff_high / nyq

    # Get the filter coefficients 
    filt = butter(order, [normal_cutoff_low, normal_cutoff_high], btype='bandpass', analog=False, output='sos')
    filtered_sig = sosfilt(filt, data)
    return filtered_sig

if __name__ == '__main__':
    read_file = True
    if read_file:
        # Create datalog object
        dlog = datalog()
        with open(input_file_path, "r") as f:
            line = f.readline()
            while line:
                line_list  = line.split('\t')
                dlog.elap_milli.append(int(line_list[0].split(' ')[-1]))
                dlog.current_sample = line_list[1].split(' ')[-1]
                dlog.latitude.append(line_list[2].split(' ')[-1])
                dlog.longitude.append(line_list[3].split(' ')[-1])
                dlog.num_strokes = line_list[4].split(' ')[-1]
                dlog.aX.append(line_list[5].split(' ')[-1])
                dlog.aY.append(line_list[6].split(' ')[-1])
                dlog.aZ.append(line_list[7].split(' ')[-1])
                # Read next line
                line = f.readline()
        # set current sample back to zero
        dlog.current_sample = 0
        count_samples()
        avg_accel_data()
        plot_coords(sample_interval=500)
        calc_total_distance()
        calc_max_speed(sample_interval=2500)
        plot_accel(plot_all_axes_sum=True, round_precision=6, x_axis_step=5, plot_avg=True, start_point=6500, end_point=7000)
    else:
        # Read from Serial Port
        dlog = datalog()
        aX_fft_idx = 0
        num_fft_samples = 40
        sample_freq = 1.33
        sample_period = 1 / sample_freq
        aX_fft_arr = [0] * num_fft_samples
        plt.figure()
        with serial.Serial(port='COM4',baudrate=115200) as ser:
            while(1):
                ser_line = ser.readline()
                if ser_line:
                    ser_str = ser_line.decode('ascii')
                    print(ser_str)
                    ser_list = ser_str.split("\t")
                    if len(ser_list) > 1:
                        x_accel = ser_list[4].split(" ")[-1]
                        sample = ser_list[1].split(" ")[-1]
                        dlog.current_sample = sample
                        dlog.aX.append(x_accel)
                        aX_fft_arr[aX_fft_idx] = x_accel
                        if aX_fft_idx == num_fft_samples - 1:
                            # Run FFT
                            # Calc Double-sided power spectrum
                            aX_fft_mag = abs(np.fft.fft(aX_fft_arr))
                            aX_fft_freq = np.fft.fftfreq(n=num_fft_samples, d=sample_period)
                            # Clear previous plot 
                            plt.clf()
                            plt.scatter(aX_fft_freq, aX_fft_mag)
                            # Label points for easy identification of spectral content
                            for i, freq in enumerate(aX_fft_freq):
                                if freq >= -1.5 and freq <= 1.5:
                                    plt.annotate(freq, (freq, aX_fft_mag[i]))
                            plt.ylim([0, 10])
                            plt.title("Double-sided power spectrum for X-axis")
                            plt.xlabel("Frequency (Hz)")
                            plt.ylabel("Magnitude |aX|")
                            plt.pause(0.05)
                            plt.show(block=False)
                            aX_fft_idx = 0
                        else:
                            aX_fft_idx += 1

        
                