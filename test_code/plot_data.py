
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import time
import math
import datetime
import os
import sys

from fontTools.ttLib.woff2 import bboxFormat

matplotlib.use('TkAgg')
from whittaker_eilers import WhittakerSmoother




def extract_nonzero_segments(vec):

    # Find indices where values are nonzero
    nonzero_mask = vec != 0

    # Find the start and end of each nonzero segment
    # Pad with False to handle segments at boundaries
    padded = np.concatenate(([False], nonzero_mask, [False]))

    # Find where segments start and end
    diff = np.diff(padded.astype(int))
    starts = np.where(diff == 1)[0]
    ends = np.where(diff == -1)[0]

    # Extract each segment
    segments = [vec[start:end] for start, end in zip(starts, ends)]

    return segments


RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40      # Complementary filter constant





#Kalman filter variables
Q_angle = 0.01
Q_gyro = 0.0015

R_angle = 0.005
y_bias = 0.0
x_bias = 0.0
XP_00 = 0.0
XP_01 = 0.0
XP_10 = 0.0
XP_11 = 0.0
YP_00 = 0.0
YP_01 = 0.0
YP_10 = 0.0
YP_11 = 0.0
KFangleX = 0.0
KFangleY = 0.0




def kalmanFilterY ( accAngle, gyroRate, DT):
    y=0.0
    S=0.0

    global KFangleY
    global Q_angle
    global Q_gyro
    global y_bias
    global YP_00
    global YP_01
    global YP_10
    global YP_11

    KFangleY = KFangleY + DT * (gyroRate - y_bias)

    YP_00 = YP_00 + ( - DT * (YP_10 + YP_01) + Q_angle * DT )
    YP_01 = YP_01 + ( - DT * YP_11 )
    YP_10 = YP_10 + ( - DT * YP_11 )
    YP_11 = YP_11 + ( + Q_gyro * DT )

    y = accAngle - KFangleY
    S = YP_00 + R_angle
    K_0 = YP_00 / S
    K_1 = YP_10 / S

    KFangleY = KFangleY + ( K_0 * y )
    y_bias = y_bias + ( K_1 * y )

    YP_00 = YP_00 - ( K_0 * YP_00 )
    YP_01 = YP_01 - ( K_0 * YP_01 )
    YP_10 = YP_10 - ( K_1 * YP_00 )
    YP_11 = YP_11 - ( K_1 * YP_01 )

    return KFangleY

def kalmanFilterX ( accAngle, gyroRate, DT):
    x=0.0
    S=0.0

    global KFangleX
    global Q_angle
    global Q_gyro
    global x_bias
    global XP_00
    global XP_01
    global XP_10
    global XP_11


    KFangleX = KFangleX + DT * (gyroRate - x_bias)

    XP_00 = XP_00 + ( - DT * (XP_10 + XP_01) + Q_angle * DT )
    XP_01 = XP_01 + ( - DT * XP_11 )
    XP_10 = XP_10 + ( - DT * XP_11 )
    XP_11 = XP_11 + ( + Q_gyro * DT )

    x = accAngle - KFangleX
    S = XP_00 + R_angle
    K_0 = XP_00 / S
    K_1 = XP_10 / S

    KFangleX = KFangleX + ( K_0 * x )
    x_bias = x_bias + ( K_1 * x )

    XP_00 = XP_00 - ( K_0 * XP_00 )
    XP_01 = XP_01 - ( K_0 * XP_01 )
    XP_10 = XP_10 - ( K_1 * XP_00 )
    XP_11 = XP_11 - ( K_1 * XP_01 )

    return KFangleX




gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0
CFangleX = 0.0
CFangleY = 0.0
kalmanX = 0.0
kalmanY = 0.0

#df = pd.read_csv("output_test_initial.csv")
df = pd.read_csv(r"C:\Users\mmidd\Downloads\output_log_2025-11_15_run.csv")

time = df["Time"].to_numpy()
Ax = df["AccX"].to_numpy()
Ay = df["AccY"].to_numpy()
Az = df["AccZ"].to_numpy()
Gx = df["GyrX"].to_numpy()
Gy = df["GyrY"].to_numpy()
Gz = df["GyrZ"].to_numpy()

t_prev = 0.0

kalman_x = []
kalman_y = []
comp_x = []
comp_y = []

for i in range(len(time)):

    #Read the accelerometer,gyroscope and magnetometer values
    ACCx = Ax[i]
    ACCy = Ay[i]
    ACCz = Az[i]
    GYRx = Gx[i]
    GYRy = Gy[i]
    GYRz = Gz[i]

    ##Calculate loop Period(LP). How long between Gyro Reads
    LP = time[i] - t_prev
    t_prev = time[i]

    #Convert Gyro raw to degrees per second
    rate_gyr_x =  GYRx * G_GAIN
    rate_gyr_y =  GYRy * G_GAIN
    rate_gyr_z =  GYRz * G_GAIN

    #Calculate the angles from the gyro.
    gyroXangle+=rate_gyr_x*LP
    gyroYangle+=rate_gyr_y*LP
    gyroZangle+=rate_gyr_z*LP

   #Convert Accelerometer values to degrees
    AccXangle =  (math.atan2(ACCy,ACCz)*RAD_TO_DEG)
    AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG

    #convert the values to -180 and +180
    if AccYangle > 90:
        AccYangle -= 270.0
    else:
        AccYangle += 90.0

    #Complementary filter used to combine the accelerometer and gyro values.
    CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
    CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle

    #Kalman filter used to combine the accelerometer and gyro values.
    kalmanY = kalmanFilterY(AccYangle, rate_gyr_y,LP)
    kalmanX = kalmanFilterX(AccXangle, rate_gyr_x,LP)

    #append results for this time step
    kalman_x.append(kalmanX)
    kalman_y.append(kalmanY)

    comp_x.append(CFangleX)
    comp_y.append(CFangleY)

print(df)

def moving_average(data, window_size):
    box = np.ones(window_size) / window_size
    return np.convolve(data, box, mode='same')



# first do angle measurement
whittaker_smoother = WhittakerSmoother(lmbda=1.0e5, order=2, data_length=len(kalman_x))
smoothed_kalman_x = whittaker_smoother.smooth(kalman_x)
smoothed_kalman_x = moving_average(kalman_x, 35)

steady_state_angle = np.where(time > time[-1]-60.0, smoothed_kalman_x, 0.0)
steady_state_angle = steady_state_angle[steady_state_angle != 0].mean()
print(steady_state_angle)
smoothed_kalman_x = smoothed_kalman_x - steady_state_angle

#plt.figure()
#plt.plot(time, smoothed_kalman_x)

whittaker_smoother = WhittakerSmoother(lmbda=1000000, order=2, data_length=len(df["Altitude"].to_numpy()))
smoothed_alt = whittaker_smoother.smooth(df["Altitude"].to_numpy())
smoothed_alt = moving_average(df["Altitude"].to_numpy(), 35)

#plt.figure()
#plt.plot(time, smoothed_alt, color = 'blue')
'''
fig, ax = plt.subplots(figsize=(2.4, 3.2))
fig.subplots_adjust(0, 0, 1, 1)
ax.plot(time, smoothed_alt, color = 'blue')


#combined metric
tilt_threshold = -4.0 # degrees
height_threshold = 1.55 # meters
air_pressure_time_buffer = 2.25 #seconds
rise_intervals = np.where(smoothed_kalman_x < tilt_threshold, smoothed_alt, 0.0)
rise_intervals_time = np.where(smoothed_kalman_x < tilt_threshold, time, 0.0)
rise_intervals_indices = np.where(smoothed_kalman_x < tilt_threshold, np.arange(len(smoothed_kalman_x)), 0.0)

rise_intervals = extract_nonzero_segments(rise_intervals)
rise_indices = extract_nonzero_segments(rise_intervals_indices)
time_intervals = extract_nonzero_segments(rise_intervals_time)

#extend intervals by time buffer
for i in range(len(time_intervals)):
    end_time_c = time_intervals[i][-1]
    end_index = rise_indices[i][-1]
    target_time_c = end_time_c + air_pressure_time_buffer
    index = int(end_index)
    while(time_intervals[i][-1] < target_time_c):
        time_intervals[i] = np.append(time_intervals[i], time[index])
        rise_intervals[i] = np.append(rise_intervals[i], smoothed_alt[index])
        index += 1

# do linear regression to find slope and average rise
floor_increase_counter = 0
floor_map ={0:'G', 1:'G1', 2:'1', 3:'2', 4:'3', 5:'4', 6:'5', 7:'6', 8:'7', 9:'8'}
for i in range(len(time_intervals)):
    # linear least squares to find average trend of rise points:
    X = np.array([time_intervals[i], np.ones(len(time_intervals[i]))]).transpose()
    y = np.array(rise_intervals[i]).transpose()

    B = np.matmul(np.matmul(np.linalg.inv(np.matmul(X.transpose(), X)), X.transpose()), y)
    print(B)

    plot_x = time_intervals[i]
    plot_y = []
    for x in plot_x:
        plot_y.append(B[0]*x + B[1])

    ax.plot(plot_x, plot_y, color = 'green')


    if(plot_y[-1] - plot_y[0] > height_threshold):
        floor_increase_counter += 1
        ax.plot([time_intervals[i][0], time_intervals[i][-1]],[rise_intervals[i].mean(), rise_intervals[i].mean()], color = 'red')

ax.text(500,np.array(smoothed_alt).mean(),floor_map[floor_increase_counter], ha='center', fontsize=25, family="monospace")
fig.savefig("data_plot.png", bbox_inches='tight', pad_inches=0, dpi=100.0)
'''

# make some images for ops code:
#fig, ax = plt.subplots(figsize=(2.4, 3.2))
#fig.subplots_adjust(0, 0, 1, 1)
#ax.text(.5,.5,"Collecting Data", ha='center', fontsize=14, family="monospace")
#ax.axis('off')
#fig.savefig("collecting_data.png", bbox_inches='tight', pad_inches=0, dpi=100.0)
#
fig, ax = plt.subplots(figsize=(2.4, 3.2))
fig.subplots_adjust(0, 0, 1, 1)

ax.text(.5,.5,"Not at Home", ha='center', fontsize=14, family="monospace")
ax.axis('off')
fig.savefig("not_at_home.png", bbox_inches='tight', pad_inches=0, dpi=100.0)

#plt.figure()
#plt.plot(time, Ax)


# Create a sample signal
#t = np.arange(256)

#signal = np.sin(2 * np.pi * t / 32) + 0.5 * np.sin(2 * np.pi * t / 8)
print(len(Ay))
start_window = 0
print(time)

cut_index = np.argmax(time > time[-1]-300.0)
print(time[cut_index:])


time_windows = []
vibration_score = []
start_window = 0

while(start_window < len(Ay)-257):
    end_window = start_window + 256
    #print((time[start_window]))
    #print(time[end_window])
    time_windows.append(time[start_window])

    signal = Ay[start_window:end_window]
    N = len(signal)
    T = (time[end_window] - time[start_window])/256.0

    # Compute the FFT of the signal
    fft_result = np.fft.fft(signal)

    # Compute the frequencies corresponding to the FFT result
    freq = np.fft.fftfreq(N, T)
    magnitude = np.abs(fft_result)

    vibration_score.append(np.sum(magnitude[4:N//2]))

    # Plot the real and imaginary parts of the FFT result
    #print(freq[5:N//2])
    #print(magnitude[5:N//2])
    #plt.figure()
    #plt.stem(freq[5:N//2], magnitude[5:N//2])
    #plt.xlabel('Frequency')
    #plt.ylabel('Magnitude')
    #plt.ylim(0,10000)
    #plt.legend()
    #plt.show()

    start_window += 256

#print(vibration_score)
plt.figure
plt.plot(time_windows, vibration_score)
plt.show()