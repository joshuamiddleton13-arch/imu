import time
import math
import IMU
import datetime
import os
import sys
import smbus
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import spidev as SPI
sys.path.append("..")
from lib import LCD_2inch
from PIL import Image,ImageDraw,ImageFont
import serial



# define BMP388 Device I2C address
I2C_ADD_BMP388_AD0_LOW = 0x76
I2C_ADD_BMP388_AD0_HIGH = 0x77
I2C_ADD_BMP388 = I2C_ADD_BMP388_AD0_HIGH

BMP388_REG_ADD_WIA = 0x00
BMP388_REG_VAL_WIA = 0x50

BMP388_REG_ADD_ERR = 0x02
BMP388_REG_VAL_FATAL_ERR = 0x01
BMP388_REG_VAL_CMD_ERR = 0x02
BMP388_REG_VAL_CONF_ERR = 0x04

BMP388_REG_ADD_STATUS = 0x03
BMP388_REG_VAL_CMD_RDY = 0x10
BMP388_REG_VAL_DRDY_PRESS = 0x20
BMP388_REG_VAL_DRDY_TEMP = 0x40

BMP388_REG_ADD_CMD = 0x7E
BMP388_REG_VAL_EXTMODE_EN = 0x34
BMP388_REG_VAL_FIFI_FLUSH = 0xB0
BMP388_REG_VAL_SOFT_RESET = 0xB6

BMP388_REG_ADD_PWR_CTRL = 0x1B
BMP388_REG_VAL_PRESS_EN = 0x01
BMP388_REG_VAL_TEMP_EN = 0x02
BMP388_REG_VAL_NORMAL_MODE = 0x30

BMP388_REG_ADD_PRESS_XLSB = 0x04
BMP388_REG_ADD_PRESS_LSB = 0x05
BMP388_REG_ADD_PRESS_MSB = 0x06
BMP388_REG_ADD_TEMP_XLSB = 0x07
BMP388_REG_ADD_TEMP_LSB = 0x08
BMP388_REG_ADD_TEMP_MSB = 0x09

BMP388_REG_ADD_T1_LSB = 0x31
BMP388_REG_ADD_T1_MSB = 0x32
BMP388_REG_ADD_T2_LSB = 0x33
BMP388_REG_ADD_T2_MSB = 0x34
BMP388_REG_ADD_T3 = 0x35
BMP388_REG_ADD_P1_LSB = 0x36
BMP388_REG_ADD_P1_MSB = 0x37
BMP388_REG_ADD_P2_LSB = 0x38
BMP388_REG_ADD_P2_MSB = 0x39
BMP388_REG_ADD_P3 = 0x3A
BMP388_REG_ADD_P4 = 0x3B
BMP388_REG_ADD_P5_LSB = 0x3C
BMP388_REG_ADD_P5_MSB = 0x3D
BMP388_REG_ADD_P6_LSB = 0x3E
BMP388_REG_ADD_P6_MSB = 0x3F
BMP388_REG_ADD_P7 = 0x40
BMP388_REG_ADD_P8 = 0x41
BMP388_REG_ADD_P9_LSB = 0x42
BMP388_REG_ADD_P9_MSB = 0x43
BMP388_REG_ADD_P10 = 0x44
BMP388_REG_ADD_P11 = 0x45

RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA = 0.40  # Complementary filter constant

magXmin = 0
magYmin = 0
magZmin = 0
magXmax = 0
magYmax = 0
magZmax = 0

# Kalman filter variables
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

class BMP388(object):
    """docstring for BMP388"""

    def __init__(self, address=I2C_ADD_BMP388):
        self._address = address
        self._bus = smbus.SMBus(0x01)

        # Load calibration values.

        if self._read_byte(BMP388_REG_ADD_WIA) == BMP388_REG_VAL_WIA:
            print("Pressure sersor is BMP388!\r\n")
            u8RegData = self._read_byte(BMP388_REG_ADD_STATUS)
            if u8RegData & BMP388_REG_VAL_CMD_RDY:
                self._write_byte(BMP388_REG_ADD_CMD,
                                 BMP388_REG_VAL_SOFT_RESET)
                time.sleep(0.01)
        else:
            print("Pressure sersor NULL!\r\n")
        self._write_byte(BMP388_REG_ADD_PWR_CTRL,
                         BMP388_REG_VAL_PRESS_EN
                         | BMP388_REG_VAL_TEMP_EN
                         | BMP388_REG_VAL_NORMAL_MODE)
        self._load_calibration()

    def _read_byte(self, cmd):
        return self._bus.read_byte_data(self._address, cmd)

    def _read_s8(self, cmd):
        result = self._read_byte(cmd)
        if result > 128:
            result -= 256
        return result

    def _read_u16(self, cmd):
        LSB = self._bus.read_byte_data(self._address, cmd)
        MSB = self._bus.read_byte_data(self._address, cmd + 0x01)
        return (MSB << 0x08) + LSB

    def _read_s16(self, cmd):
        result = self._read_u16(cmd)
        if result > 32767:
            result -= 65536
        return result

    def _write_byte(self, cmd, val):
        self._bus.write_byte_data(self._address, cmd, val)

    def _load_calibration(self):
        print("_load_calibration\r\n")
        self.T1 = self._read_u16(BMP388_REG_ADD_T1_LSB)
        self.T2 = self._read_u16(BMP388_REG_ADD_T2_LSB)
        self.T3 = self._read_s8(BMP388_REG_ADD_T3)
        self.P1 = self._read_s16(BMP388_REG_ADD_P1_LSB)
        self.P2 = self._read_s16(BMP388_REG_ADD_P2_LSB)
        self.P3 = self._read_s8(BMP388_REG_ADD_P3)
        self.P4 = self._read_s8(BMP388_REG_ADD_P4)
        self.P5 = self._read_u16(BMP388_REG_ADD_P5_LSB)
        self.P6 = self._read_u16(BMP388_REG_ADD_P6_LSB)
        self.P7 = self._read_s8(BMP388_REG_ADD_P7)
        self.P8 = self._read_s8(BMP388_REG_ADD_P8)
        self.P9 = self._read_s16(BMP388_REG_ADD_P9_LSB)
        self.P10 = self._read_s8(BMP388_REG_ADD_P10)
        self.P11 = self._read_s8(BMP388_REG_ADD_P11)

    def compensate_temperature(self, adc_T):
        partial_data1 = adc_T - 256 * self.T1
        partial_data2 = self.T2 * partial_data1
        partial_data3 = partial_data1 * partial_data1
        partial_data4 = partial_data3 * self.T3
        partial_data5 = partial_data2 * 262144 + partial_data4
        partial_data6 = partial_data5 / 4294967296
        self.T_fine = partial_data6
        comp_temp = partial_data6 * 25 / 16384
        return comp_temp

    def compensate_pressure(self, adc_P):
        partial_data1 = self.T_fine * self.T_fine
        partial_data2 = partial_data1 / 0x40
        partial_data3 = partial_data2 * self.T_fine / 256
        partial_data4 = self.P8 * partial_data3 / 0x20
        partial_data5 = self.P7 * partial_data1 * 0x10
        partial_data6 = self.P6 * self.T_fine * 4194304
        offset = self.P5 * 140737488355328 + partial_data4 \
                 + partial_data5 + partial_data6

        partial_data2 = self.P4 * partial_data3 / 0x20
        partial_data4 = self.P3 * partial_data1 * 0x04
        partial_data5 = (self.P2 - 16384) * self.T_fine * 2097152
        sensitivity = (self.P1 - 16384) * 70368744177664 \
                      + partial_data2 + partial_data4 + partial_data5

        partial_data1 = sensitivity / 16777216 * adc_P
        partial_data2 = self.P10 * self.T_fine
        partial_data3 = partial_data2 + 65536 * self.P9
        partial_data4 = partial_data3 * adc_P / 8192
        partial_data5 = partial_data4 * adc_P / 512
        partial_data6 = adc_P * adc_P
        partial_data2 = self.P11 * partial_data6 / 65536
        partial_data3 = partial_data2 * adc_P / 128
        partial_data4 = offset / 0x04 + partial_data1 + partial_data5 \
                        + partial_data3
        comp_press = partial_data4 * 25 / 1099511627776
        return comp_press

    def get_temperature_and_pressure_and_altitude(self):
        """Returns pressure in Pa as double. Output value of "6386.2"equals 96386.2 Pa = 963.862 hPa."""

        xlsb = self._read_byte(BMP388_REG_ADD_TEMP_XLSB)
        lsb = self._read_byte(BMP388_REG_ADD_TEMP_LSB)
        msb = self._read_byte(BMP388_REG_ADD_TEMP_MSB)
        adc_T = (msb << 0x10) + (lsb << 0x08) + xlsb
        temperature = self.compensate_temperature(adc_T)
        xlsb = self._read_byte(BMP388_REG_ADD_PRESS_XLSB)
        lsb = self._read_byte(BMP388_REG_ADD_PRESS_LSB)
        msb = self._read_byte(BMP388_REG_ADD_PRESS_MSB)

        adc_P = (msb << 0x10) + (lsb << 0x08) + xlsb
        pressure = self.compensate_pressure(adc_P)
        altitude = 4433000 * (0x01 - pow(pressure / 100.0 / 101325.0,
                                         0.1903))

        return (temperature, pressure, altitude)

def moving_average(data, window_size):
    box = np.ones(window_size) / window_size
    return np.convolve(data, box, mode='same')

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

def kalmanFilterY(accAngle, gyroRate, DT):
    y = 0.0
    S = 0.0

    global KFangleY
    global Q_angle
    global Q_gyro
    global y_bias
    global YP_00
    global YP_01
    global YP_10
    global YP_11

    KFangleY = KFangleY + DT * (gyroRate - y_bias)

    YP_00 = YP_00 + (- DT * (YP_10 + YP_01) + Q_angle * DT)
    YP_01 = YP_01 + (- DT * YP_11)
    YP_10 = YP_10 + (- DT * YP_11)
    YP_11 = YP_11 + (+ Q_gyro * DT)

    y = accAngle - KFangleY
    S = YP_00 + R_angle
    K_0 = YP_00 / S
    K_1 = YP_10 / S

    KFangleY = KFangleY + (K_0 * y)
    y_bias = y_bias + (K_1 * y)

    YP_00 = YP_00 - (K_0 * YP_00)
    YP_01 = YP_01 - (K_0 * YP_01)
    YP_10 = YP_10 - (K_1 * YP_00)
    YP_11 = YP_11 - (K_1 * YP_01)

    return KFangleY

def kalmanFilterX(accAngle, gyroRate, DT):
    x = 0.0
    S = 0.0

    global KFangleX
    global Q_angle
    global Q_gyro
    global x_bias
    global XP_00
    global XP_01
    global XP_10
    global XP_11

    KFangleX = KFangleX + DT * (gyroRate - x_bias)

    XP_00 = XP_00 + (- DT * (XP_10 + XP_01) + Q_angle * DT)
    XP_01 = XP_01 + (- DT * XP_11)
    XP_10 = XP_10 + (- DT * XP_11)
    XP_11 = XP_11 + (+ Q_gyro * DT)

    x = accAngle - KFangleX
    S = XP_00 + R_angle
    K_0 = XP_00 / S
    K_1 = XP_10 / S

    KFangleX = KFangleX + (K_0 * x)
    x_bias = x_bias + (K_1 * x)

    XP_00 = XP_00 - (K_0 * XP_00)
    XP_01 = XP_01 - (K_0 * XP_01)
    XP_10 = XP_10 - (K_1 * XP_00)
    XP_11 = XP_11 - (K_1 * XP_01)

    return KFangleX


######################################################################################################
save_csv_log = False


######################################################################################################

# setup code for display:
RST = 27
DC = 25
BL = 18
bus = 0
device = 0
disp = LCD_2inch.LCD_2inch()
# Initialize library.
disp.Init()
# Clear display.
disp.clear()
#Set the backlight to 100
disp.bl_DutyCycle(100)
image_collecting = Image.open('/home/mmidd/imu/test_code/collecting_data.png')
disp.ShowImage(image_collecting)

#setup code for imu:
IMU.detectIMU()  # Detect if BerryIMU is connected.
if (IMU.BerryIMUversion == 99):
    print(" No BerryIMU found... exiting ")
    sys.exit()
IMU.initIMU()  # Initialise the accelerometer, gyroscope and compass

# setup cell modem
ser = serial.Serial("/dev/ttyUSB2", 115200)
ser.flushInput()
phone_number = '13152476985'
rec_buff = ''


# logging variables:
gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0
kalmanX = 0.0
kalmanY = 0.0

cwd = os.getcwd()
bmp388 = BMP388()

string_list = ["Time,KalmanX,KalmanY,Altitude,Pressure,AccX,AccY,AccZ,GyrX,GyrY,GyrZ"]

time_count = 0
a = datetime.datetime.now()
starting_time = datetime.datetime.now()

kalmanx_vector = []
altitude_vector = []
time_vector = []

car_parked = False
fft_buffer = []
fft_time_buffer = []
consecutive_count = 0

while car_parked == False:

    # Read the accelerometer,gyroscope and magnetometer values
    ACCx = IMU.readACCx()
    ACCy = IMU.readACCy()
    ACCz = IMU.readACCz()
    GYRx = IMU.readGYRx()
    GYRy = IMU.readGYRy()
    GYRz = IMU.readGYRz()

    ##Calculate loop Period(LP). How long between Gyro Reads
    b = datetime.datetime.now() - a
    a = datetime.datetime.now()
    LP = b.microseconds / (1000000 * 1.0)
    # outputString = "Loop Time %5.2f " % ( LP )
    outputString = str((a - starting_time).total_seconds())
    time_count += b.total_seconds()
    # Convert Gyro raw to degrees per second
    rate_gyr_x = GYRx * G_GAIN
    rate_gyr_y = GYRy * G_GAIN
    rate_gyr_z = GYRz * G_GAIN
    # Calculate the angles from the gyro.
    gyroXangle += rate_gyr_x * LP
    gyroYangle += rate_gyr_y * LP
    gyroZangle += rate_gyr_z * LP
    # Convert Accelerometer values to degrees
    AccXangle = (math.atan2(ACCy, ACCz) * RAD_TO_DEG)
    AccYangle = (math.atan2(ACCz, ACCx) + M_PI) * RAD_TO_DEG
    # convert the values to -180 and +180
    if AccYangle > 90:
        AccYangle -= 270.0
    else:
        AccYangle += 90.0
    # Kalman filter used to combine the accelerometer and gyro values.
    kalmanY = kalmanFilterY(AccYangle, rate_gyr_y, LP)
    kalmanX = kalmanFilterX(AccXangle, rate_gyr_x, LP)

    temperature, pressure, altitude = bmp388.get_temperature_and_pressure_and_altitude()
    pressure = pressure / 100.0
    altitude = altitude / 100.0
    if(save_csv_log):
        outputString += "," + str(kalmanX)
        outputString += "," + str(kalmanY)
        outputString += "," + str(altitude)
        outputString += "," + str(pressure)
        outputString += "," + str(ACCx)
        outputString += "," + str(ACCy)
        outputString += "," + str(ACCz)
        outputString += "," + str(GYRx)
        outputString += "," + str(GYRy)
        outputString += "," + str(GYRz)
        string_list.append(outputString)

    kalmanx_vector.append(kalmanX)
    altitude_vector.append(altitude)
    time_vector.append(float((a - starting_time).total_seconds()))

    #check if parked
    fft_buffer.append(ACCy)
    fft_time_buffer.append((a - starting_time).total_seconds())
    if(len(fft_buffer) == 256):
        signal = fft_buffer
        N = len(signal)
        T = (fft_time_buffer[-1] - fft_time_buffer[0]) / 256.0
        # Compute the FFT of the signal
        fft_result = np.fft.fft(signal)
        # Compute the frequencies corresponding to the FFT result
        #freq = np.fft.fftfreq(N, T)
        magnitude = np.abs(fft_result)

        vibration_score = np.sum(magnitude[4:N // 2])
        if(vibration_score < 38000):
            consecutive_count += 1
        else:
            consecutive_count = 0
        if(consecutive_count > 5):
            car_parked = True
            image_parked = Image.open('/home/mmidd/imu/test_code/car_parked.png')
            disp.ShowImage(image_parked)
            time.sleep(6.0)

        fft_buffer = []
        fft_time_buffer = []

    time.sleep(0.001)

if(save_csv_log):
    with open('/home/mmidd/imu/test_code/output_log_' + str(datetime.datetime.now()) + '.csv', 'w') as file:
        for string in string_list:
            file.write(string + '\n')


image_processing = Image.open('/home/mmidd/imu/test_code/processing_data.png')
disp.ShowImage(image_processing)

kalmanx = np.array(kalmanx_vector)
altitude = np.array(altitude_vector)
time_v = np.array(time_vector)

# look at only the prior 6 minutes of data or less
end_time = time_v[-1]
cut_index = np.argmax(time > end_time-360.0)
kalmanx = kalmanx[cut_index:]
altitude = altitude[altitude:]
time_v = time_v[cut_index:]

# now post process data
smoothed_kalman_x = moving_average(kalmanx, 35)
steady_state_angle = np.where(time_v > time_v[-1]-20.0, smoothed_kalman_x, 0.0)
steady_state_angle = steady_state_angle[steady_state_angle != 0].mean()
smoothed_kalman_x = smoothed_kalman_x - steady_state_angle
smoothed_alt = moving_average(altitude, 35)
#combined metric
tilt_threshold = -4.0 # degrees
height_threshold = 1.65 # meters
air_pressure_time_buffer = 2.25 #seconds
rise_intervals = np.where(smoothed_kalman_x < tilt_threshold, smoothed_alt, 0.0)
rise_intervals_time = np.where(smoothed_kalman_x < tilt_threshold, time_v, 0.0)
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
        time_intervals[i] = np.append(time_intervals[i], time_v[index])
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
    plot_x = time_intervals[i]
    plot_y = []
    for x in plot_x:
        plot_y.append(B[0]*x + B[1])

    if(plot_y[-1] - plot_y[0] > height_threshold):
        floor_increase_counter += 1

if(floor_increase_counter > 0):
    #send floor result
    try:
        floor_result = floor_map[floor_increase_counter]
    except:
        floor_result = 99

    fig, ax = plt.subplots(figsize=(2.4, 3.2))
    fig.subplots_adjust(0, 0, 1, 1)
    ax.text(.5,.5,str(floor_result), ha='center', fontsize=20, family="monospace")
    ax.axis('off')
    fig.savefig('/home/mmidd/imu/test_code/data_out.png', bbox_inches='tight', pad_inches=0, dpi=100.0)

    image_result = Image.open('/home/mmidd/imu/test_code/data_out.png')
    disp.ShowImage(image_result)

    text_message = 'Floor Parked: ' + str(floor_result)
    command = "AT+CMGF=1"
    ser.write((command+'\r\n').encode())
    time.sleep(1.0)
    command = "AT+CMGS=\""+phone_number+"\""
    ser.write((command+'\r\n').encode())
    time.sleep(1.0)
    ser.write(text_message.encode())
    time.sleep(2.0)
    ser.write(b'\x1A')
    time.sleep(5.0)

else:
    image_result = Image.open('/home/mmidd/imu/test_code/not_at_home.png')
    disp.ShowImage(image_result)


time.sleep(15.0)
# now close program and shut down
disp.module_exit()
time.sleep(2.0)
ser.close()

time.sleep(5.0)
os.system("sudo shutdown now")


