from machine import I2C
from machine import Pin, PWM
import time
import mpu6050
import math

MPU_I2C_SCL = 22
MPU_I2C_SDA = 21
PWM_PIN = 17
BUTM_PIN = 16

PWM_FREQ = 86000 # 50kHz avoid flashing
PWM_DUTY = 0 # 0%
DATA_BUFFER_LENGTH = 30  # filter data length
DATA_FILTER_LENGTH = 5   # filter length, each time update 25 data

class MPU_DRIVER:
    """
    this class is used to get the data from mpu6050
    it includes a filter to filter the original data
    used moving average filter
    """
    def __init__(self, mpu_i2c, data_buffer_length=100, data_filter_length=25):
        self.i2c = mpu_i2c
        self.mpu = mpu6050.accel(self.i2c)

        self.data_length = data_buffer_length
        self.filt_length = data_filter_length

        self.gx_axis_data = []
        self.gy_axis_data = []
        self.gz_axis_data = []
        self.ax_axis_data = []
        self.ay_axis_data = []
        self.az_axis_data = []
    
    def init_filter(self):
        self.gx_axis_data = []
        self.gy_axis_data = []
        self.gz_axis_data = []
        self.ax_axis_data = []
        self.ay_axis_data = []
        self.az_axis_data = []
        for _ in range(self.data_length):
            self.gx_axis_data.append(self.mpu.get_values()["GyX"])
            self.gy_axis_data.append(self.mpu.get_values()["GyY"])
            self.gz_axis_data.append(self.mpu.get_values()["GyZ"])
            self.ax_axis_data.append(self.mpu.get_values()["AcX"])
            self.ay_axis_data.append(self.mpu.get_values()["AcY"])
            self.az_axis_data.append(self.mpu.get_values()["AcZ"])
    
    def filt(self):
        for _ in range(self.filt_length):
            self.gx_axis_data.pop(0)
            self.gy_axis_data.pop(0)
            self.gz_axis_data.pop(0)
            self.ax_axis_data.pop(0)
            self.ay_axis_data.pop(0)
            self.az_axis_data.pop(0)
            self.gx_axis_data.append(self.mpu.get_values()["GyX"])
            self.gy_axis_data.append(self.mpu.get_values()["GyY"])
            self.gz_axis_data.append(self.mpu.get_values()["GyZ"])
            self.ax_axis_data.append(self.mpu.get_values()["AcX"])
            self.ay_axis_data.append(self.mpu.get_values()["AcY"])
            self.az_axis_data.append(self.mpu.get_values()["AcZ"])

        gx = sum(self.gx_axis_data) / self.data_length
        gy = sum(self.gy_axis_data) / self.data_length
        gz = sum(self.gz_axis_data) / self.data_length
        ax = sum(self.ax_axis_data) / self.data_length
        ay = sum(self.ay_axis_data) / self.data_length
        az = sum(self.az_axis_data) / self.data_length

        return "{} {} {} {} {} {}".format(gx, gy, gz, ax, ay, az)


class TST:
    """
    this class is used to control the MCU ESP32
    it includes a mpu6050 driver, a button and a pwm driver
    it can control the pwm duty cycle by the mpu6050
    """
    def __init__(self, mpu, butm, pwm):
        self.mpu = mpu
        self.butm = butm
        self.pwm = pwm

        self.last_control_flag = False

    def is_control(self):
        res = True
        for _ in range(5):  # 5 times check to avoid shaking
            if self.butm.value() == 0:
                res = False
        return res
    
    def ligh_on(self):
        self.pwm.duty_u16(6000)

    def ligh_off(self):
        self.pwm.duty_u16(0)

    def start(self):
        while True:
            if self.is_control():
                # if the button is pressed, start control
                if self.last_control_flag == False:
                    # if the button is pressed at the first time
                    self.mpu.init_filter()
                    self.last_control_flag = True
                    self.ligh_on()
                    print("START")

                # get the data from mpu6050
                print(self.mpu.filt())
                    
            else:
                if self.last_control_flag == True:
                    # if the button is released at the first time
                    self.last_control_flag = False
                    self.ligh_off()
                    print("STOP")

# here is the main function
mpu_i2c = I2C(scl=Pin(MPU_I2C_SCL), sda=Pin(MPU_I2C_SDA))
mpu = MPU_DRIVER(mpu_i2c, DATA_BUFFER_LENGTH, DATA_FILTER_LENGTH)
pwm = PWM(Pin(PWM_PIN), freq=PWM_FREQ, duty_u16=PWM_DUTY)
butm = Pin(BUTM_PIN, Pin.IN)

tst = TST(mpu, butm, pwm)
tst.start()