from machine import I2C
from machine import Pin, PWM
import time
import mpu6050

MPU_I2C_SCL = 22
MPU_I2C_SDA = 21
PWM_PIN = 17
BUTM_PIN = 16

PWM_FREQ = 50000 # 50kHz avoid flashing
PWM_DUTY = 0 # 0%
DATA_BUFFER_LENGTH = 100  # filter data length
DATA_FILTER_LENGTH = 25   # filter length, each time update 25 data

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

        self.z_axis_data = []
        self.z_pos = 0
    
    def init_filter(self):
        self.z_axis_data = []
        for _ in range(self.data_length):
            self.z_axis_data.append(self.mpu.get_values()["GyZ"])
        self.vi = self.filt_z()

        self.z_pos = 0
    
    def filt_z(self):
        for _ in range(self.filt_length):
            self.z_axis_data.pop(0)
            self.z_axis_data.append(self.mpu.get_values()["GyZ"])
        result_val = sum(self.z_axis_data) / self.data_length
        return result_val

    def update_position(self):
        v = self.filt_z()
        if abs(v) < 100:
            return self.z_pos
        else:
            self.z_pos += v*0.01  # here 0.01 is the time interval
            return self.z_pos


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

        self.last_duty = 0
        self.now_duty = 0
        self.last_control_flag = False

    def is_control(self):
        res = True
        for _ in range(5):  # 5 times check to avoid shaking
            if self.butm.value() == 0:
                res = False
        return res

    def action_analysis(self):
        pos = self.mpu.update_position() # get position info
        print(pos)
        if abs(pos) < 50: # avoid shaking
            return 0
        else:
            # here transform the position to the duty cycle
            # lets them have the same scale
            return int((pos*16250)//3200)

    def start(self):
        while True:
            if self.is_control():
                # if the button is pressed, start control
                if self.last_control_flag == False:
                    # if the button is pressed at the first time
                    self.mpu.init_filter()
                    self.last_control_flag = True

                duty = self.action_analysis()
                if duty == 0:
                    print("stay")
                    pass
                elif duty > 0:
                    print("up")
                    self.now_duty = self.last_duty + duty
                    if self.now_duty > 65535:
                        self.now_duty = 65535
                    self.pwm.duty_u16(self.now_duty)
                elif duty < 0:
                    print("down")
                    self.now_duty = self.last_duty + duty
                    if self.now_duty < 0:
                        self.now_duty = 0
                    self.pwm.duty_u16(self.now_duty)
            else:
                if self.last_control_flag == True:
                    # if the button is released at the first time
                    # save the last duty for algorithm to use
                    self.last_duty = self.now_duty
                    self.last_control_flag = False

            time.sleep(0.01)


# here is the main function
mpu_i2c = I2C(scl=Pin(MPU_I2C_SCL), sda=Pin(MPU_I2C_SDA))
mpu = MPU_DRIVER(mpu_i2c, DATA_BUFFER_LENGTH, DATA_FILTER_LENGTH)
pwm = PWM(Pin(PWM_PIN), freq=PWM_FREQ, duty_u16=PWM_DUTY)
butm = Pin(BUTM_PIN, Pin.IN)

tst = TST(mpu, butm, pwm)
tst.start()