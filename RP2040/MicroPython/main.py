import machine
import utime
import imu 

i2c = machine.I2C(1, sda=machine.Pin(14), scl=machine.Pin(15), freq=400000)
print("I2C addr: ", i2c.scan()[0])
sensor = imu.MPU6050(i2c)

while True:
    #print(sensor.accel.xyz, sensor.gyro.xyz, sensor.temperature)
    print("加速度：{:7.2f}{:7.2f}{:7.2f}  陀螺仪：{:9.2f}{:9.2f}{:9.2f}  温度：{:5.1f}" \
          .format(sensor.accel.x, sensor.accel.y, sensor.accel.z,  \
                  sensor.gyro.x, sensor.gyro.y, sensor.gyro.z, \
                  sensor.temperature))
    utime.sleep(0.02)