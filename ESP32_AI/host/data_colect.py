import serial
import time

class TST_HOST:

    def __init__(self, serial_port, baud_rate=115200, timeout=2, file_name="data.csv"):
        # 配置串口参数
        serial_port = 'COM3'  # 串口号
        baud_rate = 115200    # MicroPython通常使用115200的波特率
        timeout = 2           # 读取超时时间

        # 创建串口连接
        self.ser = serial.Serial(serial_port, baud_rate, timeout=timeout)

        # 确保串口已打开
        if not self.ser.is_open:
            self.ser.open()

        # 打开文件
        self.file = open(file_name, "w")
        self.label = 1
    
    def format_data(self, data):
        data = data.split(" ")
        lendata = len(data)
        print("raw length:",lendata)

        if lendata%2 != 0:
            data = data[:-1]
            lendata -= 1
        if lendata > 120:
            each_size = int((lendata - 120)/2)
            data = data[each_size:-each_size]
            
        elif lendata < 120:
            each_size = int((120 - lendata)/2)
            data = ["0"]*each_size + data + ["0"]*each_size
        
        print("fix length:",len(data))

        data = ",".join(data)
        data = data + "," + str(self.label) + "\n"
        return data


    def label_data(self):
        # 读取执行结果
        line = ""

        while True:
            # 读取一行数据
            line_temp = self.ser.readline()

            if "START" in line_temp.decode('utf-8'):
                print("开始记录 标签：{}".format(str(self.label)))
                continue

            if "STOP" in line_temp.decode('utf-8'):
                print("停止记录")
                line = line[:-1]
                self.file.write(self.format_data(line))
                line = ""
                continue

            line += line_temp.decode('utf-8')[:-2] + " "

            # 打印数据
            print(line_temp.decode('utf-8').strip())


tst = TST_HOST("COM3")
tst.label_data()
