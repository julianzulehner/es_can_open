import serial 

ser = serial.Serial("COM17", baudrate=115200)

while True:
    res = ser.read_until(b"\r\n") 
    res_str = res.decode('ascii').strip()
    print(res_str)

ser.close()