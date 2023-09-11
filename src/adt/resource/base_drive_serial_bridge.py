import serial


if __name__ == '__main__':
    ser = serial.Serial('COM8', 9600)
    print(ser.name)
    ser.write(b'mz')
    ser.close()
