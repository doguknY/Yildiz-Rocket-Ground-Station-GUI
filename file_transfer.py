import serial as sr
import sys


# get file name from command line argument
file_name = sys.argv[1]

# open serial port
ser = sr.Serial('/dev/tty.usbserial-A50285BI', 115200)
ser.write(b'D')
ser.write(open(file_name, 'rb').read())
ser.close()