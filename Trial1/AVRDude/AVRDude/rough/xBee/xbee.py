import serial
import xbee

ser= serial.Serial('/dev/ttyUSB0',9600,timeout=100000)
xbee = ZigBee(ser)
xbee.tx(dest_addr='13A200', data='8')
ser.close()

