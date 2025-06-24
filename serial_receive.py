import serial
import time

startTime= None
receiveLen = 0

readSer = serial.Serial(port='COM3', baudrate=115200, parity= 'N', timeout=10)
while True:
	data = readSer.read(1024)
	if startTime == None:
		startTime = time.time()
		print("First time received.")
	receiveLen = receiveLen + 1
	if receiveLen % 1000 == 0:
		print(str(receiveLen) + "times received.")
	if receiveLen >= 100000:
		break
endTime = time.time()
spentTime = endTime - startTime
print(spentTime)
readSer.close()
