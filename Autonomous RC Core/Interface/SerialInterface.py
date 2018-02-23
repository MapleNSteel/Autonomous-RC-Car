import serial
import serial.tools.list_ports
import signal
import sys
import rospy
import glob
import time
from std_msgs.msg import Float32, Bool, String

from Header import makeHeader

running = True

serialPort=serial.Serial()
autopilotPort=serialPort

armed=0

throttle=0
steering=0

actualSpeed=0

estop=0
pos=[0,0,0]

Kp=3.5
Ki=1
Kd=0.5

tuningMode=False

def subThrottle(msg):
	global throttle
	if(not estop):
		throttle=msg.data
		autopilotPort.write(makeHeader(throttle, steering, pos, estop))
	
def subSteering(msg):
	global steering
	steering=msg.data
	autopilotPort.write(makeHeader(throttle, steering, pos, estop))
def subA(msg):
	if(msg.data):
		armController()
	else:
		disarmController()
def subTM(msg):
	global tuningMode
	if(msg.data):
		print("Tuning Mode on.")
		tuningMode=1
		autopilotPort.write(b"TM:1")
	else:
		print("Tuning Mode off.")
		tuningMode=0
		autopilotPort.write(b"TM:0")	
		
def subE(msg):
	global estop, throttle
	if(msg.data):
		estop=1
		throttle=0
		autopilotPort.write(b"E:1")
	else:
		estop=0
		autopilotPort.write(b"E:0")	
		
def subKp(msg):
	global Kp
	Kp=msg.data
	autopilotPort.write(b"Kp:"+str(Kp))
def subKi(msg):
	global Ki
	Ki=msg.data
	autopilotPort.write(b"Ki:"+str(Ki))
def subKd(msg):
	global Kd
	Kd=msg.data
	autopilotPort.write(b"Kd:"+str(Kd))
	
subThrottle=rospy.Subscriber("/autonomous_rc/Throttle", Float32, subThrottle)
subSteering=rospy.Subscriber("/autonomous_rc/Steering", Float32, subSteering)

subKproportional=rospy.Subscriber("/autonomous_rc/Kp", Float32, subKp)
subKintegral=rospy.Subscriber("/autonomous_rc/Ki", Float32, subKi)
subKderivative=rospy.Subscriber("/autonomous_rc/Kd", Float32, subKd)

subEstop=rospy.Subscriber("/autonomous_rc/EStop", Bool, subE)
subTuningMode=rospy.Subscriber("/autonomous_rc/TuningMode", Bool, subTM)
subArm=rospy.Subscriber("/autonomous_rc/Armed", Bool, subA)

pubAlive=rospy.Publisher("/autonomous_rc/Heartbeat", Float32, queue_size = 1)
pubTargetSpeed=rospy.Publisher("/autonomous_rc/TargetSpeed", Float32, queue_size = 1)
pubSpeed=rospy.Publisher("/autonomous_rc/Heartbeat", Float32, queue_size = 1)
pubError=rospy.Publisher("/autonomous_rc/Error", Float32, queue_size = 1)


def exit_gracefully(signum, frame):

	running = False
	
	original_sigint = signal.getsignal(signal.SIGINT)
	# restore the original signal handler as otherwise evil things will happen
	# in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
	signal.signal(signal.SIGINT, original_sigint)

	try:
		if raw_input("\nReally quit? (y/n)> ").lower().startswith('y'):

			disarmController()
			autopilotPort.flush()
			autopilotPort.close()
			time.sleep(1)
			sys.exit(1)

	except KeyboardInterrupt:
		print("Ok ok, quitting")
		disarmController()
		autopilotPort.close()
		sys.exit(1)
	
	# restore the exit gracefully handler here	
	signal.signal(signal.SIGINT, exit_gracefully)

def armController():
	global autopilotPort, armed
	if(armed==0):
		autopilotPort.write(b"Arm")
		armed=1
		print("Arming...")
		
def disarmController():
	global autopilotPort, armed
	if(armed==1):
		autopilotPort.write(b"Disarm")	
		armed=0	

def main():

	global autopilotPort
	
	rospy.init_node('interface')
	
	signal.signal(signal.SIGINT, exit_gracefully)

	serialDevices=glob.glob('/dev/tty*')
		
	found=False
	
	print("Searching for autopilot.")
	
	for serialDevice in serialDevices:
		try:
			autopilotPort=serial.Serial(serialDevice, 115200, timeout=0)
		except:
			continue
			
		autopilotPort.flush()
		print("Are you the autopilot?")
		autopilotPort.write(b"Are you the autopilot?")
		while(not autopilotPort.inWaiting()):
		    continue
		a=autopilotPort.readline()[0:-2]
		print(a)
		test="Yes"
		neq=True
		print(test==a)
		if(test==a):
			found=True
			break
		autopilotPort.close()
	
	if(not found):
		autopilotPort.close()
		print("Could not find autopilot.")
		sys.exit(1)
	
	print("Found autopilot.")
	
	armController()
	while(not autopilotPort.inWaiting()):
		continue
	
	while(running):
		if(autopilotPort.inWaiting()>0):
			Terminal_Output=autopilotPort.readline().strip()
			global actualSpeed, tuningMode
			for entry in Terminal_Output[0:-2].split(', '):
				try:
					Id, value = entry.split(': ')
				except ValueError:
					print(entry)
					continue
				if(Id == "Heart Beat"):
					 print("Heart Beat:")
					 print(float(value))
					 pubAlive.publish(float(value))
				if(Id == "Speed"):
					 print("Speed:")
					 print(float(value))
					 actualSpeed=float(value)
					 pubSpeed.publish(float(value))
				if(Id == "Desired Speed"):
					 print("Desired Speed:")
					 print(float(value))
					 pubTargetSpeed.publish(float(value))
				if(tuningMode):
					 print("Error:")
					 print(throttle-actualSpeed)
					 pubError.publish(throttle-actualSpeed)
	
if __name__ == "__main__": 
	main()
