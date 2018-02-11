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

estop=0
pos=[0,0,0]

Kp=3.5
Ki=1
Kd=0.5

tuningMode=0

def subThrottle(msg):
	throttle=msg.data
	autopilotPort.write(makeHeader(throttle, steering, pos, estop))
def subSteering(msg):
	steering=msg.data
	autopilotPort.write(makeHeader(throttle, steering, pos, estop))
def subA(msg):
	if(msg.data):
		armController()
	else:
		disarmController()
def subTM(msg):
	if(msg.data):
		tuningMode=1
		autopilotPort.write("TM:1")
	else:
		tuningMode=0
		autopilotPort.write("TM:0")	
		
def subE(msg):
	if(msg.data):
		estop=1
		autopilotPort.write("E:1")
	else:
		estop=0
		autopilotPort.write("E:0")	
		
def subKp(msg):
	Kp=msg.data
	autopilotPort.write("Kp:"+str(Kp))
def subKi(msg):
	Kp=msg.data
	autopilotPort.write("Ki:"+str(Ki))
def subKd(msg):
	Kp=msg.data
	autopilotPort.write("Kd:"+str(Kd))
	
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
pubError=rospy.Publisher("/autonomous_rc/Heartbeat", Float32, queue_size = 1)


def exit_gracefully(signum, frame):

	running = False
	
	original_sigint = signal.getsignal(signal.SIGINT)
	# restore the original signal handler as otherwise evil things will happen
	# in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
	signal.signal(signal.SIGINT, original_sigint)

	try:
		if raw_input("\nReally quit? (y/n)> ").lower().startswith('y'):

			disarmController()
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
		autopilotPort.write("Arm")
		armed=1
		print("Arming...")
		
def disarmController():
	global autopilotPort, armed
	if(armed==1):
		autopilotPort.write("Disarm")	
		armed=0	

def main():

	global autopilotPort
	
	rospy.init_node('interface')
	
	signal.signal(signal.SIGINT, exit_gracefully)

	serialDevices=glob.glob('/dev/tty*')
		
	found=False
	
	for serialDevice in serialDevices:
		print(serialDevice)
		try:
			autopilotPort=serial.Serial(serialDevice, 115200, timeout=0.5)
		except:
			continue
		autopilotPort.write("Are you the autopilot?")
		print("Are you the autopilot?")
		time.sleep(1)
		a=autopilotPort.readline().rstrip('\n')[0:-1]
		print(a)
		test="Yes"
		neq=False
		if(len(a)==len(test)):
			for i in range(0,len(a)):
					if(a[i]!=test[i]):
						neq=True
		if(not neq):
			found=True
			break
	
	if(not found):
		autopilotPort.close()
		sys.exit(1)
	
	armController()
	while(not autopilotPort.inWaiting()):
		continue
	
	while(running):
		if(autopilotPort.inWaiting()>0):
			Terminal_Output=autopilotPort.readline()
			
			for entry in Terminal_Output[0:-2].split(', '):
				Id, value = entry.split(': ')
				if(Id == "Heart Beat"):
					 print("Heart Beat:")
					 print(float(value))
					 pubAlive.publish(float(value))
				if(Id == "Speed"):
					 print("Speed:")
					 print(float(value))
					 pubSpeed.publish(float(value))
				if(Id == "Desired Speed"):
					 print("Desired Speed:")
					 print(float(value))
					 pubTargetSpeed.publish(float(value))
				if(Id == "Error"):
					 print("Error:")
					 print(float(value))
					 pubError.publish(float(value))
	
if __name__ == "__main__": 
	main()
