import serial
import serial.tools.list_ports
import signal
import sys
import rospy
from std_msgs.msg import Float32, Bool

from Header import makeHeader

running = True

serialPort=serial.Serial()
autopilotPort=serialPort

armed=0

throttle=0
steering=0

estop=0
pos=[0,0,0]

def subThrottle(msg):
	throttle=msg.data
	autopilotPort.write(makeHeader(throttle, steering, pos, estop))
def subSteering(msg):
	steering=msg.data
	autopilotPort.write(makeHeader(throttle, steering, pos, estop))
def subArmed(msg):
	if(msg.data):
		armController()
	else:
		disarmController()

subThrottle=rospy.Subscriber("/autonomous_rc/Throttle", Float32, subThrottle)
subSteering=rospy.Subscriber("/autonomous_rc/Steering", Float32, subSteering)
subArmed=rospy.Subscriber("/autonomous_rc/Armed", Bool, subArmed)

pubAlive=rospy.Publisher("/autonomous_rc/Alive", Bool, queue_size = 1)


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
    
    rospy.init_node('interface')
	
    signal.signal(signal.SIGINT, exit_gracefully)

    serialDevices=serial.tools.list_ports.comports()
    
    for serialDevice in serialDevices:
    	global autopilotPort
    	try:
    	    autopilotPort=serial.Serial(serialDevice.device, 115200, timeout=1)
    	except:
    	    continue
    	autopilotPort.write("Are you the autopilot?\n")
    	
    	if(autopilotPort.readline()=="Yes"):
    		break
    		
    print(autopilotPort.port)
    
    armController()
    while(not autopilotPort.inWaiting()):
    	continue
    while(running):
    	if(autopilotPort.inWaiting()>0):
	    	print(autopilotPort.readline())
    
if __name__ == "__main__": 
    main()
