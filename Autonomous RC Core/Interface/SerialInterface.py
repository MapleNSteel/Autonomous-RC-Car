import serial
import serial.tools.list_ports
import signal
import sys
import rospy

import Header

serialPort=serial.Serial()
autopilotPort=serialPort

armed=0

def exit_gracefully(signum, frame):
    original_sigint = signal.getsignal(signal.SIGINT)
    # restore the original signal handler as otherwise evil things will happen
    # in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
    signal.signal(signal.SIGINT, original_sigint)

    try:
        if raw_input("\nReally quit? (y/n)> ").lower().startswith('y'):
            sys.exit(1)

    except KeyboardInterrupt:
        print("Ok ok, quitting")
        sys.exit(1)

	disarmController()
    # restore the exit gracefully handler here    
    signal.signal(signal.SIGINT, exit_gracefully)

def armController():
	if(armed==0):
	    autopilotPort.write("Arm")
	    
def disarmController():
	if(armed==1):
	    autopilotPort.write("Disarm")		

def main():
    
    rospy.init_node('USBControl')
	
    signal.signal(signal.SIGINT, exit_gracefully)

    serialDevices=serial.tools.list_ports.comports()
    
    for serialDevice in serialDevices:
    	global autopilotPort
    	try:
    	    autopilotPort=serial.Serial(serialDevice.device, 1152200, timeout=1)
    	except:
    	    continue
    	autopilotPort.write("Are you the autopilot?\n")
    	
    	if(autopilotPort.readline()=="Yes"):
    		armed=1
    		break
    		
    print(autopilotPort.port)
    
    armController()
    
if __name__ == "__main__": 
    main()
