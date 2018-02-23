def makeHeader(throttle, steering, position, estop, Kp=0, Ki=0, Kd=0):
    if(Kp==Ki==Kd==0):
    	return "T:"+str(throttle)+",S:"+str(steering)+",P:"+str(position[0])+";"+str(position[1])+";"+str(position[2])+",E:"+str(estop)+"\n"
    else:
        return "T:"+str(throttle)+",S:"+str(steering)+",P:"+str(position[0])+";"+str(position[1])+";"+str(position[2])+",E:"+str(estop)+",Kp:"+str(Kp)+",Ki:"+str(Ki)+",Kd:"+str(Kd)+"\n"
