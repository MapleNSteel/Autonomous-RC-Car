def makeHeader(throttle, steering, position, estop):
	return "T:"+str(throttle)+",S:"+str(steering)+",P:"+str(position[0])+";"+str(position[1])+";"+str(position[2])+",E:"+str(estop)+"\n"
