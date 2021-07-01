from controller import Robot
from time import sleep,time

if __name__ == "__main__":
    robot2 = Robot()
    timestep =64
    max_speed = 10
    gate_motor = robot2.getMotor('gate_motor')
    gate_open = False
    start_time = robot2.getTime()
    
   
    while robot2.step(timestep) != -1:
        current_time = robot2.getTime()
        
        if((current_time-start_time)>20.0):
            start_time = robot2.getTime()
            current_time = robot2.getTime()
        print("time",current_time)
        if(6<(current_time-start_time)< 13 and not(gate_open)):
            gate_motor.setPosition(-1.57)
            gate_motor.setVelocity(1*max_speed)
            gate_open = True
        
        if ((current_time-start_time)>13 and gate_open ): 
            gate_motor.setPosition(0)
            gate_motor.setVelocity(1*max_speed)
            gate_open = False
