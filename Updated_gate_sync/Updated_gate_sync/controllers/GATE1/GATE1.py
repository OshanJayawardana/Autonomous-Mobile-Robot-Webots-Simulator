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
        if((current_time-start_time) < 10 and not(gate_open)):
            gate_motor.setPosition(-1.57)
            gate_motor.setVelocity(1*max_speed)
            gate_open = True
        
        if ((current_time-start_time)>10 and gate_open ): 
            gate_motor.setPosition(0)
            gate_motor.setVelocity(1*max_speed)
            gate_open = False
    """
        current_time = robot2.getTime()
        temp_time = current_time-start_time
        if(temp_time>=20):
            start_time = robot2.getTime()
            current_time = robot2.getTime()
            print("Time updated!")
            continue
        if(temp_time<10 and not(open_gate)):
            gate_motor.setPosition(-1.57)
            gate_motor.setVelocity(1*max_speed)
            gate_motor.setPosition(1.57)
            gate_motor.setVelocity(-1*max_speed)
            open_gate = True
            print("Gate opened!")
        elif(temp_time<20 and temp_time>=10 and gate_open):
            gate_motor.setPosition(1.57)
            gate_motor.setVelocity(-1*max_speed)
            open_gate = True
            print("Gate Closed!")
        """
        
 
        
        
        
        
    
    
    
    
   