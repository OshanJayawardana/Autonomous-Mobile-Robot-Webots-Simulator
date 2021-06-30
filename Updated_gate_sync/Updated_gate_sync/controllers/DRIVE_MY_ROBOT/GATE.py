
from controller import Robot,DistanceSensor


if __name__ == "__main__":
    
    
    robot = Robot()
    sensor = DistanceSensor('sensor1')
    timestep =64
    max_speed = 2*6.28
    print(sensor.getType())
    gate_motor = robot.getMotor('gate_motor')
    
    
    left_motor.setPosition(1.57)
    left_motor.setVelocity(1*max_speed)
    
   
        

