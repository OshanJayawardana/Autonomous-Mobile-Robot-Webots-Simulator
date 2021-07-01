
from controller import Robot,DistanceSensor


if __name__ == "__main__":
    
    
    robot = Robot()
    sensor = DistanceSensor('sensor1')
    timestep =64
    max_speed = 6.28
    sensor.enable(10)
    left_motor = robot.getMotor('MOTOR1')
    right_motor = robot.getMotor('MOTOR2')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
   
    while robot.step(timestep) != -1:
        sensorval =sensor.getValue()
        print(sensorval)
        if(sensorval<100):
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            sensorval =sensor.getValue()
            continue
        left_speed = 1*max_speed
        right_speed = 1*max_speed
        
        
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        

