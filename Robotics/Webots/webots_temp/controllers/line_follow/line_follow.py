"""line_follow controller."""
from controller import Robot, DistanceSensor, Motor

TIME_STEP = 64
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

ir=[]
for i in range(8):
    ir.append(robot.getDevice("ir"+str(i)))
    ir[i].enable(TIME_STEP)
    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.1*MAX_SPEED)
rightMotor.setVelocity(0.1*MAX_SPEED)
p=0
i=0
d=0
kp=0.15
ki=0.001
kd=0.0001
last_error=0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    ir_values=[]
    ir_sum=0
    
    for i in range(8):
        ir_val=ir[i].getValue()
        ir_values.append(ir_val)
        ir_sum+=ir_val
    
    sd=0
    for i in range(8):
        sd+=(ir_values[i]-ir_sum/8)**2
    sd=(sd/8)**0.5
    #print(sd)   
    for i in range(8):
        ir_values[i]=(ir_values[i]-(ir_sum/8))/(sd+0.0001)
    print(ir_values)
    
    pos=0
    for i in range(4):
        pos+=ir_values[i]*(i-4)+ir_values[7-i]*(4-i)
        #pos+=ir_values[i]*(-1)+ir_values[7-i]*(1)
    print("pos="+str(pos))
    error=0.0-pos
    print("error="+str(error))
    p=error
    i+=p
    d=error-last_error
    last_error=error
    motor_speed=kp*p+ki*i+kd*d
    
    leftMotor.setVelocity(0.5*MAX_SPEED-motor_speed)
    rightMotor.setVelocity(0.5*MAX_SPEED+motor_speed)
    pass
# Enter here exit cleanup code.
