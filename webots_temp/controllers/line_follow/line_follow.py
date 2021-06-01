"""line_follow controller."""
from controller import Brake, Robot, DistanceSensor, Motor, PositionSensor

TIME_STEP = 64
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

#functions
######################################################
#pid function
def pid(p,i,d,last_error):
    print("pid")
    #kp=0.14
    #ki=0.001
    #kd=0.0001
    kp=0.14
    ki=0.001
    kd=0.0001
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

    for i in range(8):
        ir_values[i]=(ir_values[i]-(ir_sum/8))/(sd+0.0001)
    
    pos=0
    for i in range(4):
        #pos+=ir_values[i]*(i-4)+ir_values[7-i]*(4-i)
        pos+=ir_values[i]*(-i+4)+ir_values[7-i]*(-4+i)
        #pos+=ir_values[i]*(-1)+ir_values[7-i]*(1)

    error=0.0-pos
    p=error
    i+=p
    d=error-last_error
    last_error=error
    motor_speed=kp*p+ki*i+kd*d
    left_speed=(0.5*MAX_SPEED-motor_speed)
    right_speed=(0.5*MAX_SPEED+motor_speed)
    return p,i,d,last_error,left_speed,right_speed
#....................................................
#junction identify
def juncFind():
    ir_left=ts[0].getValue()
    ir_right=ts[1].getValue()
    left=ir_left<400
    right=ir_right<400
    if left and not(right):
        junc=0
    elif left and right:
        junc=1
    elif not(left) and right:
        junc=2
    else:
        junc=-1
    return junc
#....................................................
#set motors
def setMotors(left_speed,right_speed,dc):
    #setting motor speeds    
    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)
    
    #front_leftMotor.setVelocity(left_speed)
    #front_rightMotor.setVelocity(right_speed)
    #setting brakes
    left_brk.setDampingConstant(dc)
    right_brk.setDampingConstant(dc)
    
    #front_left_brk.setDampingConstant(dc)
    #front_right_brk.setDampingConstant(dc)
    #storing the speed for next loop
    last_left_speed=left_speed
    last_right_speed=right_speed
    return last_left_speed,last_right_speed
    
#....................................................
#sharpTurn
def sharpTurn(turn,pos_lst,junc,direct_count,turn_command):
    if turn==2:
        pos_val=pos_left.getValue()
    else:
        pos_val=pos_right.getValue()
    if abs(pos_val)>0:
        pos_lst.append(pos_val)
        print(abs(pos_lst[0]-pos_lst[-1]))
        if abs(pos_lst[0]-pos_lst[-1])>2.9:
            junc=-1
            pos_lst=[]
            turn_command=0
            #pos_left.disable()
            #pos_right.disable()
            direct_count+=1
    if turn==0:
        print("turning left")
        left_speed=-0.1*MAX_SPEED
        right_speed=0.5*MAX_SPEED
    elif turn==1:
        left_speed=0.5*MAX_SPEED
        right_speed=0.5*MAX_SPEED
        print("going forward")
    elif turn==2:
        left_speed=0.5*MAX_SPEED
        right_speed=-0.1*MAX_SPEED
        print("turning right")
    else:
        print("wrong input. enter a value from 0-2")
    return pos_lst,junc,left_speed,right_speed,direct_count,turn_command

#....................................................
#braking
def brakes(last_left_speed,last_right_speed,dc,turn_command):
    print("brake")
    speed=max([last_left_speed,last_right_speed])
    if speed<0.5*MAX_SPEED:
        left_speed=(0)
        right_speed=(0)
        led.set(1)
        turn_command=1
        dc=0
    else:
        left_speed=speed-0.09
        right_speed=speed-0.09
        dc+=0
    return left_speed,right_speed,dc,turn_command
######################################################

#Initialization
######################################################
#ir panel
ir=[]
for i in range(8):
    ir.append(robot.getDevice("ir"+str(i)))
    ir[i].enable(TIME_STEP)
#....................................................

#two irs to detect junctions
ts=[]
for i in range(2):
    ts.append(robot.getDevice("ts"+str(i)))
    ts[i].enable(TIME_STEP)
#....................................................

#brakes
left_brk=robot.getDevice("brake_left")
right_brk=robot.getDevice("brake_right")
left_brk.setDampingConstant(0)
right_brk.setDampingConstant(0)

#front_left_brk=robot.getDevice("front_left_brake")
#front_right_brk=robot.getDevice("front_right_brake")
#front_left_brk.setDampingConstant(0)
#front_right_brk.setDampingConstant(0)
#....................................................

#encorders
pos_left=robot.getDevice("pos_left")
pos_right=robot.getDevice("pos_right")
pos_left.enable(TIME_STEP)
pos_right.enable(TIME_STEP)
#....................................................

#led
led=robot.getDevice("led") 
#....................................................

#motors 
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.1*MAX_SPEED)
rightMotor.setVelocity(0.1*MAX_SPEED)

#front_leftMotor = robot.getDevice('front_left')
#front_rightMotor = robot.getDevice('front_right')
#front_leftMotor.setPosition(float('inf'))
#front_rightMotor.setPosition(float('inf'))
#front_leftMotor.setVelocity(0.1*MAX_SPEED)
#front_rightMotor.setVelocity(0.1*MAX_SPEED)
#....................................................

#initial pid values
p=0
i=0
d=0
last_error=0
#....................................................

#junction identifying parameters
dc=0
turn_command=0
pos_lst=[]
junc=-1
direct=[2,1,2,2,2,0,2,2,2,2,1,0]
direct_count=0
######################################################

# Main loop:
######################################################
while robot.step(TIME_STEP) != -1:
    
    if junc!=-1 and turn_command:#turning code
        pos_lst,junc,left_speed,right_speed,direct_count,turn_command=sharpTurn(direct[direct_count],pos_lst,junc,direct_count,turn_command)        
    elif junc!=-1:#braking to run when junction is found
        left_speed,right_speed,dc,turn_command=brakes(last_left_speed,last_right_speed,dc,turn_command)
    else:#code running when freely line following
        #pos_left.disable()
        led.set(0)
        dc=0#setting brakes to zero
        p,i,d,last_error,left_speed,right_speed=pid(p,i,d,last_error)#running pid
        #checking junction detecting sensors
        junc=juncFind()
        
    #setting motor values
    last_left_speed,last_right_speed = setMotors(left_speed,right_speed,dc)
    pass
######################################################