"""line_follow controller."""
from controller import Robot, DistanceSensor, Motor, LED

TIME_STEP = 64
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

def pillar_counting(TIME_STEP, MAX_SPEED, robot):

    ir=[]
    for i in range(8):
        ir.append(robot.getDevice("ir"+str(i)))
        ir[i].enable(TIME_STEP)
    
    wheels = []
    wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
    for i in range(4):
        wheels.append(robot.getDevice(wheelsNames[i]))
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(0.4*MAX_SPEED)
    
    ds = []
    dsNames = ['ds_right', 'ds_left']
    for i in range(2):
        ds.append(robot.getDevice(dsNames[i]))
        ds[i].enable(TIME_STEP)
        
    ts = []
    tsNames = ['ts0', 'ts1']
    for i in range(2):
        ts.append(robot.getDevice(tsNames[i]))
        ts[i].enable(TIME_STEP)
    
    p=0
    i=0
    d=0
    kp=0.2
    ki=0.02
    kd=0.002
    last_error=0
    
    count = 0
    s = 0
    led = robot.getDevice('led2')
    ps=0
    cs=0
    pj=0
    turn = 0
    pc=1
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(TIME_STEP) != -1:
        #ir_values=[]
        #ir_sum=0
        
        led.set(0)
        
        #for i in range(8):
            #ir_val=ir[i].getValue()
            #ir_values.append(ir_val)
            #ir_sum+=ir_val
        
        #sd=0
        #for i in range(8):
            #sd+=(ir_values[i]-ir_sum/8)**2
        #sd=(sd/8)**0.5
        
        #for i in range(8):
            #ir_values[i]=(ir_values[i]-(ir_sum/8))/(sd+0.0001)
        
        #pos=0
        #for i in range(4):
            #pos+=ir_values[i]*(-i+4)+ir_values[7-i]*(-4+i)
        #error=0.0-pos
        #p=error
        #i+=p
        #d=error-last_error
        #last_error=error
        #motor_speed=kp*p+ki*i+kd*d
        
        #counting pillars
        for j in range(2):
            if 850.0<ds[j].getValue()<900.0:
                pj=j
                cs=1
                if ps==0:
                    count += 1 
                    led.set(1)
                    ps=cs    
                    print('No. of pillars:',count)
                else:
                    led.set(0)
                
            else:
                #wheels[0].setVelocity(0.4*MAX_SPEED-motor_speed)
                #wheels[1].setVelocity(0.4*MAX_SPEED+motor_speed)
                #wheels[2].setVelocity(0.4*MAX_SPEED-motor_speed)
                #wheels[3].setVelocity(0.4*MAX_SPEED+motor_speed)
                if pj==j:
                    ps=0
                    cs=0
            
            #print(ds[j].getValue(),j)
            #print('No. of pillars:',count)
            
            #turn back
            if ts[j].getValue() < 400:
                if count%2 == 1:
                    turn = 1
                    pc = 0
                
        pass
        
    return (turn, pc)

turn, pc = pillar_counting(TIME_STEP, MAX_SPEED, robot)
