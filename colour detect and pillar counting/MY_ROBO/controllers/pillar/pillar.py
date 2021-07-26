from controller import Robot,LED

TIME_STEP = 64
robot = Robot()
ds = []
dsNames = ['ds_right', 'ds_left']

for i in range(2):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)
       
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

count = 0
s = 0
led = robot.getDevice('led1')
led.set(0)

while robot.step(TIME_STEP) != -1:
    leftSpeed = 1.5
    rightSpeed = 1.5
           
    for i in range(2):
        if ds[i].getValue() < 950.0:
            if s != round(ds[i].getValue(),1):
                count += 1 
                #value = 1
                led.set(1)
                s = round(ds[i].getValue(),1)
                #break
            else:
                led.set(0)

        else:
            wheels[0].setVelocity(leftSpeed)
            wheels[1].setVelocity(rightSpeed)
            wheels[2].setVelocity(leftSpeed)
            wheels[3].setVelocity(rightSpeed)
            
        print(ds[i].getValue(),i)
        print('No. of pillars:',count) 
        
            
    
        
    