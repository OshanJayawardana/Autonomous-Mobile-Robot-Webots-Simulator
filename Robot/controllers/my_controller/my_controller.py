from controller import Robot

TIME_STEP = 64
robot = Robot()

wheels = []
wheelsNames = ['wheel_left', 'wheel_right']
for i in range(2):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
avoidObstacleCounter = 0
leftSpeed = 1.0
rightSpeed = 1.0
while robot.step(TIME_STEP) != -1:
    #if leftSpeed < 10:
        #leftSpeed += 1
        #rightSpeed += 1
    
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)