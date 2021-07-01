from controller import Robot

def run_robot(robot):
    time_step = 32
    max_speed = 6.28
    
    front_left_motor = robot.getMotor("front_left_motor")
    front_right_motor = robot.getMotor("front_right_motor")
    back_left_motor = robot.getMotor("back_left_motor")
    back_right_motor = robot.getMotor("back_right_motor")
    
    front_left_motor.setPosition(float("inf"))
    front_right_motor.setPosition(float("inf"))
    back_left_motor.setPosition(float("inf"))
    back_right_motor.setPosition(float("inf"))
    
    front_left_motor.setVelocity(0.0)
    front_right_motor.setVelocity(0.0)
    back_left_motor.setVelocity(0.0)
    back_right_motor.setVelocity(0.0)
    
    ir=[]
    for i in range(8):
        ir.append(robot.getDevice("ir"+str(i)))
        ir[i].enable(time_step)
        
    p=0
    i=0
    d=0
    kp=0.15
    ki=0.001
    kd=0.0001
    last_error=0
    
    left_ds = robot.getDistanceSensor("ds_left")
    left_ds.enable(time_step)
    
    right_ds = robot.getDistanceSensor("ds_right")
    right_ds.enable(time_step)
    
    while robot.step(time_step) != -1:
    
        left_speed = -max_speed * 0.5
        right_speed = -max_speed * 0.5
    
        ir_values=[]
        jun_values=[]
        ir_sum=0
        
        for j in range(8):
            ir_val=ir[j].getValue()
            ir_values.append(ir_val)
            jun_values.append(ir_val)
            ir_sum+=ir_val
        

        sd=0
        for j in range(8):
            sd+=(ir_values[j]-ir_sum/8)**2
        sd=(sd/8)**0.5
        #print(sd)   
        for j in range(8):
            ir_values[j]=(ir_values[j]-(ir_sum/8))/(sd+0.0001)
        print(ir_values)
        
        pos=0
        for j in range(4):
            pos+=ir_values[j]*(j-4)+ir_values[7-j]*(4-j)
            #pos+=ir_values[i]*(-1)+ir_values[7-i]*(1)
        #print("pos="+str(pos))
        error=0.0-pos
        p=error
        print("p: "+str(p))
        #print("i: "+str(i))
        i+=p
        if i > 100:
            i = 100
        elif i < -100:
            i = -100
        print("i: "+str(i))
        d=error-last_error
        last_error=error
        motor_speed=kp*p+ki*i+kd*d
        print("motor speed: "+str(motor_speed))
        
        left_speed = -0.5*max_speed-motor_speed
        right_speed = -0.5*max_speed+motor_speed
        
        ##############################
        for j in range(8):
            if jun_values[j] > 400:
                break
        else:
            print("hey")
            left_speed = -max_speed * 0.5
            right_speed = -max_speed * 0.5
       
        ##############################
        left_ds_value = left_ds.getValue()
        right_ds_value = right_ds.getValue()
             
        print("left ds: {} right ds: {}".format(left_ds_value, right_ds_value))
        
        left_wall = left_ds_value < 1000
        right_wall = right_ds_value < 1000
        
        if left_wall:
            if left_ds_value > 400:
                print("turn left")
                left_speed = max_speed * 0.5
                right_speed = -max_speed * 0.5
            elif left_ds_value < 300:
                print("turn right")
                right_speed = max_speed * 0.5
                left_speed = -max_speed * 0.5
        
        elif right_wall:
            if right_ds_value > 400:
                print("turn right")
                right_speed = max_speed * 0.5
                left_speed = -max_speed * 0.5
            elif right_ds_value < 300:
                print("turn left")
                left_speed = max_speed * 0.5
                right_speed = -max_speed * 0.5
            
        
        front_left_motor.setVelocity(left_speed)
        back_left_motor.setVelocity(left_speed)
        front_right_motor.setVelocity(right_speed)
        back_right_motor.setVelocity(right_speed)
    
    
    
if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
    run_robot(my_robot)