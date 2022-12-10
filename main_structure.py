# import pandas lib as pd
import numpy as np
import pandas as pd


class acceleration():
    x = None
    y = None
    z = None

class gyro():
    roll = None
    pitch = None
    yaw = None

class out_put():
    roll = None
    pitch = None
    yaw = None
    thrust = None


def relevant_columns():
    fields = ['roll', 'pitch']

    df = pd.read_csv('2022-11-27 15-13-48 vehicle1.csv', skipinitialspace=True, usecols=fields)
    #print(df.roll)
    #print(df.pitch)
    print(df)
    mat = np.matrix(df)
    with open('relevant_columns', 'wb') as f:
        for line in mat:
            np.savetxt(f, line, fmt='%.5f')

def determine_directory(acceleration,gyro): 
    if(acceleration.x == 0 and acceleration.y == 0 and acceleration.z == 0 and gyro.roll == 0 and gyro.pitch == 0 and gyro.yaw == 0): # no motion
        out_put.thrust= m*g/f_max
        out_put.roll = 0
        out_put.pitch = 0
        out_put.yaw = 0 
        return out_put        
    elif (acceleration.x == 0 and acceleration.y == 0 and acceleration.z != 0): # motion in z axis
        if(gyro.roll != 0): # if there is roll rotaion
            f_t = abs((m*acceleration.z+m*g)/(np.cos(gyro.roll)))
            out_put.thrust = f_t/f_max
            out_put.roll = np.arccos((Ixx*1)/(0.4*f_t*l1 - 0.6*f_t*l1)) * 180/np.pi # rad
            out_put.pitch = 0
            out_put.yaw = 0
            return out_put  
        elif(gyro.pitch != 0): # if there is pitch rotaion  
            f_t = abs(m*acceleration.z+m*g)
            out_put.thrust = f_t/f_max
            out_put.roll = 0
            out_put.pitch = 0
            out_put.yaw = 0
            return out_put
        elif(gyro.roll != 0): # if there is yaw rotaion
            f_t = abs(m*acceleration.z+m*g)
            out_put.thrust = f_t/f_max
            out_put.roll = 0
            out_put.pitch = 0
            out_put.yaw = 0
            return out_put       

    """elif (acceleration.x != 0 and acceleration.y != 0 and acceleration.z == 0 and gyro.roll != 0 and gyro.pitch != 0 and gyro.yaw == 0):
        if (gyro.yaw > 0):
            thrust = 6
            roll = 0
            pitch = 0
            yaw = 0
        else:
            thrust = 6
            roll = 0
            pitch = 0
            yaw = 0  """
    
    
    """elif age < 0:
        print("You're yet to be born")
    elif age >= 18:
        print("You are allowed to party")
    else: 
        "You're too young to party"""

if __name__ == "__main__":
    m = 3.8
    g = 9.81
    RPM_max = 1201.34
    f_max = 1  
    Ixx =  0.1
    Iyy = 0.1
    Izz = 0.5
    l1 = 0.39
    #relevant_columns()
    #acc_directory()
    acceleration.x = 0
    acceleration.y = 0
    acceleration.z = 5
    gyro.roll = np.pi/6
    gyro.pitch = 0 
    gyro.yaw = 0
    a = determine_directory(acceleration, gyro)
    print('thrust: ' + str(a.thrust),'\t''roll: ' + str(a.roll), '\t''pitch: ' + str(a.pitch), '\t''yaw: ' + str(a.yaw))