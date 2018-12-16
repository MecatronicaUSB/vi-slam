import csv
import numpy as np
import matplotlib.pyplot as plt
import math

def quaternion2RPY(quaternion):
    rpy = np.zeros([1, 3])
    qx = quaternion[0]
    qy = quaternion[1]
    qz = quaternion[2]
    qw = quaternion[3]
    # roll (x-axis rotation)
    sinr_cosp = +2.0 * (qw * qx + qy * qz)
    cosr_cosp = +1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = +2.0 * (qw * qy - qz * qx)
    if math.fabs(sinp) >= 1:
        pitch = math.copysign(math.pi/ 2, sinp)# use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = +2.0 * (qw* qz + qx*qy)
    cosy_cosp = +1.0 - 2.0 * (qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
  
    rpy[0][0] = roll
    rpy[0][1] = pitch
    rpy[0][2] = yaw
    return rpy


def main():

    estPosition = np.zeros([0, 3])
    gtPosition = np.zeros([0, 3])
    estOrientationQ = np.zeros([0, 4])
    gtOrientationQ = np.zeros([0, 4])
    estOrientationRPY = np.zeros([0, 3])
    gtOrientationRPY = np.zeros([0, 3])
    Fs = 20 # Frecuencia de la camara
    #Fs = 0.5*10**-3
    Ts = 1.0/Fs # intervalo de tiempo
   


    with open('/home/lujano/Documents/outputVISlam.csv', newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            estPosition = np.append( estPosition, [[float(row[0]), float(row[1]), float(row[2])]], 0)
            estOrientationQ = np.append( estOrientationQ, [[float(row[3]), float(row[4]), float(row[5]), float(row[6])]], 0)
            gtPosition = np.append( gtPosition, [[float(row[7]), float(row[8]), float(row[9])]], 0)
            gtOrientationQ = np.append( gtOrientationQ, [[float(row[10]), float(row[11]), float(row[12]), float(row[13])]], 0)

        
    for quaternion in  estOrientationQ :
        estOrientationRPY = np.append( estOrientationRPY, quaternion2RPY(quaternion), 0)
    for quaternion in  gtOrientationQ :
        gtOrientationRPY = np.append( gtOrientationRPY, quaternion2RPY(quaternion), 0)
   
     
    rest = np.full_like(estPosition[:, 0], 1)
    estPosition[:, 0] = estPosition[:, 0] - estPosition[0, 0]*rest 
    estPosition[:, 1] = estPosition[:, 1] - estPosition[0, 1]*rest 
    estPosition[:, 2] = estPosition[:, 2] - estPosition[0, 2]*rest 

    estPosition[:, 0] = estPosition[:, 0]*20 + gtPosition[0, 0]*rest 
    estPosition[:, 1] = estPosition[:, 1]*20 + gtPosition[0, 1]*rest 
    estPosition[:, 2] = estPosition[:, 2]*5 + gtPosition[0, 2]*rest 

            
    time = np.arange(0, Ts*(estPosition[:, 0].size), Ts )


    # Plot position
    plt.figure()

    plt.subplot(3, 1, 1)
    plt.plot(time, estPosition[:, 0], 'b-', linewidth=2, label='Posicion x estimada')
    plt.plot(time, gtPosition[:, 0], 'r-', linewidth=2, label='Posicion x gt')
    plt.ylabel("x(m)")
    plt.xlabel("t(s)")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(time, estPosition[:, 1], 'b-', linewidth=2, label='Posicion y estimada')
    plt.plot(time, gtPosition[:, 1], 'r-', linewidth=2, label='Posicion y gt')
    plt.ylabel("y(m)")
    plt.xlabel("t(s)")
    plt.legend()
    
    plt.subplot(3, 1, 3)
    plt.plot(time, estPosition[:, 2], 'b-', linewidth=2, label='Posicion z estimada')
    plt.plot(time, gtPosition[:, 2], 'r-', linewidth=2, label='Posicion z gt')
    plt.ylabel("z(m)")
    plt.xlabel("t(s)")
    plt.legend()


    # Plot orientatio (RPY)
    plt.figure()

    plt.subplot(3, 1, 1)
    plt.plot(time, estOrientationRPY[:, 0]*180/math.pi, 'b-', linewidth=2, label='Roll estimado')
    plt.plot(time, gtOrientationRPY[:, 0]*180/math.pi, 'r-', linewidth=2, label='Roll gt')
    plt.ylabel("roll(°)")
    plt.xlabel("t(s)")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(time, estOrientationRPY[:, 1]*180/math.pi, 'b-', linewidth=2, label='Pitch estimado')
    plt.plot(time, gtOrientationRPY[:, 1]*180/math.pi, 'r-', linewidth=2, label='Pitch gt')
    plt.ylabel("pitch(°)")
    plt.xlabel("t(s)")
    plt.legend()
    
    plt.subplot(3, 1, 3)
    plt.plot(time, estOrientationRPY[:, 2]*180/math.pi, 'b-', linewidth=2, label='Yaw estimado')
    plt.plot(time, gtOrientationRPY[:, 2]*180/math.pi, 'r-', linewidth=2, label='Yaw gt')
    plt.ylabel("yaw(°)")
    plt.xlabel("t(s)")
    plt.legend()




    



    plt.show()
    



if __name__ == "__main__": main()
