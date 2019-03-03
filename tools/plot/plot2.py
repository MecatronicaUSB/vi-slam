import csv
import numpy as np
import matplotlib.pyplot as plt
import math

Ts = 0.05*20

lastindex = int(50)

















def main():

    plot_est = [1, 0, 0, 1] # estimacion: pos, velocidad, aceleracion, orientacion
    plot_res = [0,0,0, 0] # residuales: pos, velocidad, aceleracion, orientacion
    plot_error = [0, 0, 0, 0]  #errores: pos, velocidad, aceleracion, orientacion
    plot_debug = [0, 0]      #debug residual de posicion proveniente de la velocidad

    time = np.array([])
    estPosition = np.zeros([0, 3])
    gtPosition = np.zeros([0, 3])
    estVelocity = np.zeros([0, 3])
    gtVelocity = np.zeros([0, 3])
    estAcc = np.zeros([0, 3])
    gtAcc= np.zeros([0, 3])
    estOrientationQ = np.zeros([0, 4])
    gtOrientationQ = np.zeros([0, 4])
    estOrientationRPY = np.zeros([0, 3])
    gtOrientationRPY = np.zeros([0, 3])
    Fs = 20 # Frecuencia de la camara
    #Fs = 0.5*10**-3
   
   
    #debug
    estAngVelocity= np.zeros([0, 3])

    maxTime = 142.0

    #time = np.arange(0, Ts*(estPosition[:, 0].size), Ts )
    found = False
    index = 0
    with open('/home/lujano/Documents/outputVISlam.csv', newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            time = np.append( time, [float(row[0])], 0)
            if found :
                if float(row[0])> maxTime:
                    found = True
                    lastindex = index
            estPosition = np.append( estPosition, [[float(row[1]), float(row[2]), float(row[3])]], 0)
            estVelocity = np.append( estVelocity, [[float(row[4]), float(row[5]), float(row[6])]], 0)
            estAcc = np.append( estAcc, [[float(row[7]), float(row[8]), float(row[9])]], 0)
            estOrientationQ = np.append( estOrientationQ, [[float(row[10]), float(row[11]), float(row[12]), float(row[13])]], 0)
            gtPosition = np.append( gtPosition, [[float(row[14]), float(row[15]), float(row[16])]], 0)
            gtVelocity = np.append( gtVelocity, [[float(row[17]), float(row[18]), float(row[19])]], 0)
            gtOrientationQ = np.append( gtOrientationQ, [[float(row[20]), float(row[21]), float(row[22]), float(row[23])]], 0)
            index = index+1
            #estAngVelocity = np.append( estAngVelocity, [[float(row[23]), float(row[24]), float(row[25])]], 0)

        
    for quaternion in  estOrientationQ :
        estOrientationRPY = np.append( estOrientationRPY, quaternion2RPY(quaternion), 0)
    for quaternion in  gtOrientationQ :
        gtOrientationRPY = np.append( gtOrientationRPY, quaternion2RPY(quaternion), 0)
   
    """
    rest = np.full_like(estPosition[:, 0], 1)
    estPosition[:, 0] = estPosition[:, 0] - estPosition[0, 0]*rest 
    estPosition[:, 1] = estPosition[:, 1] - estPosition[0, 1]*rest 
    estPosition[:, 2] = estPosition[:, 2] - estPosition[0, 2]*rest 

    estPosition[:, 0] = estPosition[:, 0] + gtPosition[0, 0]*rest 
    estPosition[:, 1] = estPosition[:, 1] + gtPosition[0, 1]*rest 
    estPosition[:, 2] = estPosition[:, 2] + gtPosition[0, 2]*rest 
    """
    gtAccx= fd(gtVelocity[:, 0], Ts)
    gtAccy = fd(gtVelocity[:, 1], Ts)
    gtAccz = fd(gtVelocity[:, 2], Ts)

    
    
            
  


    
    if plot_est[0] == 1:
        # Plot position
        label1 = ["Posicion x estimada", "x(m)", "Posicion x groundtruth"]
        label2 = ["Posicion y estimada", "y(m)", "Posicion y groundtruth"]
        label3 = ["Posicion z estimada", "z(m)", "Posicion z groundtruth"]
        plotTriple(estPosition, gtPosition, time, label1, label2, label3, maxTime)


    if plot_est[1] == 1:
        # Plot velocity
        label1 = ["Velocidad x estimada", "Vx(m/s)", "Velocidad x groundtruth"]
        label2 = ["Velocidad y estimada", "Vy(m/s)", "Velocidad y groundtruth"]
        label3 = ["Velocidad z estimada", "Vz(m/s)", "Velocidad z groundtruth"]
        plotTriple(estVelocity, gtVelocity, time, label1, label2, label3, maxTime)

    if plot_est[2] == 1:
        # Plot Acceleration
        label1 = ["Aceleracion x estimada", "Ax(m/s²)", "Aceleracion x groundtruth"]
        label2 = ["Aceleracion y estimada", "Ay(m/s²)", "Aceleracion y groundtruth"]
        label3 = ["Aceleracion z estimada", "Az(m/s²)", "Aceleracion z groundtruth"]
        plotTripleSeparado(estAcc[:,0], estAcc[:,1], estAcc[:,2],
            gtAccx, gtAccy, gtAccz, time, label1, label2, label3, maxTime)


    if plot_est[3] == 1:
        # Plot orientation (RPY)
        label1 = ["Roll estimado", "Roll(°)", "Roll groundtruth"]
        label2 = ["Pitch estimado", "Pitch(°)", "Pitch groundtruth"]
        label3 = ["Yaw estimado", "Yaw(°)", "Yaw groundtruth"]
        plotTripleSeparado(remap(estOrientationRPY[:, 0]*180/math.pi), estOrientationRPY[:, 1]*180/math.pi, estOrientationRPY[:, 2]*180/math.pi,
            remap(gtOrientationRPY[:, 0]*180/math.pi), gtOrientationRPY[:, 1]*180/math.pi, gtOrientationRPY[:, 2]*180/math.pi, time, label1, label2, label3, maxTime)

       
    if plot_res[0] == 1:
         # Plot residual position
        label1 = ["Residual Posicion x estimado", "Rx(m)", "Residual Posicion x groundtruth"]
        label2 = ["Residual Posicion y estimado", "Ry(m)", "Residual Posicion y groundtruth"]
        label3 = ["Residual Posicion z estimado", "Rz(m)", "Residual Posicion z groundtruth"]
        plotTripleSeparado(residual(estPosition[:,0]), residual(estPosition[:,1]), residual(estPosition[:,2]),
            residual(gtPosition[:, 0]), residual(gtPosition[:, 1]), residual(gtPosition[:, 2]), time, label1, label2, label3, maxTime)

    if plot_res[1] == 1:
        label1 = ["Residual Velocidad x estimado", "RVx(m)", "Residual Velocidad x groundtruth"]
        label2 = ["Residual Velocidad y estimado", "RVy(m)", "Residual Velocidad y groundtruth"]
        label3 = ["Residual Velocidad z estimado", "RVz(m)", "Residual Velocidad z groundtruth"]
        plotTripleSeparado(residual(estVelocity[:,0]), residual(estVelocity[:,1]), residual(estVelocity[:,2]),
            residual(gtVelocity[:, 0]), residual(gtVelocity[:, 1]), residual(gtVelocity[:, 2]), time, label1, label2, label3, maxTime)


    if plot_res[3] == 1:
        # Plot residual orientation (RPY)
        label1 = ["Residual Roll estimado", "Rroll(°)", "Residual Roll groundtruth"]
        label2 = ["Residual Pitch estimado", "Rpitch(°)", "Residual Pitch groundtruth"]
        label3 = ["Residual Yaw estimado", "Ryaw(°)", "Residual Yaw groundtruth"]
        residualRotationEst = 180.0/math.pi *residualRotation(estOrientationRPY[:, 0],  estOrientationRPY[:, 1], estOrientationRPY[:, 2])
        residualRotationGt = 180.0/math.pi *residualRotation(gtOrientationRPY[:, 0],  gtOrientationRPY[:, 1], gtOrientationRPY[:, 2])
        plotTriple(residualRotationEst, residualRotationGt, time, label1, label2, label3, maxTime)
        plotAndHist(residualRotationEst[1:, 0],residualRotationEst[1:, 1], residualRotationEst[1:, 2],
            time, label1, label2, label3, maxTime)
        
        label1 = ["Error Residual Roll", "ERroll(°)", ""]
        label2 = ["Error Residual Pitch", "ERpitch(°)", ""]
        label3 = ["Error Residual Yaw", "ERyaw(°)", ""]
        #print(errorP(residualRotationEst[1:, 0]-residualRotationGt[1:, 0], residualRotationGt[1:, 0]))
        #print(errorP(residualRotationEst[1:, 1]-residualRotationGt[1:, 1], residualRotationGt[1:, 1]))
        #print(errorP(residualRotationEst[1:, 2]-residualRotationGt[1:, 2], residualRotationGt[1:, 2]))
        plotAndHist(residualRotationEst[1:, 0]-residualRotationGt[1:, 0],residualRotationEst[1:, 1]-residualRotationGt[1:, 1], residualRotationEst[1:, 2]-residualRotationGt[1:, 2],
            time, label1, label2, label3, maxTime)
        
    # plot errors

    if plot_error[0] == 1:
        # Plot error de posicion
        label1 = ["Error Posicion x", "Ex(m)", ""]
        label2 = ["Error Posicion y", "Ey(m)", ""]
        label3 = ["Error Posicion z", "Ez(m)", ""]
        plot(estPosition[:, 0]-gtPosition[:, 0], estPosition[:, 1]-gtPosition[:, 1], estPosition[:, 2]-gtPosition[:, 2],
            time, label1, label2, label3, maxTime)

    if plot_error[1] == 1:
        # Plot error de velocidad
        label1 = ["Error Velocidad x", "EVx(m/s)", ""]
        label2 = ["Error Velocidad y", "EVy(m/s)", ""]
        label3 = ["Error Velocidad z", "EVz(m/s)", ""]
        plot(estVelocity[:, 0]-gtVelocity[:, 0], estVelocity[:, 1]-gtVelocity[:, 1], estVelocity[:, 2]-gtVelocity[:, 2],
            time, label1, label2, label3, maxTime)

    if plot_error[3] == 1:
        # Plot error de velocidad
        label1 = ["Error Roll", "Eroll(°)", ""]
        label2 = ["Error Pitch", "Epitch(°)", ""]
        label3 = ["Error Yaw", "Eyaw(°)", ""]
        plotAndHist(error((estOrientationRPY[:, 0]-gtOrientationRPY[:, 0])*180/np.pi),error( (estOrientationRPY[:, 1]-gtOrientationRPY[:, 1])*180/np.pi), error( (estOrientationRPY[:, 2]-gtOrientationRPY[:, 2])*180/np.pi),
            time, label1, label2, label3, maxTime)
        
        #plotHist(error((estOrientationRPY[:, 0]-gtOrientationRPY[:, 0])*180/np.pi),error( (estOrientationRPY[:, 1]-gtOrientationRPY[:, 1])*180/np.pi), error( (estOrientationRPY[:, 2]-gtOrientationRPY[:, 2])*180/np.pi),
        #   time, label1, label2, label3, maxTime)



    if plot_debug[0] == 1:
        # Plot residual de posicion proveniente de la integracion de la velocidad de la imu
        plt.figure()

        plt.subplot(3, 1, 1)
        plt.plot(time, residual(fi(estVelocity[:, 0], 0.05)), 'b-', linewidth=2, label=' Residual de Pose en x estimada')
        plt.plot(time, residual(gtPosition[:,0]), 'r-', linewidth=2, label='Residual de Pose en x gt')
        plt.ylabel("Rx(m/s²)")
        plt.xlabel("t(s)")
        plt.legend()

        plt.subplot(3, 1, 2)
        plt.plot(time, residual(fi(estVelocity[:, 1], 0.05)), 'b-', linewidth=2, label=' Residual de Pose en y estimada')
        plt.plot(time, residual(gtPosition[:,1]), 'r-', linewidth=2, label='Residual de Pose en y gt')
        plt.ylabel("Ry(m/s²)")
        plt.xlabel("t(s)")
        plt.legend()
        
        plt.subplot(3, 1, 3)
        plt.plot(time, residual(fi(estVelocity[:, 2], 0.05)), 'b-', linewidth=2, label=' Residual de Pose en z estimada')
        plt.plot(time, residual(gtPosition[:,2]), 'r-', linewidth=2, label='Residual de Pose en z gt')
        plt.ylabel("Rz(m/s²)")
        plt.xlabel("t(s)")
        plt.legend()


    if plot_debug[1] == 1:
        # Plot residual de posicion proveniente de la integracion de la velocidad de la imu
        plt.figure()

        positionx = fi(estVelocity[:, 0], 0.05)
        positiony = fi(estVelocity[:, 1], 0.05)
        positionz = fi(estVelocity[:, 2], 0.05)

        
        positionx = positionx - positionx[0]*rest +gtPosition[0,0]*rest
        positiony = positiony - positiony[0]*rest +gtPosition[0,1]*rest
        positionz = positionz - positionz[0]*rest +gtPosition[0,2]*rest

        plt.subplot(3, 1, 1)
        plt.plot(time, positionx , 'b-', linewidth=2, label='Pose integrada x estimada')
        plt.plot(time, gtPosition[:,0], 'r-', linewidth=2, label='Pose en x gt')
        plt.ylabel("Rx(m/s²)")
        plt.xlabel("t(s)")
        plt.legend()

        plt.subplot(3, 1, 2)
        plt.plot(time, positiony, 'b-', linewidth=2, label=' Pose integrada y estimada')
        plt.plot(time, gtPosition[:,1], 'r-', linewidth=2, label='Pose en y gt')
        plt.ylabel("Ry(m/s²)")
        plt.xlabel("t(s)")
        plt.legend()
        
        plt.subplot(3, 1, 3)
        plt.plot(time, positionz, 'b-', linewidth=2, label=' Pose integrada z estimada')
        plt.plot(time, gtPosition[:,2], 'r-', linewidth=2, label='Pose en z gt')
        plt.ylabel("Rz(m/s²)")
        plt.xlabel("t(s)")
        plt.legend()


    plt.gcf().canvas.mpl_connect('key_press_event', quit_figure)
    plt.show()

def errorP(error, div):
    remap_matrix = np.array([])
    for x in np.arange(error.size):
        error_value = 0.0
        if div[x]!= 0.0:
            error_value = error[x]/div[x]


        remap_matrix = np.append( remap_matrix, [error_value], 0)
    return np.sum(remap_matrix)/ error.size  

def error(error):
    remap_matrix = np.array([])
    for x in np.arange(error.size):
        error_value = error[x]
        if error[x]>180:
            error_value=error[x]-360
        if error[x]<-180:
            error_value=error[x]+360
        remap_matrix = np.append( remap_matrix, [error_value], 0)
    return remap_matrix 



       
def remap(angle):
    remap_matrix = np.array([])
    for x in np.arange(angle.size):
        angle_value  = angle[x] 

        if angle_value <0:
            angle_value = angle_value+360

        remap_matrix = np.append( remap_matrix, [angle_value], 0)
    return remap_matrix 



def remap3(angle):
    remap_matrix = np.array([])
    for x in np.arange(angle.size):
        angle_value  = angle[x] 

        angle_value = angle_value-360

        remap_matrix = np.append( remap_matrix, [angle_value], 0)
    return remap_matrix 
def remap2(angle):
    remap_matrix = np.array([])
    last_value = angle[0]
    for x in np.arange(angle.size):
        angle_value  = angle[x] 

        if abs(last_value-angle_value)>180:
            angle_value = angle_value+360

        remap_matrix = np.append( remap_matrix, [angle_value], 0)

        last_value = angle_value
    return remap_matrix 
        
def residual3(acc):
    residual_matrix = np.zeros([1, 3])
    rx = 0.0
    ry = 0.0
    rz = 0.0
    for x in np.arange(acc[:, 0].size):
        if  x != 0 :
            rx = acc[x, 0]-acc[x-1, 0]
            ry = acc[x ,1]-acc[x-1, 1]
            rz = acc[x ,2]-acc[x-1, 2]
            residual_matrix = np.append( residual_matrix, [[rx, ry, rz]], 0)
        else:
            residual_matrix = np.append( residual_matrix, [[rx, ry, rz]], 0)


        
    return residual_matrix

def residualAlterado(velocidad, tInicial, dt):
    residualesVelocidad = residual(velocidad)
    residual_matrix = np.array([])
    lastTraslation = tInicial
    for x in np.arange(velocidad.size):
        rx = lastTraslation+residualesVelocidad[x]*dt
        residual_matrix = np.append(residual_matrix, [rx])
        lastTraslation = rx
    return residual_matrix



def residual(acc):
    residual_matrix = np.array([])
    rx = 0.0
    for x in np.arange(acc.size):
        if  x != 0 :
            rx = acc[x]-acc[x-1]
            residual_matrix = np.append(residual_matrix, [rx])
        else:
            residual_matrix = np.append(residual_matrix, [rx])


        
    return residual_matrix


def residualRotation(Roll, Pitch, Yaw):
    residual_matrix = np.zeros([1, 3])
    rRoll = 0.0
    rPitch = 0.0
    rYaw = 0.0
    for x in np.arange(Roll.size):
        if  x != 0 :
            init_rotationMatrix = RPY2rotationMatrix(Roll[x-1], Pitch[x-1], Yaw[x-1])
            final_rotationMatrix = RPY2rotationMatrix(Roll[x], Pitch[x], Yaw[x])
            residual_rotationMatrix = np.dot(init_rotationMatrix.transpose() , final_rotationMatrix) # inverse rotation by final rotation
            residualRPY = rotationMatrix2RPY(residual_rotationMatrix)
            rRoll = residualRPY[0][0]
            rPitch = residualRPY[0][1]
            rYaw = residualRPY[0][2]
            residual_matrix = np.append( residual_matrix, [[rRoll, rPitch, rYaw]], 0)
        else:
            residual_matrix = np.append( residual_matrix, [[rRoll, rPitch, rYaw]], 0)



        
    return residual_matrix

def fd(acc, dt): # funcion para derivar una matriz
    derivate_matrix = np.array([])
    last_x = 0
    for x in acc:
        dx = (x-last_x)/dt
        derivate_matrix = np.append(derivate_matrix, [dx])
        last_x = x
        
    return derivate_matrix

def fi(acc, dt):
    integral_matrix =  np.array([])
    acc_sum = 0
    for x in acc:
        acc_sum = x*dt+acc_sum
        integrl_matrix = np.append(integral_matrix, [acc_sum])
        
    return integral_matrix
    
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

def plotTriple( estData, gtData, time, label1, label2, label3, maxTime):

    # Plot residual orientation (RPY)
    plt.figure()
    

    plt.subplot(3, 1, 1)
    plt.plot(time, estData[:,0], 'b-', linewidth=2, label=label1[0])
    plt.plot(time, gtData[:, 0], 'r-', linewidth=2, label=label1[2])
    plt.ylabel(label1[1])
    plt.legend()
    plt.xlim([0, maxTime])
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')


    plt.subplot(3, 1, 2)
    plt.plot(time, estData[:,1], 'b-', linewidth=2, label=label2[0])
    plt.plot(time, gtData[:,1], 'r-', linewidth=2, label=label2[2])
    plt.ylabel(label2[1])
    plt.xlim([0, maxTime])
    plt.legend()
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')

    
    plt.subplot(3, 1, 3)
    plt.plot(time, estData[:, 2], 'b-', linewidth=2, label=label3[0])
    plt.plot(time, gtData[:, 2], 'r-', linewidth=2, label=label3[2])
    plt.ylabel(label3[1])
    plt.xlabel("t(s)")
    plt.xlim([0, maxTime])
    plt.legend()
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')

    plt.subplots_adjust(left=0.07, bottom=0.07, right=0.7, top=0.98, wspace=0.2, hspace=0.12)

    return

def plotTripleSeparado( estData1, estData2, estData3, gtData1, gtData2, gtData3, time, label1, label2, label3, maxTime):

    # Plot residual orientation (RPY)
    plt.figure()
  
    plt.subplot(3, 1, 1)
    plt.plot(time, estData1, 'b-', linewidth=2, label=label1[0])
    plt.plot(time, gtData1, 'r-', linewidth=2, label=label1[2])
    plt.ylabel(label1[1])
    plt.xlabel("t(s)")
    plt.legend()
    plt.xlim([0, maxTime])
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')


    plt.subplot(3, 1, 2)
    plt.plot(time, estData2, 'b-', linewidth=2, label=label2[0])
    plt.plot(time, gtData2, 'r-', linewidth=2, label=label2[2])
    plt.ylabel(label2[1])
    plt.xlabel("t(s)")
    plt.xlim([0, maxTime])
    plt.legend()
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')

    
    plt.subplot(3, 1, 3)
    plt.plot(time, estData3, 'b-', linewidth=2, label=label3[0])
    plt.plot(time, gtData3, 'r-', linewidth=2, label=label3[2])
    plt.ylabel(label3[1])
    plt.xlabel("t(s)")
    plt.xlim([0, maxTime])
    plt.legend()
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')

    plt.subplots_adjust(left=0.07, bottom=0.07, right=0.7, top=0.98, wspace=0.2, hspace=0.12)

    return




def plot(Data1, Data2, Data3, time, label1, label2, label3, maxTime):

    # Plot residual orientation (RPY)
 
    plt.figure()

    plt.subplot(3, 1, 1)
    plt.plot(time, Data1, 'b-', linewidth=2, label=label1[0])
    plt.ylabel(label1[1])
    plt.xlabel("t(s)")
    plt.legend()
    plt.xlim([0, maxTime])
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')


    plt.subplot(3, 1, 2)
    plt.plot(time, Data2, 'b-', linewidth=2, label=label2[0])
    plt.ylabel(label2[1])
    plt.xlabel("t(s)")
    plt.legend()
    plt.xlim([0, maxTime])
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')

    
    plt.subplot(3, 1, 3)
    plt.plot(time, Data3, 'b-', linewidth=2, label=label3[0])
    plt.ylabel(label3[1])
    plt.xlabel("t(s)")
    plt.legend()
    plt.xlim([0, maxTime])
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')

    plt.subplots_adjust(left=0.07, bottom=0.07, right=0.7, top=0.98, wspace=0.2, hspace=0.12)

    return


def plotHist(Data1, Data2, Data3, time, label1, label2, label3, maxTime):

    # Plot residual orientation (RPY)

    plt.figure()

    plt.subplot(3, 1, 1)
    plt.hist(Data1[0:lastindex], bins = 30, density=True)
    plt.ylabel("Frecuencia")
    plt.xlabel(label1[1])
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')


    plt.subplot(3, 1, 2)
    plt.hist( Data2[0:lastindex],  bins =30, density=True )
    plt.ylabel("Frecuencia")
    plt.xlabel(label2[1])
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')

    
    plt.subplot(3, 1, 3)
    plt.hist(Data3[0:lastindex], bins=30, density=True)
    plt.ylabel("Frecuencia")
    plt.xlabel(label3[1])
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')

    plt.subplots_adjust(left=0.07, bottom=0.07, right=0.7, top=0.98, wspace=0.2, hspace=0.26)

    return



def plotAndHist(Data1, Data2, Data3, time, label1, label2, label3, maxTime):

    # Plot residual orientation (RPY)

    plt.figure()

    plt.subplot2grid( (3, 4), (0, 3))
    plt.hist(Data1[0:lastindex], 30, density=True, orientation="horizontal")
    plt.xlabel("Frecuencia")
    plt.ylabel(label1[1])
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')


    plt.subplot2grid( (3, 4), (1, 3))
    plt.hist( Data2[0:lastindex],  bins =30, density=True, orientation="horizontal" )
    plt.xlabel("Frecuencia")
    plt.ylabel(label2[1])
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')

    
    plt.subplot2grid( (3, 4), (2, 3))
    plt.hist(Data3[0:lastindex], bins=30, density=True, orientation="horizontal")
    plt.xlabel("Frecuencia")
    plt.ylabel(label3[1])
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')




    plt.subplot2grid( (3, 4), (0, 0), colspan=3)
    plt.plot(time, Data1, 'b-', linewidth=2, label=label1[0])
    plt.ylabel(label1[1])
    plt.xlabel("t(s)")
    plt.legend()
    plt.xlim([0, maxTime])
    ymax = np.max(Data1[0:lastindex])
    ymin = np.min(Data1[0:lastindex])
    plt.ylim([ymin*(1+0.1), ymax*(1+0.1)])
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')


    plt.subplot2grid( (3, 4), (1, 0), colspan=3)
    plt.plot(time, Data2, 'b-', linewidth=2, label=label2[0])
    plt.ylabel(label2[1])
    plt.xlabel("t(s)")
    plt.legend()
    ymax = np.max(Data2[0:lastindex])
    ymin = np.min(Data2[0:lastindex])
    plt.ylim([ymin*(1+0.1), ymax*(1+0.1)])
    plt.xlim([0, maxTime])
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')

    
    plt.subplot2grid( (3, 4), (2, 0), colspan=3)
    plt.plot(time, Data3, 'b-', linewidth=2, label=label3[0])
    plt.ylabel(label3[1])
    plt.xlabel("t(s)")
    plt.legend()
    ymax = np.max(Data3[0:lastindex])
    ymin = np.min(Data3[0:lastindex])
    plt.ylim([ymin*(1+0.1), ymax*(1+0.1)])
    plt.xlim([0, maxTime])
    plt.minorticks_on()
    plt.grid(b=True, which='major', color=[0.3, 0.3, 0.3], linestyle='-')
    plt.grid(b=True, which='minor', color=[0.2, 0.2, 0.3], linestyle='--')

   
    plt.subplots_adjust(left=0.07, bottom=0.07, right=0.7, top=0.98, wspace=0.5, hspace=0.26)


    return

def quit_figure(event):
    if event.key == 'q':
        plt.close('all')


def RPY2rotationMatrix(roll, pitch, yaw ):


    c1 = np.cos(roll)
    s1 = np.sin(roll)
    c2 = np.cos(pitch)
    s2 = np.sin(pitch)
    c3 = np.cos(yaw)
    s3 = np.sin(yaw)
    
    rotationMatrix = np.zeros([3, 3])


    rotationMatrix[0, 0] = c3*c2
    rotationMatrix[0, 1] = c3*s2*s1-s3*c1
    rotationMatrix[0, 2] = c3*s2*c1+s3*s1

    rotationMatrix[1, 0] = s3*c2
    rotationMatrix[1, 1] = s3*s2*s1+c3*c1
    rotationMatrix[1, 2] = s3*s2*c1-c3*s1

    rotationMatrix[2, 0] = -s2
    rotationMatrix[2, 1] = c2*s1
    rotationMatrix[2, 2] = c2*c1



    return rotationMatrix



def rotationMatrix2RPY(rotationMatrix):

    
    
    r11 = rotationMatrix[0, 0] 
    r12 = rotationMatrix[0, 1] 
    r13 = rotationMatrix[0, 2] 

    r21 = rotationMatrix[1, 0] 
    r22 = rotationMatrix[1, 1] 
    r23 = rotationMatrix[1, 2] 

    r31 = rotationMatrix[2, 0] 
    r32 = rotationMatrix[2, 1] 
    r33 = rotationMatrix[2, 2] 

    yaw = np.arctan2(r21, r11)
    pitch = np.arctan2(-r31, np.sqrt(r32*r32+r33*r33))
    roll = np.arctan2(r32, r33)

    angles = np.zeros([1, 3])

    angles[0][0] = roll
    angles[0][1] = pitch
    angles[0][2] = yaw

    return angles


if __name__ == "__main__": main()
