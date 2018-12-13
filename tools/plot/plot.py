import csv
import numpy as np
import matplotlib.pyplot as plt

def main():

    estPosition = np.zeros([0, 3])
    gtPosition = np.zeros([0, 3])
    estOrientationQ = np.zeros([0, 4])
    gtOrientationQ = np.zeros([0, 4])
    Fs = 20 # Frecuencia de la camara
    #Fs = 0.5*10**-3
    Ts = 1.0/Fs # intervalo de tiempo
   


    with open('outputUWSlam.csv', newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            estPosition = np.append( estPosition, [[float(row[0]), float(row[1]), float(row[2])]], 0)
            gtOrientationQ = np.append( gtOrientationQ, [[float(row[3]), float(row[4]), float(row[5]), float(row[6])]], 0)
        print(estPosition)
        print(estPosition[:, 0])
        
            
    time = np.arange(0, Ts*(estPosition[:, 0].size), Ts )

    plt.figure()

    plt.subplot(3, 1, 1)
    plt.plot(time, estPosition[:, 0], 'b-', linewidth=2, label='Posicion x estimada')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(time, estPosition[:, 1], 'b-', linewidth=2, label='Posicion y estimada')

    plt.legend()
    
    plt.subplot(3, 1, 3)
    plt.plot(time, estPosition[:, 2], 'b-', linewidth=2, label='Posicion z estimada')
    plt.legend()
    



    plt.show()
    



if __name__ == "__main__": main()