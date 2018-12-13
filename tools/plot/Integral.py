#!/usr/bin/env python
# -*- coding: utf-8 -*-



# n = 2 ** 6  # Número de intervalos
# f = 400.0  # Hz
# dt = 1 / (f * 16)  # Espaciado, 16 puntos por período
# t = np.linspace(0, (n - 1) * dt, n)  # Intervalo de tiempo en segundos
# y= signal.square(2 * np.pi * f* t)
# plt.figure()
# plt.axis('auto')
# plt.plot(t, y)
# plt.plot(t, y, 'ko')
# plt.xlabel('Tiempo (s)')
# plt.ylabel('$y(t)$')
#
#
# plt.figure()
# Y = fft(y) / n  # Normalizada
# frq = fftfreq(n, dt)  # Recuperamos las frecuencias
# plt.plot(frq, np.abs(Y))  # Representamos la parte imaginaria
# plt.annotate(s=u'f = 400 Hz', xy=(400.0, -0.5), xytext=(400.0 + 1000.0, -0.5 - 0.35), arrowprops=dict(arrowstyle = "->"))
# plt.annotate(s=u'f = -400 Hz', xy=(-400.0, 0.5), xytext=(-400.0 - 2000.0, 0.5 + 0.15), arrowprops=dict(arrowstyle = "->"))
# plt.annotate(s=u'f = 800 Hz', xy=(800.0, 0.25), xytext=(800.0 + 600.0, 0.25 + 0.35), arrowprops=dict(arrowstyle = "->"))
# plt.annotate(s=u'f = -800 Hz', xy=(-800.0, -0.25), xytext=(-800.0 - 1000.0, -0.25 - 0.35), arrowprops=dict(arrowstyle = "->"))
# plt.ylim(-1, 1)
# plt.xlabel('Frecuencia (Hz)')
# plt.ylabel('Im($Y$)')
#
#
# plt.show()
import matplotlib.pyplot as plt
import numpy as np
from numpy import pi
from scipy.fftpack import fft, fftfreq
from scipy import signal, arange
from scipy.signal import *

def plotSpectrum(y,Fs):
     """

     grafica la amplitud del espectro de y(t)

     """

     n = len(y) # longitud de la señal
     k = arange(n)
     T = n/Fs
     frq = k/T # 2 lados del rango de frecuancia
     frq = frq[range(int(n/2))] # Un lado del rango de frecuencia
     Y = fft(y)/n # fft calcula la normalizacion
     Y = Y[range(int(n/2))]
     plt.plot(frq,abs(Y),'r') # grafica el espectro de frecuencia
     plt.xlabel('Frecuencia (Hz)')
     plt.ylabel('|Y(f)|')


def fi(acc, dt):
    integral_matrix = np.array([])
    acc_sum = 0.0
    for x in acc:
        acc_sum = x*dt+acc_sum
        integral_matrix = np.append(integral_matrix, [acc_sum])
        
    return integral_matrix


def fd(acc, dt):
    derivate_matrix = np.array([])
    last_x = 0
    for x in acc:
        dx = (x-last_x)/dt
        derivate_matrix = np.append(derivate_matrix, [dx])
        last_x = x
        
    return derivate_matrix

def residualAcc(acc):
    residual_matrix = np.array([])
    for x in np.arange(acc.size):
        if  x != 0 and x%10 == 0 :
            residual = acc[x-1]-acc[x-10]
            residual_matrix = np.append(residual_matrix, [residual])

        
    return residual_matrix

def residualVel(acc, dt):
    residual_matrix = np.array([])
    temporal_data = np.array([])
    lastSum = 0
    currentSum = 0
    for x in np.arange(acc.size):
        if  x%10 == 0:
            currentSum = np.sum(temporal_data)
            residual = (currentSum-lastSum)*dt
            if x-10 != 0:
                print(x)
                residual_matrix = np.append(residual_matrix, [residual])
                temporal_data = np.array([])
                temporal_data = np.append(temporal_data, [acc[x]]) #primer dato futuro
            lastSum = currentSum
            
        else:
            temporal_data = np.append(temporal_data, [acc[x]])
  

    return residual_matrix








def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y



def main():

    #try:
        #with open("SenalSensorSerial.txt", "r") as out_file:
            #lines = out_file.readlines()

            #for line in lines:
                #Lectura = np.fromstring(line, dtype=float, sep=' ')
                #Y = np.vstack([Y, Lectura[1]])
    #except:
        #print("Error al abrir el archivo")
        #out_file.close()

        # Filter requirements.
    # Filter requirements.
  
    order = 5
    cutoff = 10.0 # desired cutoff frequency of the filter, Hz
    Fs = 200
    #Fs = 0.5*10**-3
    Ts = 1.0/Fs # intervalo de tiempo
    angxGt = np.loadtxt('angxGt.out')
    angyGt = np.loadtxt('angyGt.out')
    angzGt = np.loadtxt('angzGt.out')

    angxEst = np.loadtxt('angxEst.out')
    angyEst = np.loadtxt('angyEst.out')
    angzEst = np.loadtxt('angzEst.out')
    signal_vector = np.loadtxt('accy.out')
    gt_vel = np.loadtxt('vely.out')
    gt_pos = np.loadtxt('posy.out')
    bias = np.loadtxt('biasy.out')
    t_size = signal_vector.size
    t_vector = np.arange(0, Ts*t_size, Ts )
    t2_vector = np.arange(0, Ts*t_size-10*Ts, 10*Ts )
    y = signal_vector
    data = y



    # Get the filter coefficients so we can check its frequency response.
    b, a = butter_lowpass(cutoff, Fs, order)
    """
    w, h = freqz(b, a, worN=8000)
    plt.subplot(2, 1, 1)
    plt.plot(0.5*Fs*w/np.pi, np.abs(h), 'b')
    plt.plot(cutoff, 0.5*np.sqrt(2), 'ko')
    plt.axvline(cutoff, color='k')
    plt.xlim(0, 65)
    plt. plt.plot(t2_vector, residualAccxEst, 'y-', linewidth=2, label= 'Resiudal Aceleracion Medida')itle("Lowpass Filter Frequency Response")
    plt. plt.plot(t2_vector, residualAccxEst, 'y-', linewidth=2, label= 'Resiudal Aceleracion Medida')label('Frequency [Hz]')
    plt. plt.plot(t2_vector, residualAccxEst, 'y-', linewidth=2, label= 'Resiudal Aceleracion Medida')rid()
    """


    # Filter the data, and plot both the original and filtered signals.
    y_filter = lfilter(b, a, y) ## butter_lowpass_filter(data, cutoff, Fs, order)
    residualAccxEst = residualAcc(y_filter)
    residualAccxGt = residualAcc(lfilter(b, a, fd(gt_vel, 1/Fs)) )
    residualVelGt = residualAcc(gt_vel)
    residualVelEst = residualAcc(fi(y_filter, 1/Fs))

    residualPosGt = residualAcc(gt_pos)
    residualPosEst = residualAcc(fi(fi(signal_vector, 1/Fs), 1/Fs))
    print(type(residualPosEst[0]))

    """

    plt.subplot(2, 1, 2)
    plt.plot(t_vector, data, 'b-', label='data')
    plt.plot(t_vector, y_filter, 'g-', linewidth=2, label='filtered data')
    plt.xlabel('Time [sec]')
    plt.grid()
    plt.legend()
    plt.subplots_adjust(hspace=0.35)

    plt.figure()
    plt.subplot(2,2,1)
    plt.hist(signal_vector, bins='auto')
    plt.title("Histogramas de Infrarrojo 20 cm")
    plt.subplot(2,2,2)
    plt.plot(t_vector, signal_vector)
    plt.title("Se#al en tiempo")

    #plt.subplot(2,2,3)
  

    plt.figure()
    plotSpectrum(y_filter, Fs)
    plt.title("Se#al en frecuencia2")
 
	
    plt.figure()
    plotSpectrum(y, Fs)
    plt.title("Se#al en frecuencia")
   
    plt.figure()
    plt.plot(t_vector, gt_vel)
    plt.plot(t_vector, fi(lfilter(b, a, fd(gt_vel, 1/Fs)), 1/Fs))
    plt.title("Velocidad Real")

    

    plt.figure()
    plt.plot(t_vector, y, 'g-', linewidth=2, label='filtered data')
    plt.title("Se#al cruda")
    plt.legend()
    """
    """
    plt.figure()
    plt.plot(t_vector, y_filter, 'g-', linewidth=2, label='Aceleracion filtrada')
    plt.title("Se#al filtrada")
    plt.legend()

    plt.figure()
    plt.plot(t_vector, lfilter(b, a, fd(gt_vel, 1/Fs)), 'g-', linewidth=2, label='Derivada de velocidad')
    plt.plot(t_vector, y_filter, 'y-', linewidth=2, label='Acc Medida')
    plt.title("Derivada de velocidad")
    plt.legend()
  
    plt.figure()
    plt.plot(t_vector, gt_vel, 'g-', linewidth=2, label='Gt velocidad')
    plt.plot(t_vector, fi(y_filter, 1/Fs), 'y-', linewidth=2, label='Acc int')
    plt.title("Velocidad")
    plt.legend()
  

    plt.figure()
    plt.plot(t_vector, y_filter - lfilter(b, a, fd(gt_vel, 1/Fs)), 'g-', linewidth=2, label='Error aceleracion')
    plt.title("Error filtrado Aceleracion")
    plt.legend()
  

    plt.figure()
    plt.plot(t_vector, gt_pos, 'g-', linewidth=2, label='Gt pos')
    plt.plot(t_vector, fi(fi(lfilter(b, a, fd(gt_vel, 1/Fs)), 1/Fs), 1/Fs) , 'b-', linewidth=2, label='Pos int')
    plt.plot(t_vector, fi(fi(y_filter, 1/Fs), 1/Fs), 'y-', linewidth=2, label='Acc integ')
    plt.title("Posicion")
    plt.legend()
    
    """
    
    plt.figure()
    plt.plot(t_vector, angxGt, 'b-', linewidth=2, label='Roll Gt')
    plt.plot(t_vector, angxEst, 'y-', linewidth=2, label='Roll Est')
    plt.title("Angx")
    plt.legend()

    plt.figure()
    plt.plot(t2_vector, residualAcc(angxGt), 'b-', linewidth=2, label='Residual Roll Gt')
    plt.plot(t2_vector, residualAcc(angxEst), 'y-', linewidth=2, label='Residual Roll Est')
    plt.title("Residual Angx")
    plt.legend()

    plt.figure()
    plt.plot(t_vector, angyGt, 'b-', linewidth=2, label='Pitch Gt')
    plt.plot(t_vector, angyEst, 'y-', linewidth=2, label='Pitch Est')
    plt.title("Angy")
    plt.legend()


    plt.figure()
    plt.plot(t2_vector, residualAcc(angyGt), 'b-', linewidth=2, label='Residual Pitch Gt')
    plt.plot(t2_vector, residualAcc(angyEst), 'y-', linewidth=2, label='Residual Pitch Est')
    plt.title("Residual Angy")
    plt.legend()

    plt.figure()
    plt.plot(t_vector, angzGt, 'b-', linewidth=2, label='Yaw Gt')
    plt.plot(t_vector, angzEst, 'y-', linewidth=2, label='Yaw Est')
    plt.title("Angz")
    plt.legend()

    plt.figure()
    plt.plot(t2_vector, residualAcc(angzGt), 'b-', linewidth=2, label='Residual Yaw Gt')
    plt.plot(t2_vector, residualAcc(angzEst), 'y-', linewidth=2, label='Residual Yaw Est')
    plt.title("Residual Angz")
    plt.legend()

    """
    plt.figure()
    plt.plot(t_vector, bias, 'g-', linewidth=2, label='Bias')
    plt.title("bias")
    plt.legend()
  

    plt.figure()
    plt.plot(t_vector, fi(y_filter, 1/Fs), 'g-', linewidth=2, label='Bias')
    plt.title("Velocidad estimada")
    plt.legend()
    """

    """

    plt.figure()
    plt.plot(t2_vector, residualAccxGt, 'g-', linewidth=2, label=' Residual Aceleracion Real')
    plt.plot(t2_vector, residualAccxEst, 'y-', linewidth=2, label= 'Resiudal Aceleracion Medida')

    plt.title("Residual Acc")
    plt.legend()

    plt.figure()
    plt.plot(t2_vector, residualAccxGt- residualAccxEst, 'g-', linewidth=2, label=' Residual Aceleracion Real')

    plt.title("Error Residual Acc")


    plt.figure()
    plt.plot(t2_vector, residualVelGt, 'g-', linewidth=2, label=' Residual Velocidad Real')
    plt.plot(t2_vector, residualVelEst, 'y-', linewidth=2, label= 'Resiudal Velocidad Medida')

    plt.title("Residual Velocidad")
    plt.legend()

    plt.figure()
    plt.plot(t2_vector, residualVelGt- residualVelEst, 'g-', linewidth=2, label=' Residual Aceleracion Real')

    plt.title("Error Residual velocidad")

    plt.figure()
    plt.plot(t2_vector, residualPosGt, 'g-', linewidth=2, label=' Residual Pos Real')
    plt.plot(t2_vector, residualPosEst, 'y-', linewidth=2, label= 'Resiudal Pos Medida')

    plt.title("Residual Posicion")
    plt.legend()

    plt.figure()
    plt.plot(t2_vector, residualPosGt- residualPosEst, 'g-', linewidth=2, label=' Residual Pos Real')

    plt.title("Error Residual Posicion")

    plt.figure()
    plt.plot(t_vector, bias, 'g-', linewidth=2, label='Bias')
    plt.title("bias")
    plt.legend()

    plt.figure()
    plt.plot(t_vector, lfilter(b, a, fd(gt_vel, 1/Fs)), 'g-', linewidth=2, label='Derivada de velocidad')
    plt.plot(t_vector, y_filter, 'y-', linewidth=2, label='Acc Medida')
    plt.title("Derivada de velocidad")
    plt.legend()

    """
    

    plt.show()



if __name__ == "__main__": main()
