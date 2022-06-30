"""
 Autor: Prof. Lucas Vago Santana
 Curso: Robótica Móvel - Engenharia de Controle e Automação
 Instituição: Ifes - campus Linhares
 Revisado em: 22/05/2022
 
 # Modificação dos exemplos para controle direto da IDE Python
 # Utilize o PLAY e o CTRL+C do Spyder IDE para controlar a simulação sem 
   necessidade de clicar nos botões do CoppeliaSim
"""

import vrep
import time
import sys
import numpy as np

vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
   print ('Servidor conectado!') 
else:
    print ('Problemas para conectar o servidor!')
    sys.exit()

#Ativa modo síncrono da RemoteAPI
vrep.simxSynchronous(clientID, True) 

#Inicia a simulação
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);

# Handles da Cena [Todos os Objetos que compõem o robô]
_, TargetbodyHandle = vrep.simxGetObjectHandle(clientID,'Alvo',vrep.simx_opmode_oneshot_wait)
_, bodyHandle = vrep.simxGetObjectHandle(clientID,'Robo_Ifes',vrep.simx_opmode_oneshot_wait)

_, leftMotorHandle = vrep.simxGetObjectHandle(clientID,'LeftMotor',vrep.simx_opmode_oneshot_wait)
_, rightMotorHandle = vrep.simxGetObjectHandle(clientID,'RightMotor',vrep.simx_opmode_oneshot_wait)

_, SensorL = vrep.simxGetObjectHandle(clientID,'SensorL',vrep.simx_opmode_oneshot_wait)
_, SensorC = vrep.simxGetObjectHandle(clientID,'SensorC',vrep.simx_opmode_oneshot_wait)
_, SensorR = vrep.simxGetObjectHandle(clientID,'SensorR',vrep.simx_opmode_oneshot_wait)

  
def Controlador_Goal_To_Goal(xg, yg, phig, x, y, phi):
    rho = np.sqrt((xg-x)**2 + (yg-y)**2)
    phid = np.arctan2(yg-y,xg-x)
    
    kp = 1.5
    
    if rho >= 0.1:        
        u = 0.5
        e = np.arctan2(np.sin(phid - phi), np.cos(phid-phi))
        w = kp * e
    else:
        u = 0.0        
        w = 0.0
        
    return u, w

R = 0.035   # Raio da Roda
L = 0.28    # Distância entre-rodas
    
#Função de Controle do Uniciclo a partir de (v,w)
def ControleUniciclo(u,w,wr_b,wl_b):    
    wd=(2*u+w*L)/(2*R)+wr_b
    we=(2*u-w*L)/(2*R)+wl_b
    
    vrep.simxSetJointTargetVelocity(clientID,  leftMotorHandle, we, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, wd, vrep.simx_opmode_streaming)

#Função que simula algoritmos globais de sensoriamento [Ex. Visão Computacional]
def Obter_Posicao(handle): 
    _, pos = vrep.simxGetObjectPosition(clientID, handle,-1,vrep.simx_opmode_streaming)
    _, ori_body = vrep.simxGetObjectOrientation(clientID, handle,-1,vrep.simx_opmode_streaming)

    phi = ori_body[2] 
    phi = np.arctan2(np.sin(phi), np.cos(phi)) #[rad] - Correção domínio [-pi, pi]
    x = pos[0]   
    y = pos[1]
    
    return x, y, phi

#Função de Leitura dos sensores
def Ler_Sensores(handleL,handleC,handleR):    
    _, stateL, resL, _, _ = vrep.simxReadProximitySensor(clientID, handleL, vrep.simx_opmode_streaming)
    _, stateC, resC, _, _ = vrep.simxReadProximitySensor(clientID, handleC, vrep.simx_opmode_streaming)
    _, stateR, resR, _, _ = vrep.simxReadProximitySensor(clientID, handleR, vrep.simx_opmode_streaming)
    
    Max_Sensor_Distance = 1.0
    
    if(stateL == 1):
        sl = resL[2]
    else:
        sl = Max_Sensor_Distance
    if(stateC == 1):
        sc = resC[2]
    else:
        sc = Max_Sensor_Distance
    if(stateR == 1):
        sr = resR[2]
    else:
        sr = Max_Sensor_Distance
    
    return stateL, stateC, stateR, sl, sc, sr

def main():    
    #Variáveis  de Simulação (Parâmetros medidos/estabelecidos da cena do V-REP)
    t = 0       # Guarda tempo de simulação no Python
    dt = 0.05   # Intervalo de integração
    u = 0  
    w = 0  
     
   #Loop de controle do robô
    while vrep.simxGetConnectionId(clientID) != -1:
        t0 = time.perf_counter()
        
        t += dt
        
        x, y, phi = Obter_Posicao(bodyHandle)
        xg, yg, phig = Obter_Posicao(TargetbodyHandle) 
        
        ###################################################################
        #### Desvio de Obstáculo
        #### Implemente a seguir o desvio de obstáculos
        ###################################################################
        StateL, StateC, StateR, sl, sc, sr = Ler_Sensores(SensorL, SensorC, SensorR)
        

             


             
        u, w = Controlador_Goal_To_Goal(xg, yg, phig, x, y, phi)  
        
        #Saturação truncada
        umax = 1.0
        wmax = np.pi
        P=np.array([[30,30,30],[-30,-30,30]])
        S=np.array([[1-sl],[1-sc],[1-sr]])
        A=P@S
        wl_b,wr_b=A[0,0], A[1,0],

        if(np.abs(u) > umax):
            u = np.sign(u)*umax
        if(np.abs(w) > wmax):
            w = np.sign(w)*wmax
            
        ControleUniciclo(u,w,wr_b,wl_b)
        ###################################################################
        ###################################################################
        ###################################################################
        print("Tempo de Simulação: %1.2f [seg]" % (t))
        
        vrep.simxSynchronousTrigger(clientID); # Trigger next simulation step (Blocking function call)
        
        while(time.perf_counter()-t0 <= dt): _ # Para o loop até 50ms

try:
    main()
    
except KeyboardInterrupt:
    # stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    vrep.simxFinish(clientID)