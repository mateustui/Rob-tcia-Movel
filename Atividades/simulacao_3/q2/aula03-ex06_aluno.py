"""
 Código revisado em: Maio de 2022
    - Modificação: START/STOP automático da simulação
    - Inspiração: https://github.com/CoppeliaRobotics/remoteApiBindings/blob/master/python/python/simpleSynchronousTest.py
    
 Exercício Uniciclo:
    - Abrir a cena no simulador V-REP (Não requer clique para iniciar simulação)
    - Iniciar script na IDE Spyder (Botão PLAY acima e STOP no console ao lado >>)
    - Considerar dt = 50ms
    - Restante do código inspirado em exemplos da RemoteAPI para linguagem Python do simulador V-REP
    - Documentação acessível em: http://www.coppeliarobotics.com/helpFiles/en/legacyRemoteApiOverview.htm
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

# Handle do Robô (Motor e Corpo)
_, BodyHandle = vrep.simxGetObjectHandle(clientID,'Uniciclo',vrep.simx_opmode_oneshot_wait)
_, BodyAlvo = vrep.simxGetObjectHandle(clientID,'Alvo',vrep.simx_opmode_oneshot_wait)

def Obter_Pose(handle): #Obtém posição do objeto da cena [x,y,phi]
    _, pos = vrep.simxGetObjectPosition(clientID, handle,-1, vrep.simx_opmode_streaming)
    _, ori = vrep.simxGetObjectOrientation(clientID,  handle, -1, vrep.simx_opmode_streaming)

    return pos[0], pos[1], ori[2]

def Setar_Pose(handle, x, y, phi): #Seta configuração do objeto da centa [x,y,phi]
    vrep.simxSetObjectPosition(clientID, handle, -1, [x,y,0.01], vrep.simx_opmode_streaming)
    vrep.simxSetObjectOrientation(clientID, handle, -1, [0,0,phi], vrep.simx_opmode_streaming)
    
def Norm_Ang(a): #Correção de ângulo para o domínio [-pi,pi]
    return np.arctan2(np.sin(a), np.cos(a))

def main():
    # Pose inicial NULA
    x = 0
    y = 0
    phi = 0
    controle = True
    count=0
    Kp = 2.0 # Começar com Kp = 2.0 || Em seguida, testar Kp = 0.5
    u0 = 1.5
    # Intervalo de integração
    dt = 0.05   # Intervalo de integração

    #Configura cena
    Setar_Pose(BodyHandle, x, y, phi)
    
    xg, yg, phig = Obter_Pose(BodyAlvo)
    
    time.sleep(0.1) #Corrige bug da primeira leitura
    
    #Loop de controle do robô
    while vrep.simxGetConnectionId(clientID) != -1:
        t0 = time.perf_counter() #Controle de tempo
        
        xg, yg, phig = Obter_Pose(BodyAlvo)
        


        phid = np.arctan2(yg-y,xg-x)
        e = phid - phi
        e = Norm_Ang(e) 
        rho = np.sqrt((xg-x)**2 + (yg-y)**2)
        
        if rho < 0.1 :
            controle = False
            count+=1
        else:
            controle=True
        
        if controle:
            u = u0
            w = Kp * e
        else:
            u = 0.0
            w = 0.0


        ###Implementar controle aqui####
        
    
        ###############################
        
        
        # Simulação da cinemática
        xp = u * np.cos(phi)
        yp = u * np.sin(phi)
        phip = w
            
        x += xp*dt
        y += yp*dt
        phi += phip*dt
             
        #Aplica cálculos ao objeto no simulador
        Setar_Pose(BodyHandle, x, y, phi)
        
        #Print de dados
        print("x: %1.2f  y: %1.2f  xg: %1.2f  yg: %1.2f" % (x, y, xg, yg))
    
        vrep.simxSynchronousTrigger(clientID); # Trigger next simulation step (Blocking function call)
        
        while(time.perf_counter()-t0 <= dt): _ # Loop 

try:
    main()
    
except KeyboardInterrupt:
    # stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    vrep.simxFinish(clientID)