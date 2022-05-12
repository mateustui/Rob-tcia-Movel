"""
 Exercício Uniciclo:
 - Abrir a cena no simulador V-REP e iniciar simulação
 - Iniciar script na IDE Spyder (Botão PLAY acima e STOP no console ao lado >>)
 - Considerar dt = 50ms
 - Restante do código inspirado em exemplos da RemoteAPI para linguagem Python do simulador V-REP
 - Documentação acessível em: http://www.coppeliarobotics.com/helpFiles/en/legacyRemoteApiOverview.htm
 - Child Script da cena deve conter incialização da API Remota em modo Síncrono simRemoteApi.start(1000, 1300, false, true) dentro da função sysCall_init()
 
# Pose inicial capturada do simulador:
_, pos = vrep.simxGetObjectPosition(clientID,  BodyHandle, -1, vrep.simx_opmode_oneshot_wait)
_, ori = vrep.simxGetObjectOrientation(clientID,  BodyHandle, -1, vrep.simx_opmode_oneshot_wait)

x = pos[0]
y = pos[1]
phi = ori[2]
"""

import vrep
import time
import sys
import numpy as np

#Fecha todas as comunicações existentes
vrep.simxFinish(-1) 

#Inicializa RemoteAPI com parâmetros (IP,PORT,waitUntilConnected,doNotReconnectOnceDisconnected,timeOutInMs,commThreadCycleInMs)
clientID = vrep.simxStart('127.0.0.1',1000,True,True,2000,5) 

#Ativa modo síncrono da RemoteAPI 
vrep.simxSynchronous(clientID, True) 

if clientID != -1:
    print ('Servidor conectado!')
else:
    #Falha de conexão
    print ('Problemas para conectar o servidor!')
    sys.exit('Programa finalizado!')
    
# Handle do Robô (Motor e Corpo)
_, BodyHandle = vrep.simxGetObjectHandle(clientID,'Uniciclo',vrep.simx_opmode_oneshot_wait)

#Função de simulação
#Inicialização
dt = 0.05

#Pose inicial NULA
x = 00
y = 0
phi = 0


#Pose desejada
xd = -5
yd = -8

controle = True
Kp = 2.0 # Começar com Kp = 2.0 || Em seguida, testar Kp = 0.5
u0 = 1.5

  # Simulação do Uniciclo
u = 0.0
w = 0.0

#Controle do tempo de simulação
t = 0

def norm_ang(a):
  return np.arctan2(np.sin(a),np.cos(a))    



#Loop de controle do robô. Roda por 10s
while controle:
    t0 = time.perf_counter() #Controle de tempo
    t+=dt    
   
    #Controle Goal-to-Goal básico
    phid = np.arctan2(yd-y,xd-x)
    e = phid - phi
    e = norm_ang(e) 
    rho = np.sqrt((xd-x)**2 + (yd-y)**2)
    
    if rho < 0.1:
        controle = False
    
    if controle:
        u = u0
        w = Kp * e
    else:
        u = 0.0
        w = 0.0

    xp = u * np.cos(phi)
    yp = u * np.sin(phi)
    phip = w

        
    x += xp*dt
    y += yp*dt
    phi += phip*dt
    
    #Correção de ângulo para o domínio [-pi,pi]
    #phi = np.arctan2(np.sin(phi), np.cos(phi))
        
    #Aplica cálculos ao objeto no simulador
    vrep.simxSetObjectPosition(clientID, BodyHandle, -1, [x,y,0.01], vrep.simx_opmode_streaming)
    vrep.simxSetObjectOrientation(clientID, BodyHandle, -1, [0,0,phi], vrep.simx_opmode_streaming)

    #Print de dados
    print("x: %1.2f  y: %1.2f  phi(°): %1.2f t: %1.2f" % (x, y, phi*180/np.pi, t))

    vrep.simxSynchronousTrigger(clientID); # Trigger next simulation step (Blocking function call)
    
    while(time.perf_counter()-t0 <= dt): _ # Loop de 50ms
    
#Desligamento
vrep.simxFinish(clientID) # fechando conexao com o servidor
print ('Conexao fechada!')