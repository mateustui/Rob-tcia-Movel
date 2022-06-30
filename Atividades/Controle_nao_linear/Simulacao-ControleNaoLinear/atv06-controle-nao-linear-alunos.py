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
_, leftMotorHandle = vrep.simxGetObjectHandle(clientID,'LeftMotor',vrep.simx_opmode_oneshot_wait)
_, rightMotorHandle = vrep.simxGetObjectHandle(clientID,'RightMotor',vrep.simx_opmode_oneshot_wait)
_, bodyHandle = vrep.simxGetObjectHandle(clientID,'Robo_Ifes',vrep.simx_opmode_oneshot_wait)
_, noseHandle  = vrep.simxGetObjectHandle(clientID,'Nariz',vrep.simx_opmode_oneshot_wait)
_, SensorL = vrep.simxGetObjectHandle(clientID,'SensorL',vrep.simx_opmode_oneshot_wait)
_, SensorC = vrep.simxGetObjectHandle(clientID,'SensorC',vrep.simx_opmode_oneshot_wait)
_, SensorR = vrep.simxGetObjectHandle(clientID,'SensorR',vrep.simx_opmode_oneshot_wait)

#Chama função genérica de reset do encoder. Está no Child Script das rodas no simulador V-REP
vrep.simxCallScriptFunction(clientID,'RightWheel',vrep.sim_scripttype_childscript,'Reset_Right_Encoder',[],[],[],bytearray(),vrep.simx_opmode_blocking)
vrep.simxCallScriptFunction(clientID,'LeftWheel',vrep.sim_scripttype_childscript,'Reset_Left_Encoder',[],[],[],bytearray(),vrep.simx_opmode_blocking)

#Função que reposiciona AlvoTjt (Verde)
def setTargetTjt(xg, yg):    
    vrep.simxSetObjectPosition(clientID, TargetbodyHandle, -1, [xg, yg, 0], vrep.simx_opmode_streaming)
    
def Controlador_Goal_To_Goal(xg, yg, phig, x, y, phi):
    u = 0.0
    w = 0.0
        
    return u, w

def Controlador_Coordenadas_Polares(xg, yg, phig, x, y, phi):
    u = 0.0
    w = 0.0

    return u, w

def Controlador_Coordenadas_Polares_com_Orientacao_Final(xg, yg, phig, x, y, phi):
    u = 0.0
    w = 0.0

    return u, w

def Controlador_Cinematica_Inversa(xgp, ygp, xg, yg, phig, x, y, phi):
    u = 0.0
    w = 0.0
    
    return u, w

R = 0.035   # Raio da Roda
L = 0.28    # Distância entre-rodas
    
#Função de Controle do Uniciclo a partir de (v,w)
def ControleUniciclo(u,w):    
    wd=(2*u+w*L)/(2*R)
    we=(2*u-w*L)/(2*R)
    
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

#Função que captura pulsos de encoders simulados nas rodas do robô
#Encoder está simulado no Child Script de cada roda na cena do V-REP 
def getEncoderPulses():
    _, pe = vrep.simxGetIntegerSignal(clientID,'Left_Encoder',vrep.simx_opmode_streaming)
    _, pd = vrep.simxGetIntegerSignal(clientID,'Right_Encoder',vrep.simx_opmode_streaming)
     
    return pe, pd
  
def norm_ang(a):
    return np.arctan2(np.sin(a),np.cos(a))

def main():    
    #Variáveis  de Simulação (Parâmetros medidos/estabelecidos da cena do V-REP)
    t = 0       # Guarda tempo de simulação no Python
    dt = 0.05   # Intervalo de integração
    xg_ant = 0  # Cálculo da derivada numérica da trajetória
    yg_ant = 0  # Cálculo da derivada numérica da trajetória
    
    
   #Loop de controle do robô
    while vrep.simxGetConnectionId(clientID) != -1:
        t0 = time.perf_counter()
        
        t += dt
            
        ###################################################################
        #### Implementar Controlador do Robô aqui                 #########
        ###################################################################  
        tjt = 1 # 0: Controle de Posição   1: Controle de Trajetória    
        if(tjt == 0):
            #Poisção Desejada
            xg, yg, phig = Obter_Posicao(TargetbodyHandle)  
            xgp = 0
            ygp = 0
        elif(tjt == 1):
            #Trajetória Desejada
            xg = 2 * np.cos(0.1 * t)
            yg = 0.5 * np.sin(0.2 * t)
            phig = 0
            xgp = (xg - xg_ant) / dt
            ygp = (yg - yg_ant) / dt
            xg_ant = xg
            yg_ant = yg   
            setTargetTjt(xg, yg)
        
        x, y, phi = Obter_Posicao(bodyHandle)
        
        controlador = 4 #Selecionar controlador desejado
        if(controlador == 1):
            u, w = Controlador_Goal_To_Goal(xg, yg, phig, x, y, phi)        
        elif(controlador == 2):
            u, w = Controlador_Coordenadas_Polares(xg, yg, phig, x, y, phi)
        elif(controlador == 3):
            u, w = Controlador_Coordenadas_Polares_com_Orientacao_Final(xg, yg, phig, x, y, phi)
        elif(controlador == 4):
            u, w = Controlador_Cinematica_Inversa(xgp, ygp, xg, yg, phig, x, y, phi)
        
        #Saturação truncada
        umax = 1.0
        wmax = np.pi
        
        if(np.abs(u) > umax):
            u = np.sign(u)*umax
        if(np.abs(w) > wmax):
            w = np.sign(w)*wmax
            
        ControleUniciclo(u,w)
        ###################################################################
        ###################################################################
        ###################################################################
        
        print("xg: %1.2f  yg: %1.2f  phig: %1.2f" % (xg, yg, phig))
        print("x : %1.2f  y : %1.2f  phi : %1.2f" % (x, y, phi))
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