"""
 Autor: Prof. Lucas Vago Santana
 Curso: Robótica Móvel - Engenharia de Controle e Automação
 Instituição: Ifes - campus Linhares
 Revisado em: 22/05/2022
 
 # Modificação dos exemplos para controle direto da IDE Python
 # Utilize o PLAY e o CTRL+C do Spyder IDE para controlar a simulação sem 
   necessidade de clicar nos botões do CoppeliaSim
"""

from turtle import pen
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

#Chama função genérica de reset do encoder. Está no Child Script das rodas no simulador V-REP
vrep.simxCallScriptFunction(clientID,'RightWheel',vrep.sim_scripttype_childscript,'Reset_Right_Encoder',[],[],[],bytearray(),vrep.simx_opmode_blocking)
vrep.simxCallScriptFunction(clientID,'LeftWheel',vrep.sim_scripttype_childscript,'Reset_Left_Encoder',[],[],[],bytearray(),vrep.simx_opmode_blocking)






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
    R = 0.033   # Raio da Roda
    L = 0.31    # Distância entre-rodas
    pos_x=0
    pos_y=0
    pos_phi=0   
    phi_p=0 
    erro_old_d=0
    erro_old_e=0
    erro_int_d=0
    erro_int_e=0   
    #Variáveis 
    t = 0       # Guarda tempo de simulação no Python
    dt = 0.05   # Intervalo de integração
    #xo = 0      # Posição da Odometria [xo,yo,phio]
   # yo = 0      # Posição da Odometria [xo,yo,phio]
    #phio = 0    # Posição da Odometria [xo,yo,phio]
    N = 20      # Resolução do Encoder
    pd_ant = 0  # Auxiliar - Cáluculo da variação de pulsos
    pe_ant = 0  # Auxiliar - Cáluculo da variação de pulsos
    #Qe = 0      # Velocidade [pulsos/seg]
    #Qd = 0      # Velocidade [pulsos/seg]
    #k = 0       #auxiliar
    #pe = 0
    #pd = 0
    #dpe = 0
    #dpd = 0

    #Loop de controle do robô
    while vrep.simxGetConnectionId(clientID) != -1:
        t0 = time.perf_counter()
        
        t += dt 
        
        #Obtém pulsos do encoder (pl- pulsos da esquerda) (pr - pulsos da direita)
        
        
        #Calcula posição por odometria
        xo = 0
        yo = 0
        phio = 0
        Kp_u = 1.0 # Começar com Kp = 2.0 || Em seguida, testar Kp = 0.5
        u0 = 0.5
        #Controle de posição   
        xg, yg, phig = Obter_Posicao(TargetbodyHandle)   

        phid = np.arctan2(yg-pos_y,xg-pos_x)
        e = phid - pos_phi
        e = norm_ang(e) 
        # print(np.degrees(e))
        rho = np.sqrt((xg-pos_x)**2 + (yg-pos_y)**2)
        
        if rho < 0.1 :
            controle = False
            
        else:
            controle=True
        
        if controle:
            u = u0
            w = Kp_u * e
        else:
            u = 0.0
            w = 0.0


         
        
        pe, pd = getEncoderPulses()    
        dpe = (pe -  pe_ant)/dt
        dpd = (pd - pd_ant)/dt
        wd_c=(((pd - pd_ant)*2*np.pi)/N)/dt
        we_c=(((pe - pe_ant)*2*np.pi)/N)/dt
        pe_ant, pd_ant = pe, pd
        
        N=20#resolucao encoder
        wd=(2*u+w*L)/(2*R)
        we=(2*u-w*L)/(2*R)
        



        xp=(R/2)*(we_c+wd_c)*np.cos(pos_phi)
        yp=(R/2)*(we_c+wd_c)*np.sin(pos_phi)
        phi_p=(R/L)*(wd_c-we_c)
        pos_x=pos_x+xp*dt
        pos_y=pos_y+yp*dt
        pos_phi=pos_phi+phi_p*dt

        Sp_d=(wd*N)/(2*np.pi)
        Sp_e=(we*N)/(2*np.pi)
        acao_d=0
        acao_e=0
        kp=0.1
        ki=0.0
        kd=0.0  
        erro_d=Sp_d-dpd
        erro_e=Sp_e-dpe
        
        
        acao_d=erro_d*kp+erro_int_d*ki+(erro_d-erro_old_d)/dt*kd
        acao_e=erro_e*kp+erro_int_e*ki+(erro_e-erro_old_e)/dt*kd
        erro_int_d+=erro_d*dt
        erro_int_e+=erro_e*dt
        erro_old_d=erro_d
        erro_old_e=erro_e
        vrep.simxSetJointTargetVelocity(clientID,  leftMotorHandle, acao_e, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, acao_d, vrep.simx_opmode_streaming)
        
        # Teste da odometria contra dados do simulador)
        x_real, y_real, phi_real = Obter_Posicao(bodyHandle)
        
        print("Pose Real        (x,y,phi): (%1.2f, %1.2f, %1.2f)" % (x_real, y_real, phi_real))
        print("Pose Odometria   (x,y,phi): (%1.2f, %1.2f, %1.2f)" % (pos_x, pos_y, pos_phi))
        #print("Pose do Alvo     (x,y,phi): (%1.2f, %1.2f, %1.2f)" % (xg, yg, phig))
        #print("Pulsos dos Encoders (E, D): (%d, %d)" % (pe,pd))
        #print("we, wd: (%d, %d)" % (wl,wr))
        
        vrep.simxSynchronousTrigger(clientID); # Trigger next simulation step (Blocking function call)
        
        while(time.perf_counter()-t0 <= dt): _ # Para o loop até 50ms
    

try:
    main()
    
except KeyboardInterrupt:
    # stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    vrep.simxFinish(clientID)