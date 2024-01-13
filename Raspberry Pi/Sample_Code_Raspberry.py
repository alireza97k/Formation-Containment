import time
import socket
import numpy as np
import threading 



Agent_ID = 3

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('172.20.10.6', 1245))
data = s.recv(1024)
data = data.decode('utf-8')
Start_Time_Code = eval(data)    
time_First_Start_R = time.time()

print("Start time is  : " , data )
s.close()


x_0 = np.array([[3],[2],[0]])

def Dynamic(initial,u):
    A = np.array([[1,1,0.0074],[0,1,0.0908],[0,0,0.8219]])
    B = np.array([[0.0003],[0.0092],[0.1781]])
    Output = np.matmul(A,initial) + np.matmul(B,u)
    return Output


def Send_Position():
    global x_0
    global time_First_Start_R
    global Start_Time_Code
    print("Start Code Send Position")
    Counter = 0
    Start_Time = time.time()
     
    while True:
        t1 = time.time();
    
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('172.20.10.6', 1234))
    
        Data_Send = x_0.tolist()
        Data_Send.append(Agent_ID)
        Data_Send = str(Data_Send)
        Data_Send = Data_Send.encode()
        s.send(Data_Send)
        s.close()
        
        t2 = time.time()
        #print("Time is : ",(t2-t1)*1000)
        if t2-t1 < 0.1:
            time.sleep(0.1 - (t2-t1))
        


def Recive_Control():
    global x_0
    global time_First_Start_R
    global Start_Time_Code
    print("Start Code Recieve Position ")
    Counter = 0
    Start_Time = time.time()
    while time.time()-time_First_Start_R < (60 - Start_Time_Code):
        z = 1
        
    while True:
        t1 = time.time();
        u = np.array([[0]])
        Output = Dynamic(x_0,u)
        x_0 = Output
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('172.20.10.6', 5678))
        data = s.recv(1024)
        data = data.decode('utf-8')
        data = eval(data)
 
        s.close()
        
        t2 = time.time()
        Counter = Counter + 1
        print(Counter)
        print(data)
        if t2-t1 < 0.2:
            time.sleep(0.2 - (t2-t1))




Thread1 = threading.Thread(target=  Send_Position,)
Thread2 = threading.Thread(target=  Recive_Control,)

Thread1.start()
Thread2.start()

Thread1.join()
Thread2.join()





