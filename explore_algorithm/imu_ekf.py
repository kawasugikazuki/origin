import numpy as np

def get_angle_acc(acc):
    th_acc=-np.arctan2(acc[0],np.sign(acc[2])*np.sqrt(acc[1]*acc[1]+acc[2]*acc[2]))
    ps_acc=np.arctan2(acc[1],acc[2])
    y=np.array([th_acc,ps_acc])
    return y

def get_Kalamgain(P,c,r):
    CPC=np.dot(c,np.dot(P,c))+r
    return np.dot(P,np.dot(c,np.linalg.inv(CPC)))

def get_preEstimation2(x,gyro,Ts,Tri):
    Q=np.array([[0,Tri[1,0],-Tri[1,1]],[1,Tri[1,1]*Tri[0,2],Tri[1,0]*Tri[0,2]]])
    return x+np.dot(Q,gyro)*Ts

def get_preVariance2(x,gyro,P,b,q,Ts,Tri):
    A=np.array([[1,-(Tri[1,1]*gyro[1]+Tri[1,0]*gyro[2])*Ts],[(Tri[1,0]/Tri[0,0]/Tri[0,0]*gyro[1]-Tri[1,1]/Tri[0,0]/Tri[0,0]*gyro[2])*Ts,1+(Tri[1,0]*Tri[0,2]*gyro[1]-Tri[1,1]*Tri[0,2]*gyro[2])*Ts]])
    return np.dot(A,np.dot(P,A.transpose()))+q

def get_Variance(g,c,P_):
    return np.dot((np.identity(2)-(np.dot(g,c.T))),P_)

def Kalman_filer2(x,y,gyro,c,b,q,r,P,Ts,Tri):
    x_=get_preEstimation2(x,gyro,Ts,Tri)
    P_=get_preVariance2(x,gyro,P,b,q,Ts,Tri)
    g=get_Kalamgain(P_,c,r)
    return x_+np.dot(g,y-np.dot(c,x_)),get_Variance(g,c,P_)

def Jacobian_forprocessvariance2(Tri):
    return np.array([[0,Tri[1,0],-Tri[1,1]],[1,Tri[1,1]*Tri[0,2],Tri[1,0]*Tri[0,2]]])

def get_Trigonometrxic(x):
    return np.array([[np.cos(x[0]),np.sin(x[0]),np.tan(x[0])],[np.cos(x[1]),np.sin(x[1]),np.tan(x[1])]])