import matplotlib.pyplot as plt
import copy
import numpy as np

def PlotSplineCurve(FourElements,PointNum):
    
    X = np.array([])
    Y = np.array([]) 
    i=0
    for n in range(PointNum+1):
        x = FourElements[0]*(1-i)*(1-i)*(1-i) + 3*FourElements[1]*(1-i)*(1-i)*i + 3*FourElements[2]*(1-i)*i*i + FourElements[3]*i*i*i
        y = FourElements[4]*(1-i)*(1-i)*(1-i) + 3*FourElements[5]*(1-i)*(1-i)*i + 3*FourElements[6]*(1-i)*i*i + FourElements[7]*i*i*i
        i += 1/PointNum
        X = np.append(X,x)
        Y = np.append(Y,y)
    # plt.scatter(X,Y,color='r',s=30)
    # plt.plot(X,Y,linewidth=5)
    return X,Y

def PlotSplineCurveParallel(FourElementsin,PointNum,width):
    
    X=[]
    Y=[]
    i=0
    FourElements = copy.copy(FourElementsin)
    FourElements[4] =FourElements[4]+width
    FourElements[5] =FourElements[5]+width
    FourElements[6] =FourElements[6]+width
    FourElements[7] =FourElements[7]+width
    for n in range(PointNum+1):
        x = FourElements[0]*(1-i)*(1-i)*(1-i) + 3*FourElements[1]*(1-i)*(1-i)*i + 3*FourElements[2]*(1-i)*i*i + FourElements[3]*i*i*i
        y = FourElements[4]*(1-i)*(1-i)*(1-i) + 3*FourElements[5]*(1-i)*(1-i)*i + 3*FourElements[6]*(1-i)*i*i + FourElements[7]*i*i*i
        i += 1/PointNum
        X.append(x)
        Y.append(y)
    plt.plot(X,Y,color='r')
    
if __name__ == '__main__':
    print("-------debug-------")
    
