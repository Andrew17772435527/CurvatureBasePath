import numpy as np
import math
import matplotlib.pyplot as plt
import tools
    

def lowPassFilter(data,filterWeight):
    filteredData = np.zeros(len(data))
    for i in range(len(data) - 1):
        if i > 2:
            filteredData[i] = \
            filterWeight[0]*(data[i])+\
            filterWeight[1]*(data[i-1])+\
            filterWeight[2]*(data[i-2])+\
            filterWeight[3]*filteredData[i-1]+\
            filterWeight[4]*filteredData[i-2]
        else:
            filteredData[i] = data[i]
        filteredData[i] = f"{filteredData[i]:.5f}"

    #plt.plot(filteredData,color='g',label='Filtered')
    #plt.plot(data,color='r',label='Row')
    #plt.legend()
    #plt.show()
    return filteredData
    
def LimitChange(array,maxChange):
    for i in range(len(array)):
        if i>5:
            diff = array[i]-array[i-1]
            diff = tools.Limit(diff,maxChange,-maxChange)
            array[i-1] = array[i]-diff

# YawRate  = Curvature  * Velocity
def calculateCurvature(odometry_data):
    Curvature = np.zeros(len(odometry_data['timestamp']))
    for i in range(len(odometry_data['timestamp']) - 1):
        if 1:#odometry_data['hostVel_mps'][i] > 5:
            Curvature[i] = odometry_data['yawRate_radps'][i] / odometry_data['hostVel_mps'][i]
        elif 0:
            Curvature[i] = math.atan(odometry_data['steeringAngle_radps'][i]) / 2.94
        if i>0 and odometry_data['hostVel_mps'][i] > 10:
            limit = abs(0.6/(odometry_data['hostVel_mps'][i]*odometry_data['hostVel_mps'][i]))
            Curvature[i] =tools.Limit(Curvature[i],limit,-limit)# max comfortable lateral accleration lower than 0.6g > v*v*c
            
    odometry_data['Curvature_radps'] = Curvature

# calculate curvaturerate
def calculateCurvatureRate(odometry_data,filterWeight):
    dt = (odometry_data['timestamp'][1] -odometry_data['timestamp'][0])/1000
    CurvatureRate_radpss = np.zeros(len(odometry_data['Curvature_radps']))
    for i in range(len(odometry_data['Curvature_radps']) - 1):
        if i > 2:
            CurvatureRate_radpss[i] = \
            filterWeight[0]*((odometry_data['Curvature_radps'][i] -odometry_data['Curvature_radps'][i-1])/dt)+\
            filterWeight[1]*((odometry_data['Curvature_radps'][i-1] -odometry_data['Curvature_radps'][i-2])/dt)+\
            filterWeight[2]*((odometry_data['Curvature_radps'][i-2] -odometry_data['Curvature_radps'][i-3])/dt)+\
            filterWeight[3]*CurvatureRate_radpss[i-1]+\
            filterWeight[4]*CurvatureRate_radpss[i-2]
        elif i == 0 :
            CurvatureRate_radpss[2] = (odometry_data['Curvature_radps'][2] -odometry_data['Curvature_radps'][1])/dt
            CurvatureRate_radpss[1] = (odometry_data['Curvature_radps'][1] -odometry_data['Curvature_radps'][0])/dt
            CurvatureRate_radpss[0] = CurvatureRate_radpss[1]
        CurvatureRate_radpss[i] = tools.Limit(CurvatureRate_radpss[i],0.1,-0.1)
        CurvatureRateRate = CurvatureRate_radpss[i] - CurvatureRate_radpss[i-1]
        if abs(CurvatureRateRate) > 0.010:
            #print("CurvatureRateRate wave!")
            CurvatureRate_radpss[i] = CurvatureRate_radpss[i-1] + (CurvatureRateRate/abs(CurvatureRateRate))*0.010
    odometry_data['CurvatureRate_radpss'] = CurvatureRate_radpss
    return odometry_data['CurvatureRate_radpss']
    
if __name__ == '__main__':
    print("-------debug-------")
    odometry_data = {'timestamp':[],'hostVel_mps':[],'yawRate_radps':[],'steeringAngle_radps':[],'accel_mps2x':[],'Curvature_radps':[],'CurvatureRate_radpss':[]}
    xlsxFileName = "90turning.xlsx"
    h5FileName = "data/front_right_radar_08_13_57_to_08_14_14_1704932037591.h5"
    filterWeight  = np.array([0.2,0.1,0.1,0.93,-0.33])
    #odometry_data = tools.Load_Excel(xlsxFileName)
    odometry_data = tools.Load_Hdf5(h5FileName)
    calculateCurvature(odometry_data)
    lowPassFilter(odometry_data['Curvature_radps'],filterWeight)
    #
    calculateCurvatureRate(odometry_data,filterWeight)
    # Row CurvatureRate
    dt = (odometry_data['timestamp'][1] -odometry_data['timestamp'][0])/1000
    CurvatureRateRow = np.zeros(len(odometry_data['Curvature_radps']))
    CurvatureRateRow[0] = odometry_data['Curvature_radps'][1] -odometry_data['Curvature_radps'][0]
    for i in range(len(odometry_data['Curvature_radps'])-1):
        
        CurvatureRateRow[i+1] = (odometry_data['Curvature_radps'][i+1] -odometry_data['Curvature_radps'][i])/dt
    plt.figure(1)
    plt.plot(CurvatureRateRow,color='r')
    plt.plot(odometry_data['CurvatureRate_radpss'],color='g')
    plt.show()