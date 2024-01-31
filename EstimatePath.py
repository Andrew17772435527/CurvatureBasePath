
import numpy as np
import matplotlib.pyplot as plt
import math
from openpyxl import load_workbook
import tools
import PlotPath
import filter
import copy
import Simulation as si



# T0 is inital time and T1 is end time in this segement

VehSelfMinSpedInDrvrStEstimr = 0.2
LoPosValInDrevrStEstimr      = 1E-8
keys = ['PositionLongitudinal', 'VelocityLongitudinal','A','AccelerationLongitudinal','PositionLateral','VelocityLateral','AccelerationLateral','Speed','Curvature','CurvatureRate','HeadingAngle']
values =[0,0,0,0,0,0,0,0,0,0,0]



#############################################################

class PreDictPath:
    
    def __init__(self):
        self.VechicleMotionT0 =dict(zip(keys, values))
        pass

    ##################### BezierCurvePath #############

    def BezierCurvePath(self,VechicleMotionT1,VehSelf,Curvature,CurvatureRate):

        ControlPointsSegments = np.zeros((4,8))
        Y = np.array([])
        X = np.array([])
        w = [0.3,0.7]
        for Index in range(4):
            self.SetInitialMotion(Index+1,VechicleMotionT1,VehSelf,Curvature,CurvatureRate)
            StopppingDistance = self.CheckVehicleStopping()
            ControlPoints = self.CreateSplineSegment(StopppingDistance,w)
            VechicleMotionT1 = self.ComputeVehicleStaetAtT1(StopppingDistance,ControlPoints)
            ControlPointsSegments[Index] = ControlPoints 
            x1,y1 = PlotPath.PlotSplineCurve(ControlPoints,10)
            #plt.scatter(ControlPoints[0:3],ControlPoints[4:7], s = 80)
            X = np.append(X,x1)
            Y = np.append(Y,y1)
            #print("-------------------",Index+1,"-------------------")
        # return ControlPointsSegments
        plt.scatter(X,Y,s = 30)
        plt.plot(X,Y,linewidth=3,color = 'r',label ='Predict Path')
        #plt.plot(X,Y,linewidth=5,linestyle='dashed',color = 'r',label ='Predict Path')
        return X,Y
    
    def BezierCurvePath1(self,VechicleMotionT1,VehSelf,Curvature,CurvatureRate,w):
        
        ControlPointsSegments = np.zeros((4,8))
        Y = np.array([])
        X = np.array([])
        for Index in range(4):
            self.SetInitialMotion(Index+1,VechicleMotionT1,VehSelf,Curvature,CurvatureRate)
            StopppingDistance = self.CheckVehicleStopping()
            ControlPoints = self.CreateSplineSegment(StopppingDistance,w)
            VechicleMotionT1 = self.ComputeVehicleStaetAtT1(StopppingDistance,ControlPoints)
            ControlPointsSegments[Index] = ControlPoints 
            x1,y1 = PlotPath.PlotSplineCurve(ControlPoints,10)
            X = np.append(X,x1)
            Y = np.append(Y,y1)
            #print("-------------------",Index+1,"-------------------")
        print("-------------------","Bezier","-------------------")
        # return ControlPointsSegments
        # plt.plot(X,Y)
        return X,Y



    def SetInitialMotion(self,Index,VechicleMotionT1,VehSelf,Curvature,CurvatureRate):
        if Index==1:
            self.VechicleMotionT0['PositionLongitudinal'] = 0
            self.VechicleMotionT0['VelocityLongitudinal'] = VehSelf['VLgt']
            self.VechicleMotionT0['A'] = VehSelf['ALgt']
            self.VechicleMotionT0['AccelerationLongitudinal'] = VehSelf['ALgt']
            self.VechicleMotionT0['PositionLateral'] = 0
            self.VechicleMotionT0['VelocityLateral'] = 0
            self.VechicleMotionT0['AccelerationLateral'] = Curvature*VehSelf['VLgt']*VehSelf['VLgt']
            self.VechicleMotionT0['Speed'] = VehSelf['VLgt']
            self.VechicleMotionT0['Curvature'] = Curvature
            self.VechicleMotionT0['CurvatureRate'] = CurvatureRate
            self.VechicleMotionT0['HeadingAngle'] = 0
        else:
            self.VechicleMotionT0['PositionLongitudinal'] = VechicleMotionT1['PositionLongitudinal']
            self.VechicleMotionT0['VelocityLongitudinal'] = VechicleMotionT1['VelocityLongitudinal']
            self.VechicleMotionT0['A'] = VechicleMotionT1['A']
            self.VechicleMotionT0['AccelerationLongitudinal'] = VechicleMotionT1['AccelerationLongitudinal']
            self.VechicleMotionT0['PositionLateral'] = VechicleMotionT1['PositionLateral']
            self.VechicleMotionT0['VelocityLateral'] = VechicleMotionT1['VelocityLateral']
            self.VechicleMotionT0['AccelerationLateral'] = VechicleMotionT1['AccelerationLateral']
            self.VechicleMotionT0['Speed'] = VechicleMotionT1['Speed']
            self.VechicleMotionT0['Curvature'] = VechicleMotionT1['Curvature']
            if Index == 2 :
                self.VechicleMotionT0['CurvatureRate'] = - CurvatureRate
            else:
                self.VechicleMotionT0['CurvatureRate'] = 0
            self.VechicleMotionT0['HeadingAngle'] = VechicleMotionT1['HeadingAngle']
            
        print("path segement in index:",Index,'\tCurvature','%.4f' % self.VechicleMotionT0['Curvature'],\
            '\tCurvatureRate','%.4f' % self.VechicleMotionT0['CurvatureRate'],\
            '\tVelocity','%.4f' % self.VechicleMotionT0['VelocityLongitudinal'],\
              '\tA','%.4f' % self.VechicleMotionT0['A'] )
        
    def CheckVehicleStopping(self):
        StopppingDistance = []
        StopTime = tools.Limit(tools.SefaDivide(self.VechicleMotionT0['Speed'],(-self.VechicleMotionT0['A'])),4,0)
        LongitudinalStoppingDistance = self.VechicleMotionT0['VelocityLongitudinal']*StopTime + self.VechicleMotionT0['AccelerationLongitudinal']*StopTime*StopTime*0.5
        LateralStoppingDistance = self.VechicleMotionT0['VelocityLateral']*StopTime + self.VechicleMotionT0['AccelerationLateral']*StopTime*StopTime*0.5
        StopsWithinSegment = False

        if abs(self.VechicleMotionT0['Speed']) < VehSelfMinSpedInDrvrStEstimr:
            StopTime = 0.1
            LongitudinalStoppingDistance = 0
            LateralStoppingDistance = 0
        if StopTime > 0 and  StopTime< 1:
            StopsWithinSegment = True
        StopppingDistance = [StopsWithinSegment,LongitudinalStoppingDistance,LateralStoppingDistance]
        return StopppingDistance
    
    def ComputeVehicleStaetAtT1(self,StopppingDistance,ControlPoints):
        VechicleMotionT1 = dict(zip(keys, values))

        Speed = tools.Limit((self.VechicleMotionT0['Speed'] + self.VechicleMotionT0['A']*1),math.inf,LoPosValInDrevrStEstimr)
        A = self.VechicleMotionT0['A']
        if StopppingDistance[0]:
            A = 0
        # convert spline to cartessian
        PosnLgt = ControlPoints[3]
        PosnLat = ControlPoints[7]
        VLgt = -3*ControlPoints[2] + 3*ControlPoints[3]
        VLat = -3*ControlPoints[6] + 3*ControlPoints[7]
        # ControlPoints = [P0Lgt,P1Lgt,P2Lgt,P3Lgt,P0Lat,P1Lat,P2Lat,P3Lat]

        # compute heading angle
        HeadingFromVelocity = math.atan2(VLat,VLgt)
        if (abs(HeadingFromVelocity)<=0):
            HeadingFromVelocity = self.VechicleMotionT0['HeadingAngle']
        Curvature = self.VechicleMotionT0['Curvature'] + self.VechicleMotionT0['CurvatureRate']
        CorrParm = self.CorrectEuclodeanParamters(A,Speed,HeadingFromVelocity,Curvature)

        VechicleMotionT1['PositionLongitudinal']    = PosnLgt
        VechicleMotionT1['PositionLateral']         = PosnLat
        VechicleMotionT1['Speed']                   = Speed
        VechicleMotionT1['A']                       = A
        VechicleMotionT1['VelocityLongitudinal']    = CorrParm[0]
        VechicleMotionT1['VelocityLateral']         = CorrParm[1]
        VechicleMotionT1['AccelerationLongitudinal']= CorrParm[2]
        VechicleMotionT1['AccelerationLateral']     = CorrParm[3]
        VechicleMotionT1['Curvature']               = Curvature
        VechicleMotionT1['HeadingAngle']            = HeadingFromVelocity
        return VechicleMotionT1


    def CorrectEuclodeanParamters(self,A,Speed,HeadingAngle,Curvature):
        Acc =  Curvature*Speed*Speed
        ALgt = -Acc*math.sin(HeadingAngle) + A*math.cos(HeadingAngle)
        ALat = Acc*math.cos(HeadingAngle) + A*math.sin(HeadingAngle)
        VLgt = math.cos(HeadingAngle)*Speed
        VLat = math.sin(HeadingAngle)*Speed
        AcVc=[VLgt,VLat,ALgt,ALat]
        return AcVc
    
    def CreateSplineSegment(self,StopppingDistance,w):
        # InitialControlPointsCalculation
        P0Lgt = self.VechicleMotionT0['PositionLongitudinal']
        P1Lgt = P0Lgt + (1/3)*self.VechicleMotionT0['VelocityLongitudinal']
        P2Lgt = -P0Lgt + 2*P1Lgt + (1/6)*self.VechicleMotionT0['AccelerationLongitudinal']
        P0Lat = self.VechicleMotionT0['PositionLateral']
        P1Lat = P0Lat + (1/3)*self.VechicleMotionT0['VelocityLateral']
        P2Lat = -P0Lat + 2*P1Lat + (1/6)*self.VechicleMotionT0['AccelerationLateral']
        if StopppingDistance[0]:
            P1Lgt = StopppingDistance[1] + P0Lgt
            P2Lgt = StopppingDistance[1] + P0Lgt
            P1Lat = StopppingDistance[2] + P0Lat
            P2Lat = StopppingDistance[2] + P0Lat
        # ComputeCurvatureBasedControlPoint
        DetectableCurvature = 1E-5
        curmax = (self.VechicleMotionT0['Curvature']+1*self.VechicleMotionT0['CurvatureRate'])
        if (abs(self.VechicleMotionT0['Curvature']) > DetectableCurvature) & (abs(curmax)>DetectableCurvature):
            # Distance at time 1 s ,which is arc length
            distance = self.VechicleMotionT0['A']/2 + self.VechicleMotionT0['Speed']*1
            r1 = 1/self.VechicleMotionT0['Curvature']
            r2 = 1/curmax
            # Arc angle
            angleOfArc1 = distance/r1
            angleOfArc2 = distance/r2
            # Postion in coordinate system 
            xrot1 = r1*math.sin(angleOfArc1)
            yrot1 = -r1*math.cos(angleOfArc1) + r1
            xrot2 = r2*math.sin(angleOfArc2)
            yrot2 = -r2*math.cos(angleOfArc2) + r2
            # Rotate back to normal coordinate system
            x1 = xrot1*math.cos(self.VechicleMotionT0['HeadingAngle']) - yrot1*math.sin(self.VechicleMotionT0['HeadingAngle'])
            y1 = xrot1*math.sin(self.VechicleMotionT0['HeadingAngle']) + yrot1*math.cos(self.VechicleMotionT0['HeadingAngle'])
            x2 = xrot2*math.cos(self.VechicleMotionT0['HeadingAngle']) - yrot2*math.sin(self.VechicleMotionT0['HeadingAngle'])
            y2 = xrot2*math.sin(self.VechicleMotionT0['HeadingAngle']) + yrot2*math.cos(self.VechicleMotionT0['HeadingAngle'])
            # Approximate end position at time 1 s
            # w1 and w2 should be dynameic parameters to meet different
            w1 = w[0]
            w2 = w[1]
            # bigger Curvature tend to return  
            P3Lgt = self.VechicleMotionT0['PositionLongitudinal'] + w1*x1 + w2*x2
            P3Lat = self.VechicleMotionT0['PositionLateral'] + w1*y1 + w2*y2
        else: # straight line motion
            P3Lgt = self.VechicleMotionT0['PositionLongitudinal'] + (self.VechicleMotionT0['A']/2+self.VechicleMotionT0['Speed'])*math.cos(self.VechicleMotionT0['HeadingAngle'])
            P3Lat = self.VechicleMotionT0['PositionLateral'] + (self.VechicleMotionT0['A']/2+self.VechicleMotionT0['Speed'])*math.sin(self.VechicleMotionT0['HeadingAngle'])
        if StopppingDistance[0]:
            P3Lgt = self.VechicleMotionT0['PositionLongitudinal'] + StopppingDistance[1]
            P3Lat = self.VechicleMotionT0['PositionLateral'] + StopppingDistance[2]
        InitialControlPoints = [P0Lgt,P1Lgt,P2Lgt,P3Lgt,P0Lat,P1Lat,P2Lat,P3Lat]
        return InitialControlPoints

    


    ##################### ModelBaseCurvePath #############
    # y = c3*x^3+c2*x^2+c1*x+c0
    # c0 = offset  # c1 = slope  # c2 =1/2*cuvature # c3 = 1/6*curvaturerate
    def ModelBaseCurvePath(self,VehSelf,Curvatureinin,CurvatureRatein):
        Accleration = VehSelf['ALgt']
        Velocity = VehSelf['VLgt']
        VelocityReal = VehSelf['VLgt']
        Curvature  = copy.copy(Curvatureinin)
        Curvaturer  = copy.copy(Curvatureinin)
        CurvatureRate =copy.copy(CurvatureRatein)
        PiontNum = 30
        PredictTime = 4
        '''
        if (VehSelf['VLgt']*PredictTime + VehSelf['ALgt']*PredictTime*PredictTime) < MinPredictDistance:
            PredictTime = MinPredictDistance/VehSelf['VLgt']
        '''   
        dt  = PredictTime / (PiontNum-1)
        X = []
        Y = []
        X_r = []
        Y_r = []
        x = 0
        y = 0
        xr = 0
        yr = 0
        Heading  = 0
        Headingr  = 0

        for i in range(1,PiontNum):
            if i > 7 and i < 16:
                CurvatureRate = -CurvatureRate
            elif i>=16:
                CurvatureRate = 0
                
            x = x + Velocity * math.cos(Heading) * dt
            y = y + Velocity * math.sin(Heading) * dt
            VelocityReal = VelocityReal + Accleration * dt
            
            if VelocityReal > 5.0:
                Velocity = Velocity + Accleration * dt
                Heading  = Heading + Velocity * Curvature*dt
            else:
                Velocity = 5.0
                Heading  = Heading + Velocity * Curvature*dt
            Curvature = Curvature + CurvatureRate*dt
            
            X.append(x)
            Y.append(y)
            
        Velocity_ = -3.0
        for i in range(1,PiontNum):
            X_r.append(xr)
            Y_r.append(yr)
            xr = xr + Velocity_ * math.cos(Headingr) * dt
            yr = yr + Velocity_ * math.sin(Headingr) * dt
            Headingr  = Headingr + Velocity_ * Curvaturer*dt
            Curvaturer = Curvaturer + CurvatureRate*dt
        return X,Y,X_r,Y_r

    ##################### ClothoidCurvePath #############
    def ClothoidCurvePath(self,VehSelf,Curvature,CurvatureRate):
        #PathParam = []
        d_c1 = 0
        d_phi = 0
        d_offset = 0
        d_c0 = 0.000
        w = 1
        d_offset = 0
        offset   = 0
        phi = 0

        c0 = Curvature
        c1 = CurvatureRate
        PiontNum = 30
        x_max = 40 #VehSelf['VLgt']*100

        q = 1.05
        n = PiontNum 
        X = np.zeros(n)
        X[1] = x_max * (1 - q)/(1- math.pow(q,n-1))
        X[n-1] = x_max
        for i in range(2,n-1):
            X[i] = X[i-1] + q*(X[i-1]-X[i-2])
        Y = np.zeros(n)
        
        for i in range(n):
            temp = (1/6)*(w * d_c1 + c1)
            temp = temp*X[i]
            temp = temp + (1/2)*(w * d_c0+ c0)
            temp = temp*X[i]
            temp = temp + (w * d_phi + phi)
            temp = temp*X[i]
            temp = temp + (w * d_offset+ offset)
            Y[i] = temp
        plt.plot(X,Y,linewidth=5)
        return X,Y
        #plt.scatter(X, Y,color='y',s=30)


if __name__ == '__main__':
    ############
    VechicleMotionT1 = dict(zip(keys, values))
    VehSelf = {'ALgt':-1,'VLgt':3}
    odometry_data = {'timestamp':[],'hostVel_mps':[],'yawRate_radps':[],'steeringAngle_radps':[],'accel_mps2x':[],'Curvature_radps':[],'CurvatureRate_radpss':[]}
    h5FileName = "data/Ufront_right_radar_14_30_20_to_14_30_50_1701757820186.h5"
    Curvature = [0.4 ]#[0.1 ,0.05, 0, -0.05, -0.1]
    CurvatureRate = [0.00]#[0.0 ,0.00, 0, -0.00, -0.0]
    Path = PreDictPath()
    #############
    # calculate curvature
    filterWeight  = np.array([0.2,0.1,0.1,0.93,-0.33])
    filterWeight1  = np.array([0.3,0.1,0.1,0.83,-0.33])
    odometry_data = tools.Load_Hdf5(h5FileName)
    filter.calculateCurvature(odometry_data)
    filter.LimitChange(odometry_data['Curvature_radps'],0.01)
    #odometry_data['Curvature_radps'] = filter.lowPassFilter(odometry_data['Curvature_radps'],filterWeight1)
    filter.calculateCurvatureRate(odometry_data,filterWeight)
    #odometry_data['CurvatureRate_radpss'] = filter.lowPassFilter(odometry_data['CurvatureRate_radpss'],filterWeight1)
    # test case index 0 1 -1
    test = 0
    if test == 1:
        for i in range(1):
            Path.ClothoidCurvePath(VehSelf,Curvature[i],CurvatureRate[i])
            Path.ModelBaseCurvePath(VehSelf,Curvature[i],CurvatureRate[i])
            Path.BezierCurvePath(VechicleMotionT1,VehSelf,Curvature[i],CurvatureRate[i])
        plt.show()
    elif test == 0:
        n = 0 #data begining index
        for i in range(len(odometry_data["timestamp"])):#len(odometry_data["timestamp"])
            plt.cla()
            VehSelf['VLgt'] = odometry_data['hostVel_mps'][i+n]
            VehSelf['ALgt'] = odometry_data['accel_mps2x'][i+n]
            Curvature = odometry_data['Curvature_radps'][i+n]
            CurvatureRate = odometry_data['CurvatureRate_radpss'][i+n]
            print("CurvatureRate_radpss",[i+n],':',odometry_data['CurvatureRate_radpss'][i+n],"\tCurvature_radps",[i+n],':',odometry_data['Curvature_radps'][i+n])
            # odometry path
            tools.plot_od_path(odometry_data, odometry_data['timestamp'][i+n],i+n,1)
            # predict path
            Path.ClothoidCurvePath(VehSelf,Curvature,CurvatureRate)
            # Path.ModelBaseCurvePath(VehSelf,Curvature,CurvatureRate)
            X,Y = Path.BezierCurvePath(VechicleMotionT1,VehSelf,Curvature,CurvatureRate)
            leftpath,rightpath = si.CalculateBounder(X,Y,1.75)
            plt.plot(leftpath[:,0],leftpath[:,1],color = 'k')
            plt.plot(rightpath[:,0],rightpath[:,1],color = 'k')
            plt.xlim(-10, 30)
            plt.ylim(-10, 10)
            plt.pause(0.5)
            