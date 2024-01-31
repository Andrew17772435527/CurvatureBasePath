
'''
class VehicleDynamicLimit:
    Velocity = 21.6     # m/s ---60km/h
    Accleration = 15    # m/ss
    Deceleration = -15  # m/ss
    Curvature = 1/8     # 1/m
    CurvatureRate = 0.01# 1/mm
    def __init__(self, Velocity, Accleration,Deceleration,Curvature,CurvatureRate):
        self.Velocity = Velocity
        self.Accleration = Accleration
        self.Deceleration = Deceleration
        self.Curvature = Curvature
        CurvatureRate = CurvatureRate
'''
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.widgets import Button
import numpy as np
import tools
import filter
import EstimatePath as ep
import math
from matplotlib.patches import Polygon
import copy

def CalculateAngle(Path_x,Path_y):
    angle = np.zeros(len(Path_x))
    for i in range(len(Path_x)-1):
        if math.sqrt( ((Path_x[i+1] - Path_x[i])*(Path_x[i+1] - Path_x[i]) + (Path_y[i+1] - Path_y[i])*(Path_y[i+1] - Path_y[i])) ) > 0.05:
            if (Path_x[i+1] - Path_x[i]) == 0:
                slope = float('inf')
            else:
                slope = (Path_y[i+1] - Path_y[i])/(Path_x[i+1] - Path_x[i])
                
            if (Path_x[i+1] - Path_x[i]) < 0 :   
                angle[i+1] = math.pi - math.atan(slope)
            else:
                angle[i+1] =  math.atan(slope)
        else:
            angle[i+1] = angle[i]
    return angle

def CalculateBounder(Path_x,Path_y,HalfWidth):
    angle = CalculateAngle(Path_x,Path_y)
    left_path = np.zeros((len(Path_x),2))
    right_path = np.zeros((len(Path_x),2))
    z = 1
    for i in range (len(Path_x)):
        if angle[i] >= (math.pi/2):
            z = -1
        else:
            z = 1
        left_path[i,0] = Path_x[i] - z*HalfWidth*math.sin(angle[i])
        left_path[i,1] = Path_y[i] + HalfWidth*math.cos(angle[i])
        right_path[i,0] = Path_x[i] + z*HalfWidth*math.sin(angle[i])
        right_path[i,1] = Path_y[i] - HalfWidth*math.cos(angle[i])  
    return left_path,right_path


def CalculateRowCurvatureRate(Curvature):
    CurvatureRateRow = np.zeros(len(Curvature))
    dt = 65/1000
    for i in range(len(Curvature)-1):
        CurvatureRateRow[i+1] = (Curvature[i+1]-Curvature[i]) / dt
    return CurvatureRateRow

if __name__ == '__main__':
    ############
    h5FileName = "data/rear_left_radar_14_05_32_to_14_06_02_1705298732255.h5"
    VechicleMotionT1 = dict(zip(ep.keys, ep.values))
    VehSelf = {'ALgt':0,'VLgt':0}
    odometry_data = {'timestamp':[],'hostVel_mps':[],'yawRate_radps':[],'steeringAngle_radps':[],'accel_mps2x':[],'Curvature_radps':[],'CurvatureRate_radpss':[]}
    odometry_data_copy = copy.copy(odometry_data)
    Path = ep.PreDictPath()
    #############
    # calculate curvature
    filterWeight  = np.array([0.1,0.1,0.1,0.3,0.4]) # for curvature rate
    filterWeight1  = np.array([0.3,0.1,0.1,0.53,-0.03]) # for lowPassFilter
    odometry_data = tools.Load_Hdf5(h5FileName)
    filter.calculateCurvature(odometry_data)
    filter.LimitChange(odometry_data['Curvature_radps'],0.01)
    filter.calculateCurvatureRate(odometry_data,filterWeight) #EEEEEEEEEEEEEEEEE
    od_path_data = tools.plot_od_path(odometry_data, odometry_data['timestamp'][0],50,0)
    #
    w = np.array([0.3,0.7])
    Curvature = odometry_data['Curvature_radps']
    CurvatureRate = odometry_data['CurvatureRate_radpss']
    VehSelf['VLgt'] = odometry_data['hostVel_mps'][0]
    VehSelf['ALgt'] = odometry_data['accel_mps2x'][0]
    X,Y = Path.BezierCurvePath1(VechicleMotionT1,VehSelf,Curvature[0],CurvatureRate[0],w)
    X_m,Y_m = Path.ModelBaseCurvePath(VehSelf,Curvature[0],CurvatureRate[0])
    # 参数范围
    a_min, a_max = 0, 1
    n_min, n_max = 0, len(od_path_data[0,:])-1
    
    # figure 3 CurvatureRate
    fig3, acr = plt.subplots()
    CurvatureRateRow = CalculateRowCurvatureRate(Curvature)
    linecr_row,      = acr.plot(CurvatureRateRow,color = 'r',label ='Row')
    CurvatureRateRow_f = filter.lowPassFilter(CurvatureRateRow,filterWeight1)
    linecr_f,          = acr.plot(CurvatureRateRow_f,color = 'b',label ='Filtered_f')
    linecr,          = acr.plot(CurvatureRate,color = 'g',label ='Filtered')
    acr.set_position([0.09, 0.23, 0.85, 0.705])
    acr.set_title('CurvatureRate filter')
    acr.legend() 
    
    # figure 2 Curvature
    fig2, ac = plt.subplots()
    Curvature_radps = filter.lowPassFilter(odometry_data['Curvature_radps'],filterWeight1)
    linec_row,      = ac.plot(odometry_data['Curvature_radps'],color = 'r',label ='Row')
    linec,          = ac.plot(Curvature_radps,color = 'g',label ='Filtered')
    #linecrate_row,  = ac.plot(odometry_data['CurvatureRate_radpss'],color = 'r',label ='CurvatureRate_row')
    #linecrate,      = ac.plot(odometry_data['CurvatureRate_radpss'],color = 'g',label ='CurvatureRate_change')
    ac.set_position([0.09, 0.23, 0.85, 0.705])
    ac.set_title('Curvature filter')
    ac.legend() 
    
    ####################################################################################################

    # figure 1
    n = 0
    x = od_path_data[0,:]
    y = od_path_data[1,:]
    end1 = n + 100
    end2 = n + 80
    begin = n-20
    if end1 > len(x)-1:
        end1 = len(x)-1
    if end2 > len(x)-1:
        end2 = len(x)-1
    if begin < 0:
        begin = 0
    fig1, ax = plt.subplots()
    car_shape = Polygon([[-0.9, -1], [2.8, -1], [2.8, 1], [-0.9, 1]])
    ax.add_patch(car_shape)
    line_bezier, = ax.plot(X,Y,linewidth=5,linestyle='dashed',color = 'r',label ='Predict Path')
    line_modelbase, = ax.plot(X_m,Y_m,linewidth=5,color = 'g',label ='Predict Path')
    leftpath,rightpath = CalculateBounder(X,Y,1.75)
    line_left_bounder, = ax.plot(leftpath[:,0],leftpath[:,1],color = 'k')
    line_right_bounder, = ax.plot(rightpath[:,0],rightpath[:,1],color = 'c')
    #line_sc = ax.scatter(rightpath[:,0],rightpath[:,1])
    #line_sc_center = ax.scatter(X,Y)
    line_od, = ax.plot(x[begin:end1],y[begin:end1],color = 'b',label ='Odometry Path')
    ax.set_xlim(-15, 35)
    ax.set_ylim(-10, 10)
    ax.set_position([0.09, 0.23, 0.85, 0.705])
    ax.set_title('Path Estimation')
    ax.legend() 
    
    #########################################################################################################
    # for path display
    def update(val):
        a = s_a.val
        n = s_n.val
        c_d = s_c.val
        cr_d = s_cr.val
        b = 1-a
        w1 = np.array([a,b])
        VehSelf['VLgt'] = odometry_data['hostVel_mps'][n]
        VehSelf['ALgt'] = odometry_data['accel_mps2x'][n]
        k1 = 0
        k2 = 0
        # 低速，转弯，减速，-----》进入弯道中 --- 需要看弯道的曲率,曲率越大补偿越多
        if (VehSelf['VLgt'] < 6) and (VehSelf['ALgt'] < 0) and (Curvature[n]*(Curvature[n]+CurvatureRate[n])>0) and (abs(Curvature[n]) > 0.030):
            print("enter the curve")
            k1 = 0
            k2 = 0 
        # 低速，转弯，加速，----》出弯道中   
        elif (VehSelf['VLgt'] < 6) and (VehSelf['ALgt'] > 0) and (Curvature[n]*(Curvature[n]+CurvatureRate[n])>0) and (abs(Curvature[n]) > 0.030):
            print("exit the curve",Curvature[n])
            d =(1 + 2*(Curvature[n]*VehSelf['VLgt'])/0.4) # 根据曲率和速度调节修正幅度
            k1 = -d*abs(Curvature[n])/Curvature[n]# 需要减小曲率,需要回正方向
            k2 = 0
            if Curvature[n]*CurvatureRate[n] > 0:# 正在加大转弯，需要减缓转弯
                k2 = -d*abs(CurvatureRate[n])/CurvatureRate[n]
        X,Y = Path.BezierCurvePath1(VechicleMotionT1,VehSelf,Curvature[n]+k1*c_d,CurvatureRate[n]+k2*cr_d,w1)
        X_m,Y_m = Path.ModelBaseCurvePath(VehSelf,Curvature[n],CurvatureRate[n])
        od_path_data = tools.plot_od_path(odometry_data, odometry_data['timestamp'][n],50,0)
        x = od_path_data[0,:]
        y = od_path_data[1,:]
        
        leftpath,rightpath = CalculateBounder(X,Y,1.75)
        line_left_bounder.set_data(leftpath[:,0],leftpath[:,1])
        line_right_bounder.set_data(rightpath[:,0],rightpath[:,1])
        
        #line_sc_center.set_offsets(np.column_stack((X, Y)))  # 更新点的坐标
        #line_sc.set_offsets(np.column_stack((rightpath[:,0], rightpath[:,1])))  # 更新点的坐标
        
        line_bezier.set_data(X,Y)
        line_modelbase.set_data(X_m,Y_m)
        line_od.set_data(x,y)
        fig1.canvas.draw_idle()
        
    # Slider for path    
    ax_a = fig1.add_axes([0.1, 0.1, 0.3, 0.03]) 
    s_a = Slider(ax_a, 'w_p3', a_min, a_max, valinit = 0.7,valstep=0.1)
    s_a.on_changed(update)
    ax_n = fig1.add_axes([0.1, 0.05, 0.3, 0.03])
    s_n = Slider(ax_n, 'n_od', n_min, n_max, valinit = 0,valstep=1)  
    s_n.on_changed(update)
    ax_c = fig1.add_axes([0.55, 0.1, 0.3, 0.03])
    s_c = Slider(ax_c, 'd_c', 0, 0.02, valinit = 0.01,valstep=0.001)  
    s_c.on_changed(update)
    ax_cr = fig1.add_axes([0.55, 0.05, 0.3, 0.03])
    s_cr = Slider(ax_cr, 'd_cr', 0, 0.02, valinit = 0.01,valstep=0.001)  
    s_cr.on_changed(update)
    
    ### add button
    def on_plus_clicked(event):
        current_val = s_n.val
        new_val = current_val + 1  # 根据需要调整增加的值
        s_n.set_val(new_val)

    def on_minus_clicked(event):
        current_val = s_n.val
        new_val = current_val - 1  # 根据需要调整减少的值
        s_n.set_val(new_val)

    # 创建加号按钮
    ax_plus = plt.axes([0.5, 0.15, 0.05, 0.03])
    button_plus = Button(ax_plus, '+')
    button_plus.on_clicked(on_plus_clicked)
    # 创建减号按钮
    ax_minus = plt.axes([0.45, 0.15, 0.05, 0.03])
    button_minus = Button(ax_minus, '-')
    button_minus.on_clicked(on_minus_clicked)
        

    ############################################################################################################    
    # for filter display
    def update_c(val):
        w1 = w_1.val
        w2 = w_2.val
        w3 = w_3.val
        w4 = w_4.val
        w5 = 0.5 - w4 #w_5.val
        filterWeight2  = np.array([w1,w2,w3,w4,w5])
        Curvature_radps = filter.lowPassFilter(odometry_data['Curvature_radps'],filterWeight2)
        linec.set_ydata(Curvature_radps)
        fig2.canvas.draw_idle()
    
    # Slider for curve
    cx_1 = fig2.add_axes([0.1, 0.16, 0.4, 0.03]) 
    w_1 = Slider(cx_1, 'w_1', -1, 1, valinit = filterWeight1[0],valstep=0.1)
    w_1.on_changed(update_c)
    cx_2 = fig2.add_axes([0.1, 0.13, 0.4, 0.03]) 
    w_2 = Slider(cx_2, 'w_2', -1, 1, valinit = filterWeight1[1],valstep=0.1)
    w_2.on_changed(update_c)
    cx_3 = fig2.add_axes([0.1, 0.1, 0.4, 0.03]) 
    w_3 = Slider(cx_3, 'w_3', -1, 1, valinit = filterWeight1[2],valstep=0.1)
    w_3.on_changed(update_c)
    cx_4 = fig2.add_axes([0.1, 0.07, 0.4, 0.03]) 
    w_4 = Slider(cx_4, 'w_4', -1, 1, valinit = filterWeight1[3],valstep=0.1)
    w_4.on_changed(update_c)
    cx_5 = fig2.add_axes([0.1, 0.04, 0.4, 0.03]) 
    w_5 = Slider(cx_5, 'w_5', -1, 1, valinit = filterWeight1[4],valstep=0.1)
    w_5.on_changed(update_c)
    
    #################################################################################################################
    
    # for filter display
    def update_cr(val):
        w1 = w_r1.val
        w2 = w_r2.val
        w3 = w_r3.val
        w4 = w_r4.val
        w5 = w_r5.val #w_5.val
        filterWeight3  = np.array([w1,w2,w3,w4,w5])
        CurvatureRate = filter.calculateCurvatureRate(odometry_data,filterWeight3)
        linecr.set_ydata(CurvatureRate)
        fig3.canvas.draw_idle()
    
    # Slider for curve
    cx_r1 = fig3.add_axes([0.1, 0.16, 0.4, 0.03]) 
    w_r1 = Slider(cx_r1, 'w_1', -1, 1, valinit = filterWeight[0],valstep=0.1)
    w_r1.on_changed(update_cr)
    cx_r2 = fig3.add_axes([0.1, 0.13, 0.4, 0.03]) 
    w_r2 = Slider(cx_r2, 'w_2', -1, 1, valinit = filterWeight[1],valstep=0.1)
    w_r2.on_changed(update_cr)
    cx_r3 = fig3.add_axes([0.1, 0.1, 0.4, 0.03]) 
    w_r3 = Slider(cx_r3, 'w_3', -1, 1, valinit = filterWeight[2],valstep=0.1)
    w_r3.on_changed(update_cr)
    cx_r4 = fig3.add_axes([0.1, 0.07, 0.4, 0.03]) 
    w_r4 = Slider(cx_r4, 'w_4', -1, 1, valinit = filterWeight[3],valstep=0.1)
    w_r4.on_changed(update_cr)
    cx_r5 = fig3.add_axes([0.1, 0.04, 0.4, 0.03]) 
    w_r5 = Slider(cx_r5, 'w_5', -1, 1, valinit = filterWeight[4],valstep=0.1)
    w_r5.on_changed(update_cr)
    

    
    plt.show()
    


