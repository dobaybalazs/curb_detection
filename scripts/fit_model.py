#!/usr/bin/env python3
import rospy 
import numpy as np
from scipy.optimize import curve_fit
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointCloud
import sensor_msgs.point_cloud2 as pc2

def func(a, x, b, c, d):
    return a*x**3+b*x**2+c*x+d

class Fitter:

    def __init__(self):
        rospy.Subscriber("/left_points",PointCloud2,self.leftCallBack)
        rospy.Subscriber("/right_points",PointCloud2,self.rightCallBack)
        self.frame_id = rospy.get_param("/frame_id")

        self.pub_left = rospy.Publisher("/fitted_left", PointCloud, queue_size=10)
        self.pub_right = rospy.Publisher("/fitted_right", PointCloud, queue_size=10)



    def leftCallBack(self,data):
        pc = pc2.read_points(data,skip_nans=True,field_names=["x","y","z"])
        xdata = []
        ydata = []
        zdata = []
        for p in pc:
            xdata.append(np.float64(p[0]))
            ydata.append(np.float64(p[1]))
            zdata.append(np.float64(p[2]))
        ## Curve fit
        '''popt, pcov = curve_fit(func,xdata,ydata)
        result_y = []
        for x in xdata:
            result_y.append(func(x, *popt))'''
        z = np.polyfit(xdata,ydata,3)
        f = np.poly1d(z)
        x_new = np.linspace(xdata[0],xdata[-1],50)
        y_new = f(x_new)
        result = PointCloud()
        result.header.frame_id = self.frame_id
        for i in range(0,len(x_new)):
            p = Point()
            p.x = x_new[i]
            p.y = y_new[i]
            p.z = -1.5
            result.points.append(p)
        self.pub_left.publish(result)

    
    def rightCallBack(self,data):
        pc = pc2.read_points(data,skip_nans=True,field_names=["x","y","z"])
        xdata = []
        ydata = []
        zdata = []
        for p in pc:
            xdata.append(np.float64(p[0]))
            ydata.append(np.float64(p[1]))
            zdata.append(np.float64(p[2]))
        ## Curve fit
        '''popt, pcov = curve_fit(func,xdata,ydata)
        result_y = []
        for x in xdata:
            result_y.append(func(x, *popt))'''
        z = np.polyfit(xdata,ydata,3)
        f = np.poly1d(z)
        x_new = np.linspace(xdata[0],xdata[-1],50)
        y_new = f(x_new)
        result = PointCloud()
        result.header.frame_id = self.frame_id
        for i in range(0,len(x_new)):
            p = Point()
            p.x = x_new[i]
            p.y = y_new[i]
            p.z = -1.5
            result.points.append(p)
        self.pub_right.publish(result)


if __name__ == '__main__':
    rospy.init_node('fit_model',anonymous=True)

    Fitter()

    rospy.spin()