'''
    Adapted from course 16831 (Statistical Techniques).
    Initially written by Paloma Sodhi (psodhi@cs.cmu.edu), 2018
    Updated by Wei Dong (weidong@andrew.cmu.edu), 2021
'''

from tqdm import tqdm
import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
from map_reader import MapReader

# variables :
#LaserReadings = [x, y, theta, xl, yl, thetal, r1......r180]
#
# parameters :
#zHit, zRand, zShort, zMax, sigmaHit, lambdaShort

class SensorModel:
    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """
    def __init__(self, occupancy_map):
        """
        TODO : Tune Sensor Model parameters here
        The original numbers are for reference but HAVE TO be tuned.
        """
        # self._z_hit = 1
        # self._z_short = 0.1
        # self._z_max = 0.1
        # self._z_rand = 100
        # # # self._z_rand = 100
        # self._sigma_hit = 50
        # self._lambda_short = 0.1
        
        # Parameters
        self._z_hit = 150
        self._z_short = 17.5
        self._z_max = 15
        self._z_rand = 100
        self._sigma_hit = 100
        self._lambda_short = 5
        
        # probability
        self.norm = norm
        self._min_probability = 0.35
        self._subsampling = 2

        # lasers
        self.laserMax = 8183
        self.nLaser = 30
        self.laserX = np.zeros((self.nLaser,1))
        self.laserY = np.zeros((self.nLaser,1))
        self.beamsRange = np.zeros((self.nLaser,1))

        # map
        self.OccMap = occupancy_map
        self.OccMapSize = np.size(occupancy_map)
        self.resolution = 10

    def WrapToPi(self,angle):
        
        angle_wrapped = angle - 2*np.pi * np.floor((angle + np.pi) / (2*np.pi))
        return angle_wrapped

    def getProbability_B(self, z_star, z_reading):
        """
        References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
        [Chapter 6.3]
        """
        # HIT - equation (6.4)
        if z_reading >= 0 and z_reading <= self.laserMax:
            pHit = np.exp(-1/2 * (z_reading - z_star)**2 / (self._sigma_hit **2))
            pHit = pHit/(np.sqrt(2*np.pi*self._sigma_hit**2))   
        else:
            pHit = 0
  
        # SHORT - equation (6.7)
        if z_reading >= 0 and z_reading <= z_star:
            # eta = 1.0/(1-np.exp(-lambdaShort*z_star))
            eta = 1
            pShort = eta*self._lambda_short*np.exp(-self._lambda_short*z_reading)
        else:
            pShort = 0
      
        # MAX - equation (6.10)
        if z_reading >= self.laserMax:
            pMax = self.laserMax
        else:
            pMax = 0
                
        # RAND - equation (6.11)
        if z_reading >=0 and z_reading < self.laserMax:
            pRand =  1/self.laserMax
        else:
            pRand = 0

        # add all probs scaled by z parameters.    
        p = self._z_hit*pHit + self._z_short*pShort+ self._z_max*pMax + self._z_rand*pRand
        # normalize
        p /= (self._z_hit + self._z_short + self._z_max + self._z_rand)
        
        return p ,pHit, pShort, pMax, pRand

    def rayCast(self,x_t1,resolution):
        
        '''
        vectorizing (mask) --- tried but did not work, probably shoud've tried precomputing for look up table.
        '''
        # beamsRange = np.zeros(self.nLaser)
        # laserX = np.zeros(self.nLaser)
        # laserY = np.zeros(self.nLaser)
        # angs   = np.zeros(self.nLaser)
        # L = 25
        # xc = x_t1[0]
        # yc = x_t1[1]
        # myPhi = x_t1[2]
        # ang = myPhi - np.pi/2
        # ang =self.WrapToPi(ang)
        # offSetX = xc + L* np.cos(ang)
        # offSetY = yc + L* np.sin(ang)
        # angStep = np.pi/self.nLaser
        # r = np.linspace(0,self.laserMax,500)
        
        # for i in range(self.nLaser):
            
        #     ang += angStep*i
        #     ang = self.WrapToPi(ang)
        #     # casting rays
        #     x = offSetX + r * np.cos(ang)
        #     y = offSetY + r * np.sin(ang)

        #     xInt = np.floor(x/self.resolution).astype(int)
        #     yInt = np.floor(y/self.resolution).astype(int)

        #     # mask = np.zeros_like(xInt).astype(bool)
        #     # mask1= np.zeros_like(xInt).astype(bool)
        #     # mask1[(xInt < 800) & (xInt>=0) & (yInt>=0) & (yInt < 800)] == True
        #     # print("x",xInt[mask1].shape,yInt[mask1].shape)
        #     # print("asdfsaf",self.OccMap[yInt[mask1],xInt[mask1]])

        #     xWithin = np.argwhere(xInt<800)
        #     yWithin = np.argwhere(yInt<800)
        #     within = np.intersect1d(xWithin,yWithin)
        #     hitInd = np.

        #     ii = 0
        #     for xx, yy in zip(xInt[mask1], yInt[mask1]):
        #         if((np.abs(self.OccMap[yInt[xx],xInt[yy]]) > 0.35)):
        #             idx = ii 
        #             break
        #         ii+=1

        #     idx = np.argwhere(mask1==True)[ii]
        #     mask[(np.abs(self.OccMap[yInt[mask1],xInt[mask1]]) > 0.35)] == True
        #     mask[((xInt < 800) & (yInt < 800)) & (np.abs(self.OccMap[yInt,xInt]) > 0.35)] == True
        #     laserX[idx] = x[idx]
        #     laserY[idx] = y[idx]
        #     beamsRange = r[idx]
        
        ''' 
        normal looping -----
        '''
        beamsRange = np.zeros(self.nLaser)
        laserX = np.zeros(self.nLaser)
        laserY = np.zeros(self.nLaser)
        angs   = np.zeros(self.nLaser)
        L = 25

        xc = x_t1[0]
        yc = x_t1[1]
        myPhi = x_t1[2]
        ang = myPhi - np.pi/2
        ang =self.WrapToPi(ang)
        offSetX = xc + L* np.cos(ang)
        offSetY = yc + L* np.sin(ang)
        
        angStep = np.pi/self.nLaser
        
        '''
        set ray step size, using 2 grid step size for each beam.
        '''
        r = np.linspace(0,self.laserMax,400)
        
        for i in range(self.nLaser):

            ang = self.WrapToPi(ang)
          
            for rs in r:
                
                x = offSetX + rs*np.cos(ang)
                y = offSetY + rs*np.sin(ang)

                xInt = np.floor(x/self.resolution).astype(int)
                yInt = np.floor(y/self.resolution).astype(int)

                # if current laser pos in valid position within occupancy grid and it likely hit something.
                if xInt < 800 and yInt < 800 and np.abs(self.OccMap[yInt,xInt]) > 0.35:
                    beamsRange[i] = rs
                    angs[i] = ang
                    laserX[i] = xInt
                    laserY[i] = yInt
                    break
                    
                # if end of the loop, range is laser's max range.    
                if rs >= self.laserMax:
                    beamsRange[i] = self.laserMax

            ang += angStep

        

        return beamsRange,laserX,laserY
    
    def beam_range_finder_model(self, z_t1_arr, x_t1):
        
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """
        """
        TODO : Add your code here
        """
        
        '''
        References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
        [Chapter 6.3]. - Beam range finder model algorithm.
        '''
        # print("laser range readings 50: ", z_t1_arr[:50])
        q = 0
        step = int(180/self.nLaser)
        z_reading = [z_t1_arr[n] for n in range(0,180,step)]
        zt_star,laserX,laserY = self.rayCast(x_t1,self.resolution)
        probs = np.zeros(self.nLaser)

        for i in range(self.nLaser):
            probs[i] ,pHit,pShort,pMax,Prand = self.getProbability_B(zt_star[i],z_reading[i])
            q += np.log(probs[i])
        
        # q is very small and thus negative after taking log, make them positive and scale with # lasers.
        q = self.nLaser/ np.abs(q)

        return q
    