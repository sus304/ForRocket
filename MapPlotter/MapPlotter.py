# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

class Plotter:
    def __init__(self):
        pass

    def Plot(self, launchsite, landingpoint):
        figsize = 6
        dpi = 100

        landingpoint.shift_point(launchsite)
        size = len(landingpoint.ballistic_x)
        Vw_size = size // 8

        plt.close('all')
        plt.figure('ballistic', figsize=(figsize,figsize), dpi=dpi)
        for i in range(Vw_size):
            start = i * 8
            array_x = np.append(landingpoint.ballistic_x[start:start+8],landingpoint.ballistic_x[start]) 
            array_y = np.append(landingpoint.ballistic_y[start:start+8],landingpoint.ballistic_y[start])
            plt.plot(array_x, array_y, color='orange', linewidth=0.5)
        plt.xlim = [0, launchsite.imgwidth]
        plt.ylim = [0, launchsite.imgheight]
        ax = plt.gca()
        ax.set_aspect(1.0)
        plt.imshow(np.asarray(launchsite.img))

        plt.figure('decent', figsize=(figsize,figsize), dpi=dpi)
        for i in range(Vw_size):
            start = i * 8
            array_x = np.append(landingpoint.decent_x[start:start+8],landingpoint.decent_x[start]) 
            array_y = np.append(landingpoint.decent_y[start:start+8],landingpoint.decent_y[start])
            plt.plot(array_x, array_y, color='orange', linewidth=0.5)
        plt.xlim = [0, launchsite.imgwidth]
        plt.ylim = [0, launchsite.imgheight]
        ax = plt.gca()
        ax.set_aspect(1.0)
        plt.imshow(np.asarray(launchsite.img))

        plt.show()        

class LaunchSite:
    def __init__(self):
        self.imgpath = ''
        self.launchpoint = [0, 0]  # [px]
        self.scalefactor = 100.0 / 100.0  # [m/px]
        self.interval = 10.0

    def initialize(self):
        self.img = Image.open(self.imgpath)
        self.imgwidth = self.img.size[0]
        self.imgheight = self.img.size[1]

class NoshiroAsanai3rd(LaunchSite):
    def __init__(self):
        self.imgpath = './pic/NoshiroAsanai3rd.png'
        self.launchpoint = [326, 367]  # [px]
        self.scalefactor = 100.0 / 58.0  # [m/px]
        self.interval = 50.0

class NoshiroOchiai3km(LaunchSite):
    def __init__(self):    
        self.imgpath = './pic/NoshiroOchiai3km.png'
        self.launchpoint = [890, 655]  # [px]
        self.scalefactor = 500.0 / 129.0  # [m/px]
        self.interval = 200.0

class NoshiroOchiai4_5km(LaunchSite):
    def __init__(self):    
        self.imgpath = './pic/NoshiroOchiai4_5km.png'
        self.launchpoint = [900, 658]  # [px]
        self.scalefactor = 1000.0 / 84.0  # [m/px]
        self.interval = 200.0

class TaikiLand(LaunchSite):
    def __init__(self):    
        self.imgpath = './pic/TaikiLand.png'
        self.launchpoint = [208, 483]  # [px]
        self.scalefactor = 200.0 / 70.0  # [m/px]
        self.interval = 50.0

class LandingPointLog:
    def __init__(self,filepath):
        all = np.loadtxt(filepath, delimiter=',')
        # 56行
        row_size = len(all[:,0])
        self.ballistic_x = np.zeros(row_size)
        self.ballistic_y = np.zeros(row_size)
        self.decent_x = np.zeros(row_size)
        self.decent_y = np.zeros(row_size)
        for i in range(row_size):
            self.ballistic_x[i] = all[i, 0]
            self.ballistic_y[i] = all[i, 1]
            self.decent_x[i] = all[i, 2]
            self.decent_y[i] = all[i, 3]

    def shift_point(self, launchsite):
        self.ballistic_x = (self.ballistic_x / launchsite.scalefactor) + launchsite.launchpoint[0]
        self.ballistic_y = -(self.ballistic_y / launchsite.scalefactor) + launchsite.launchpoint[1]
        self.decent_x = (self.decent_x / launchsite.scalefactor) + launchsite.launchpoint[0]
        self.decent_y = -(self.decent_y / launchsite.scalefactor) + launchsite.launchpoint[1]



if __name__ == '__main__':
    plotter = Plotter()
    launchsite = NoshiroOchiai3km()
    launchsite.initialize()
    point = LandingPointLog('sample.csv')
    plotter.Plot(launchsite, point)