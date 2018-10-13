#!/usr/bin/env python

import glob
import os

import cv2
import sys
import yaml

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.font_manager import FontProperties



def load_data_from_yaml(filename):
    with open(filename, 'r') as f:
        a = yaml.load(f)
        return a


def load_results(filename, description=''):
    b={}
    a = load_data_from_yaml(filename)
    b['description']=description
    b['data']=a
    return b


def plot_distance(models):
    cols=['b', 'g', 'r', 'c', 'm', 'y', 'k']
    colcount=0
    
    for i in models:
        print i['description']
        plt.plot(i['data']['dist'][0], i['data']['dist'][1], label=i['description'], color=cols[colcount])    

        plt.fill_between(i['data']['dist'][0],np.asarray(i['data']['dist'][1])-np.asarray(i['data']['dist'][2]), 
                         np.asarray(i['data']['dist'][1])+np.asarray(i['data']['dist'][2]), alpha=0.1, color=cols[colcount])
        colcount+=1
        if colcount >= len(cols):
            colcount=0
 
    
    plt.xlabel('time (secs)')
    plt.ylabel('distance')
    
    plt.title("Distance")

    fontP = FontProperties()
    fontP.set_size('small')
    plt.legend(loc='upper left', ncol=1, prop=fontP)
    #plt.legend(ncol=1, prop=fontP)
    
    plt.savefig("fix_asi_dist.pdf")
    
    plt.show()


def plot_mse(models):
    cols=['b', 'g', 'r', 'c', 'm', 'y', 'k']
    colcount=0

    for i in models:
        plt.plot(i['data']['mse'][0], i['data']['mse'][1], label=i['description'], color=cols[colcount]) 
        plt.fill_between(i['data']['mse'][0],np.asarray(i['data']['mse'][1])-np.asarray(i['data']['mse'][2]), 
                         np.asarray(i['data']['mse'][1])+np.asarray(i['data']['mse'][2]), alpha=0.1, color=cols[colcount])
        colcount+=1
        if colcount >= len(cols):
            colcount=0
    
    plt.xlabel('time (secs)')
    plt.ylabel('mse')
    
    plt.title("mse")

    fontP = FontProperties()
    fontP.set_size('small')
    plt.legend(loc="upper right",ncol=1, prop=fontP)
    
    plt.savefig("fix_asi_mse.pdf")
    
    plt.show()


def plot_var(models):
    cols=['b', 'g', 'r', 'c', 'm', 'y', 'k']
    colcount=0
    for i in models:
        plt.plot(i['data']['var'][0], i['data']['var'][1], label=i['description'], color=cols[colcount])
        plt.fill_between(i['data']['var'][0],np.asarray(i['data']['var'][1])-np.asarray(i['data']['var'][2]), 
                         np.asarray(i['data']['var'][1])+np.asarray(i['data']['var'][2]), alpha=0.1, color=cols[colcount])
        colcount+=1
        if colcount >= len(cols):
            colcount=0
    
    plt.xlabel('time (secs)')
    plt.ylabel('var')
    
    plt.title("Kriging Variance")

    fontP = FontProperties()
    fontP.set_size('small')
    plt.legend(loc="upper right", ncol=1, prop=fontP)
    
    plt.savefig("fix_asi_var.pdf")
    
    plt.show()



if __name__ == '__main__':

    models=[]
    
    for fil in glob.glob("*.res"):
        print(fil)
        #aa = load_results(fil, description=fil)
        aa = load_results(fil, description=fil)
        #print aa['data']['var']
        models.append(aa)
    
    plot_distance(models)
    plot_mse(models)
    plot_var(models)

