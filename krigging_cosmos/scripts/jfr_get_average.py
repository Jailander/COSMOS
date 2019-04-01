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
    mdn=filename.rstrip('.yaml').split('-')
    b['name']=mdn[-1]
    b['description']=description
    b['dist']=[[],[]]
    b['mse']=[[],[]]
    b['var']=[[],[]]
    for i in a:
        b['dist'][0].append(int(i['exploration_time']))
        b['dist'][1].append(float(i['dist']))
        if i.has_key('results'):
            b['mse'][0].append(int(i['exploration_time']))
            b['mse'][1].append(float(i['results']['groundtruth']['errors']['mse']))
            b['var'][0].append(int(i['exploration_time']))
            b['var'][1].append(float(i['results']['var']['mean']))
    return b


def average_distances(models, field='dist'):
    mint=models[0][field][0][0]
    maxt=models[0][field][0][-1]
    for i in models:
#        print(i['dist'])
        print(i[field][0][0], i[field][0][-1], i[field][1][0], i[field][1][-1])
        if mint>=i[field][0][0]:
            mint=i[field][0][0]
        if maxt<=i[field][0][-1]:
            maxt=i[field][0][-1]

    print(mint, maxt)
    xvals = np.arange(mint, maxt, 200,dtype=np.float32)
    xvals = xvals.tolist()
    xvals.append(float(maxt))
    print(xvals)
    
    mods=[]
    mnm=[]
    for i in models:
        yint = np.interp(xvals, i[field][0], i[field][1])
        mnm.append(i['name'])
        mods.append(yint)

    avs=[]
    astd=[]
    for i in range(len(mods[0])):
        tav=[]
        for j in range(len(mods)):
            tav.append(mods[j][i])
        avs.append(float(np.average(tav)))
        astd.append(float(np.std(tav)))
    print(avs)
    print(astd)
    print(len(avs), len(xvals))


#    nlable=0
    for i in range(len(mods)):
        plt.plot(xvals, mods[i], label=mnm[i])
#        nlable+=1
    
    
    plt.title(field)
    fontP = FontProperties()
    fontP.set_size('small')
    plt.legend(loc="upper left",ncol=2, prop=fontP)
    
    plt.show()


    return [xvals,avs,astd]


if __name__ == '__main__':

    models=[]
    dist=[]
    stdl=[]
    varil=[]
    msel=[]
    errorl=[]
    
    for fil in glob.glob("*.yaml"):
        print(fil)
        aa = load_results(fil, description=fil)
        models.append(aa)
        

    a={}
    a['dist']=average_distances(models, field='dist')
    a['mse']=average_distances(models, field='mse')
    a['var']=average_distances(models, field='var')


    plt.plot(a['mse'][0], a['mse'][1])
    plt.show()
#    print a
    
    fn = str(os.getcwd().split('/')[-2])+'.res'
    
    fdstr=yaml.safe_dump(a, default_flow_style=False)
#    print fdstr
#    fnstr='./'+'average'+'.res'
    with open(fn, 'w') as f:
        f.write(fdstr)
        f.close()
