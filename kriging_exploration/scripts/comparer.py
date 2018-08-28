#!/usr/bin/env python

import yaml
import numpy as np

import argparse

import matplotlib.pyplot as plt

from scipy import spatial
#from matplotlib.ticker import MaxNLocator
#import matplotlib as mpl
#import matplotlib.cm as cm

#import rospy


#import satellite
#from kriging_exploration.satellite import SatelliteImage
#from kriging_exploration.map_coords import MapCoords
from kriging_exploration.data_grid import DataGrid
#from krigging_data import KriggingData



def data_to_yaml(data_fn):
    mods = {}
#    mods['names']=['0.0 cm', '2.5 cm' ,'5.0 cm', '7.5 cm', '10.0 cm', '12.5 cm', '15.0 cm', '17.5 cm', '20.0 cm', '22.5 cm' ,'25.0 cm', '27.5 cm', '30.0 cm', '32.5 cm', '35.0 cm', '37.5 cm', '40.0 cm', '42.5 cm', '45.0 cm']
    mods['data']=[]
#    vals=[]
    print "open: " + data_fn
    f = open(data_fn, 'r')
    for line in f:
        line=line.strip('\n')
        a=line.split(';')
        val={}
        val['position']={'lat':float(a[0]), 'lon':float(a[1])}
        val['data']= [float(i) for i in a[2:]]
        mods['data'].append(val)
    
    print mods['data']
    yml = yaml.safe_dump(mods, default_flow_style=False)
    print yml
    filename = data_fn+'.yaml'
    fh = open(filename, "w")
    s_output = str(yml)
    fh.write(s_output)
    fh.close


class comparer(object):

    def __init__(self, cell_size):
        self.running = True
        
        self.grid = DataGrid('limits.coords', cell_size)
        self.grid2 = DataGrid('limits.coords', cell_size)
        
        #self.load_groundtruth('Iains2.yaml')
        #self.load_groundtruth('bottom_testing.yaml')
        #self.load_groundtruth('upper_testing.yaml')
        self.load_groundtruth()
        
        self.krieg_all_mmodels()
        self.grid.calculate_mean_grid()
        self.grid2.calculate_mean_grid()
        self.model_comparison()
        self.correlate()


    def model_comparison(self):
        names=[]
        means=[]
        stds=[]        


        diff = self.grid.mean_output-self.grid2.mean_output

        print "mean", np.mean(diff), np.sqrt(np.mean(diff**2)), np.std(diff), np.var(diff)
        names.append('mean')
#        means.append(np.abs(np.mean(diff)))
#        stds.append(np.abs(np.std(diff))/2.0)
        means.append(np.abs(np.mean(self.grid.mean_output)))
        stds.append(np.abs(np.sqrt(np.mean(diff**2))/2.0))

        #np.save('mean.csv', diff)
        np.savetxt("mean.csv", diff, delimiter=",")
        
        
        for i in range(len(self.grid.models)):
            diff = self.grid.models[i].output-self.grid2.models[i].output
            print i, np.mean(diff), np.sqrt(np.mean(diff**2)), np.std(diff), np.var(diff)
            nstr= str(i*5)+'-'+str((i+1)*5)
            names.append(nstr)
            #means.append(np.abs(np.mean(diff)))
            #stds.append(np.abs(np.std(diff)/2.0))
            means.append(np.abs(np.mean(self.grid.models[i].output)))
            stds.append(np.abs(np.sqrt(np.mean(diff**2))/2.0))
            np.savetxt(nstr, diff, delimiter=",")
            
        self.graph_layers(names, means, stds)
        #self.graph_layers(names, means, stds)


    def correlate(self):
#        corrcoef=[]
#
        g1np = [] #np.asarray(self.grid.mean_output)
        g2np = [] #np.asarray(self.grid2.mean_output)

        for i in self.compare_inds:
            g1np.append(self.grid.mean_output[i[1]][i[0]])
            g2np.append(self.grid2.mean_output[i[1]][i[0]])
        
#        g1np=np.random.rand(20)
#        g2np=np.random.rand(20)
        
#        ft1= np.fft.fft(np.asarray(g1np))
#        ft2= np.conjugate(np.fft.fft(np.asarray(g2np)))
        ccf= np.corrcoef(np.asarray(g1np), np.asarray(g2np))
        
#        res = np.abs(np.dot(ft1,ft2))
#        res2 = np.dot(ft1,np.conjugate(ft1))
        print ccf#, res, res/res2

        for i in range(len(self.grid.models)):
            g1np = [] #np.asarray(self.grid.mean_output)
            g2np = [] #np.asarray(self.grid2.mean_output)
    
            for j in self.compare_inds:
                g1np.append(self.grid.models[i].output[j[1]][j[0]])
                g2np.append(self.grid2.models[i].output[j[1]][j[0]])


            g1np =np.asarray(g1np)
            g2np =np.asarray(g2np)
            ccf= np.corrcoef(g1np,g2np)
            cos_sim = np.dot(g1np, g2np)/(np.linalg.norm(g1np)*np.linalg.norm(g2np))
            print i, ccf[0][1], cos_sim
            x = np.arange(0,len(g1np))
            plt.cla()
            plt.plot(x, g1np, color='b', label='Auto')
            plt.plot(x, g2np, color='g', label='Manual')
            plt.savefig(self.grid.models[i].name+'.png')
#        print g1np
#        print '----'
#        print g2np



#        
#        #ccf= np.corrcoef(g1np.reshape(np.size(g1np)), g2np.reshape(np.size(g2np)))
        #ccf= np.corrcoef(g1np.flatten(), g2np.flatten())
#        
#        print "CCF: ", ccf      
#        print "++++++++++++++"
#        print ccf[0, 1]
#        print "++++++++++++++"
#
#        for i in range(len(self.grid.models)):
#            g1np = np.asarray(self.grid.models[i].output)
#            
#            #g1np = g1np/np.max(g1np)
#            g2np = np.asarray(self.grid2.models[i].output)
#            #g2np = g2np/np.max(g2np)
#            g1np= g1np.flatten()
#            g2np= g2np.flatten()
#
#            ccf= np.corrcoef(g1np, g2np)
#
#            print "++++++++++++++"
#            #filename2 = "Manual-layer-"+str(i)+".csv"
#            #filename1 = "Auto-layer-"+str(i)+".csv"
#
#            #np.savetxt(filename1, g1np, delimiter=",", newline="\n")
#            #np.savetxt(filename2, g2np, delimiter=",", newline="\n")
#            print i, ccf#[0, 1]
#            print "++++++++++++++"
#
#
#            
        x = np.arange(0,len(g1np))
#            #y = np.asarray(vals)
#            #e = np.asarray(stds)
#            tit="data_for_layer_"+str(i+1)+'.png'
#            plt.title(tit)
        plt.cla()
        plt.plot(x, g1np, color='b', label='Auto')
        plt.plot(x, g2np, color='g', label='Manual')
#            plt.legend()
#            plt.savefig(tit)
#            plt.cla()
        plt.show()




    def graph_layers(self, names, means, stds):
        axis_font = {'fontname':'Bitstream Vera Sans', 'size':'18'}
        labels_font = {'fontname':'Bitstream Vera Sans', 'size':'14'}
        title_font = {'fontname':'Bitstream Vera Sans', 'size':'24', 'color':'black', 'weight':'normal',
              'verticalalignment':'bottom'}
        fig, ax = plt.subplots()        

        index = np.arange(len(names))
        bar_width = 0.4
        
        opacity = 0.7
        error_config = {'ecolor': '0.1'}
        
        rects1 = ax.bar(index, means, bar_width,
                        alpha=opacity, color='b',
                        yerr=stds, error_kw=error_config,
                        label='errors')
        
#        rects2 = ax.bar(index + bar_width, means_women, bar_width,
#                        alpha=opacity, color='r',
#                        yerr=std_women, error_kw=error_config,
#                        label='Women')
        
        ax.set_xlabel('Layers', **axis_font)
        ax.set_ylabel('Mean Values', **axis_font)
        ax.set_title('Mean by Layers', **title_font)
        ax.set_xticks(index + bar_width/2)
        ax.set_xticklabels(names, **labels_font)
        #ax.legend()
        
        print "!!!!"
        for i in range(len(means)):
            
            print stds[i]/means[i], means[i]

         
        
        fig.tight_layout()
        plt.savefig('comparison.png')
        #plt.show()
        

    def load_groundtruth(self):
        self.compare_inds=[]
        self.grid.load_data_from_yaml('testing-5cm-intervals.yaml')
        self.grid2.load_data_from_yaml('b.yaml')


        for j in self.grid2.models[0].orig_data:
            self.compare_inds.append((j.x, j.y))
        
        #print self.compare_inds
            


    def krieg_all_mmodels(self):
        for i in self.grid.models:
            i.do_krigging()

        for i in self.grid2.models:
            i.do_krigging()
            

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--cell_size", type=int, default=10,
                        help="cell size in meters")
    args = parser.parse_args()
    
#    rospy.init_node('kriging_simulator')

    comparer(args.cell_size)  #Half cosmos field
#    simulator(53.261685, -0.525158, 17, 640, args.cell_size)  #Full cosmos field

