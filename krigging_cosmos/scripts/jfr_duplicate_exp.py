#!/usr/bin/env python

import glob
import os

import sys
import yaml


def load_data_from_yaml(filename):
    with open(filename, 'r') as f:
        a = yaml.load(f)
        return a


if __name__ == '__main__':
    print sys.argv
    if len(sys.argv) <= 2:
        print("usage:")
        print("duplicate_exp.py exp_file_to_duplicate number_of_copies")
        sys.exit()
    
    a = load_data_from_yaml(sys.argv[1])
    name = a['name']
    inde = int(name[-1:])
    name = name[:-1]


    ncopies = int(sys.argv[2])
    for i in range(ncopies):
        inde+=1
        nname = name+str(inde)
        fnname = nname+'.exp'
        print inde, nname, fnname
        a['name']=nname
        fdstr=yaml.safe_dump(a, default_flow_style=False)
        print fdstr
        with open(fnname, 'w') as f:
            f.write(fdstr)
            f.close()
