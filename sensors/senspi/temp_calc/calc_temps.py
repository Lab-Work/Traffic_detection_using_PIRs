"""
This script is used to post process temperature calculations for collected datasets.
"""

#Variables

filenames = ['128_300_0_150821.npy']

#Imports

import sys
sys.path.insert(0, '../../lib/')
import time
import os #os.path.isfile()
import numpy as np
from mlx90621 import PIRArray

#Script
def main():
    pir = PIRArray(400000)
    for f in filenames:
        data1 = []
        data2 = []
        fi = np.load("RAW/"+f)
        for entry in fi:
            print f + " " + str(entry[0])
            data1.append([entry[0],pir.pir1.calculate_4x16_np(entry[1][0][0],entry[1][1][0],entry[1][2][0])])
            data2.append([entry[0],pir.pir2.calculate_4x16_np(entry[1][0][1],entry[1][1][1],entry[1][2][1])])
        np.save("RAW/calc1/"+f[0:-4]+"p1",data1)
        np.save("RAW/calc2/"+f[0:-4]+"p2",data2)
        fi = []
        data1=[]
        data2=[]





main()
