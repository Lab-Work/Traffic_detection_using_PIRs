"""
this is a live script to see if the PIR sensor ambient temperatures are consistent
it will print asterisks for visualizing the difference in ambient temperature
"""

import sys
sys.path.insert(1, '../../../lib/')
import numpy as np
import os
import time
from mlx90621 import PIRArray
pir = PIRArray(400000)
#np.set_printoptions(linewidth=150)
pir.set_frequency(32)
while True:
    time1 =time.time()
    pirdata = pir.read()
    print str(pir.pir1.Tambient) + "\t" + str(pir.pir2.Tambient) + "\t" + str(pir.pir1.Tambient-pir.pir2.Tambient) + "\t" + int((pir.pir1.Tambient-pir.pir2.Tambient)*20)*"*"
    while time.time()-time1 < 1.0/32:
        pass
    #print pir.calculate_4x16_np(pir.mlx_cshape(pir.mlx_ir_read()), pir.mlx_ptat(), pir.mlx_cp())
    #print (pir.mlx_cshape(pir.mlx_ir_read()))
