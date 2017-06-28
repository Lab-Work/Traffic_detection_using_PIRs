from PIRdriver import *
import time 
from time import sleep

pir = PIRSensor(1,400000)
pir.mlx_init()
pir.const_init()
pir.mlx_set_meas_mode(1)
irread = pir.mlx_ir_read()

# time for mlx to change

time1=time.time()
pir.mlx_start_read()
while pir.mlx_ir_read()==irread:
    pass
time2=time.time()
print time2-time1

time.sleep(1)

time3=time.time()
pir.mlx_start_read()
while not pir.mlx_meas_done():
    pass
time4=time.time()
print time4-time3
print
print
print 32
pir.mlx_set_frequency(32)
time5=time.time()
p = pir.mlx_ir_read()
time6=time.time()
print time6-time5
print 
print 64
pir.mlx_set_frequency(64)
time5=time.time()
p = pir.mlx_ir_read()
time6=time.time()
print time6-time5
print
print 128
pir.mlx_set_frequency(128)
time5=time.time()
p = pir.mlx_ir_read()
time6=time.time()
print time6-time5
print
print 256
pir.mlx_set_frequency(256)
time5=time.time()
p = pir.mlx_ir_read()
time6=time.time()
print time6-time5
print
print "repeat test 8"
pir.mlx_set_frequency(8)
x = pir.mlx_ir_read()
sum = 0
for i in range(10):
    time1=time.time()
    while pir.mlx_ir_read()==x:
        pass
    time2=time.time()
    sum = sum + time2 - time1
    x=pir.mlx_ir_read()
print "avg",  sum/10

print "repeat test 64"
pir.mlx_set_frequency(64)
x = pir.mlx_ir_read()
sum = 0
for i in range(100):
    time1=time.time()
    while pir.mlx_ir_read()==x:
        pass
    time2=time.time()
    sum = sum + time2 - time1
    x=pir.mlx_ir_read()
print "avg",  sum/100
st = sum/100

print "repeat test 128"
pir.mlx_set_frequency(128)
x = pir.mlx_ir_read()
sum = 0
for i in range(100):
    time1=time.time()
    while pir.mlx_ir_read()==x:
        pass
    time2=time.time()
    sum = sum + time2 - time1
    x=pir.mlx_ir_read()
print "avg", sum/100
print
print "optimal time test 64"
pir.mlx_set_frequency(64)
st=0.018
num=1
x=pir.mlx_ir_read()
count = 0
while count<5:
    num=0
    for i in range(100):
        time.sleep(st)
        y=pir.mlx_ir_read()
        if x==y:
            num+=1
        x=y
    print "%f, %d" % (st, num)
    if num==0:
        count+=1
    st+=0.00003

print "final set", st
        
        
        
