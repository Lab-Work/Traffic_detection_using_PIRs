import plotly.plotly as py
import plotly.graph_objs as go
import numpy as np
import plotly.tools as tls
import time
import datetime
import random
from sensor_classes import PIR_Array, ADC

username = 'modrie'
api_key = 'mt2vyxp6mu'
stream_token_pir = 'o4f0yesgv6'
stream_token_ult = 'mm2tpu7odu'
time.sleep(10)
py.sign_in(username, api_key)

pir = PIR_Array(400000)
ult = ADC(400000)

pir.set_frequency(16)

map1 = go.Heatmap(
        z=[],
        zmin=24,
        zmax=35,
        stream=dict(
                token=stream_token_pir,
                maxpoints=1
        )
)
sca1 = go.Scatter(
        x=[],
        y=[],
        stream=dict(
                token=stream_token_ult,
                maxpoints=100
        )
)
layout = go.Layout(
    title='PIR and Ultrasonic data'
)

#fig = go.Figure(data=[map1,sca1], layout=layout)
fig = tls.make_subplots(rows=2,cols=1)
fig.append_trace(map1,1,1)
fig.append_trace(sca1,2,1)
print py.plot(fig, filename='Raspberry Pi Streaming Example Values')

i = 0
stream_pir = py.Stream(stream_token_pir)
stream_pir.open()

stream_ult = py.Stream(stream_token_ult)
stream_ult.open()

#the main sensor reading loop
while True:
        #temps = pir.calculate_4x16_np(pir.mlx_cshape(pir.mlx_ir_read()),pir.mlx_ptat(),pir.mlx_cp())
        temps = pir.read()
        raw=ult.read(sps=3300)
        scale = 4.33/0.984
        val = raw*scale
        print val
        #temps = pir.mlx_cshape(pir.mlx_ir_read())
        stream_pir.write({'type':'heatmap','z': temps})
        stream_ult.write({'type':'scatter','x': datetime.datetime.now(), 'y':val})
        #i += 1
        # delay between stream posts
        time.sleep(1.0/16)

