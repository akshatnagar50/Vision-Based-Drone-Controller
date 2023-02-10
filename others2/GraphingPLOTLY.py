import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.express as px
import numpy as np

df=pd.read_csv('Bestt hover\Data1676007776.5031767_Kpp_0.5_Kpr_0.5_Kpt_3.5_BESTTTTT.csv')
rc_th=df['rc_th'].to_numpy()
t=df['time step'].to_numpy()
cam_x=df['cam_x'].to_numpy()
est_x = np.array(df['estimated_x'])
est_y = np.array(df['estimated_y'])
cam_y=df['cam_y'].to_numpy()
cam_z=df['cam_z'].to_numpy()
rc_roll = np.array(df['rc_r'])
roll = np.array(df['sen_roll'])
rc_pitch = np.array(df['rc_p'])
pitch = np.array(df['sen_pitch'])
rc_yaw = np.array(df['rc_y'])
yaw = np.array(df['sen_yaw'])
print(roll)
mode = np.array(df['mode'])
detectplot=[]
for i in mode:
  if i=='Controller Default':
    detectplot.append(1200)
  elif i=='Controller Detection':
    detectplot.append(1300)
  elif i=='Controller Memory':
    detectplot.append(1400)
cam_X=np.delete(cam_x,np.where(cam_x==-1000)[0])
# t_x=np.delete(t,np.where(cam_x==-1000)[0])

cam_Y=np.delete(cam_y,np.where(cam_y==-1000)[0])
# t_y=np.delete(t,np.where(cam_y==-1000)[0])

# t_z=np.delete(t,np.where(cam_z==-1000)[0])
# rc_z=np.delete(rc_th,np.where(cam_z==-1000)[0])
cam_Z=np.delete(cam_z,np.where(cam_z==-1000)[0])


fig1 = make_subplots(specs=[[{"secondary_y": True}]])

fig1.add_trace(go.Scatter(x=t, y=rc_th,
                    mode='lines+markers',
                    name='Throttle'),secondary_y=False)

fig1.add_trace(go.Scatter(x=t, y=cam_z,
                    mode='lines+markers',
                    name='Camera Z'),secondary_y=True)

fig1.add_trace(go.Line(x=t, y=detectplot, mode='lines+markers', name='DetectionMode'), secondary_y = False)
fig1.show()

fig2 = go.Figure()
fig2.add_trace(go.Scatter(x=t, y=cam_x,
                    mode='lines+markers',
                    name='Camera X'))
fig2.add_trace(go.Scatter(x=t, y=cam_y,
                   mode='lines+markers',
                    name='Camera Y'))
fig2.show()

fig3 = go.Figure()
fig3.add_trace(go.Scatter(x=cam_X, y=cam_Y, mode='lines+markers', name='XY Plot'))
fig3.show()

fig4 = make_subplots(specs=[[{"secondary_y": True}]])
fig4.add_trace(go.Scatter(x=t, y=rc_roll,
                    mode='lines+markers',
                    name='RCRoll'),secondary_y=False)

fig4.add_trace(go.Scatter(x=t, y=roll,
                    mode='lines+markers',
                    name='SensorRoll'),secondary_y=True)

fig4.add_trace(go.Line(x=t, y=detectplot, mode='lines+markers', name='DetectionMode'), secondary_y = False)
fig4.show()




fig5 = make_subplots(specs=[[{"secondary_y": True}]])
fig5.add_trace(go.Scatter(x=t, y=rc_pitch,
                    mode='lines+markers',
                    name='RCPitch'),secondary_y=False)

fig5.add_trace(go.Scatter(x=t, y=pitch,
                    mode='lines+markers',
                    name='Sensorpitch'),secondary_y=True)

fig5.add_trace(go.Line(x=t, y=detectplot, mode='lines+markers', name='DetectionMode'), secondary_y = False)
fig5.show()

fig6 = make_subplots(specs=[[{"secondary_y": True}]])
fig6.add_trace(go.Scatter(x=t, y=rc_yaw,
                    mode='lines+markers',
                    name='RCYaw'),secondary_y=False)

fig6.add_trace(go.Scatter(x=t, y=yaw,
                    mode='lines+markers',
                    name='SensorYaw'),secondary_y=True)

fig6.add_trace(go.Line(x=t, y=detectplot, mode='lines+markers', name='DetectionMode'), secondary_y = False)
fig6.show()


# ----------------------------------------------------------
for i in range(0,len(cam_x)):
    if cam_x[i]==-1000:
        cam_x[i] = est_x[i]
    
    if cam_y[i]==-1000:
        cam_y[i] = est_y[i]
fig7 = px.scatter_3d(x = cam_x, y = cam_y, z = cam_z)
fig7.show()
# ----------------------------------------------------------

fig8 = make_subplots(specs=[[{"secondary_y": True}]])
fig8.add_trace(go.Scatter(x=t, y=cam_x,
                    mode='lines+markers',
                    name='Cam_X'),secondary_y=False)

fig8.add_trace(go.Scatter(x=t, y=est_x,
                    mode='lines+markers',
                    name='Estimated_X'),secondary_y=False)

fig8.add_trace(go.Line(x=t, y=detectplot, mode='lines+markers', name='DetectionMode'), secondary_y = False)
fig8.show()

fig9 = make_subplots(specs=[[{"secondary_y": True}]])
fig9.add_trace(go.Scatter(x=t, y=cam_y,
                    mode='lines+markers',
                    name='Cam_Y'),secondary_y=False)

fig9.add_trace(go.Scatter(x=t, y=est_y,
                    mode='lines+markers',
                    name='Estimated_Y'),secondary_y=False)

fig9.add_trace(go.Line(x=t, y=detectplot, mode='lines+markers', name='DetectionMode'), secondary_y = False)
fig9.show()
