import dash
import dash_bootstrap_components as dbc
from dash import Input, Output, dcc, html
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.express as px
import numpy as np

app = dash.Dash(external_stylesheets=[dbc.themes.SKETCHY])

# the style arguments for the sidebar. We use position:fixed and a fixed width
SIDEBAR_STYLE = {
    "position": "fixed",
    "top": 0,
    "left": 0,
    "bottom": 0,
    "width": "16rem",
    "padding": "2rem 1rem",
    "background-color": "#f8f9fa",
}

# the styles for the main content position it to the right of the sidebar and
# add some padding.
CONTENT_STYLE = {
    "margin-left": "18rem",
    "margin-right": "2rem",
    "padding": "2rem 1rem",
}

sidebar = html.Div(
    [
        html.H2("Drona", className="display-4"),
        html.Hr(),
        html.P(
            "Plots for the data collected from the drone", className="lead"
        ),
        dbc.Nav(
            [
                dbc.NavLink("Drone Data", href="/", active="exact"),
                dbc.NavLink("XYZ plots", href="/page-1", active="exact"),
                dbc.NavLink("Page 2", href="/page-2", active="exact"),
            ],
            vertical=True,
            pills=True,
        ),
    ],
    style=SIDEBAR_STYLE,
)

content = html.Div(id="page-content", style=CONTENT_STYLE)

app.layout = html.Div([dcc.Location(id="url"), sidebar, content])
# df=pd.read_csv('Data1676027365.0775366_Kpp_0.25_Kpr_0.25_Kpt_3.5.csv')
# rc_th=df['rc_th'].to_numpy()
# t=df['time step'].to_numpy()
# cam_x=df['cam_x'].to_numpy()
# est_x = np.array(df['estimated_x'])
# est_y = np.array(df['estimated_y'])
# cam_y=df['cam_y'].to_numpy()
# cam_z=df['cam_z'].to_numpy()
# rc_roll = np.array(df['rc_r'])
# roll = np.array(df['sen_roll'])
# rc_pitch = np.array(df['rc_p'])
# pitch = np.array(df['sen_pitch'])
# rc_yaw = np.array(df['rc_y'])
# yaw = np.array(df['sen_yaw'])
# print(roll)
# mode = np.array(df['mode'])
# detectplot=[]
# for i in mode:
#   if i=='Controller Default':
#     detectplot.append(1200)
#   elif i=='Controller Detection':
#     detectplot.append(1300)
#   elif i=='Controller Memory':
#     detectplot.append(1400)
# cam_X=np.delete(cam_x,np.where(cam_x==-1000)[0])
# # t_x=np.delete(t,np.where(cam_x==-1000)[0])

# cam_Y=np.delete(cam_y,np.where(cam_y==-1000)[0])
# # t_y=np.delete(t,np.where(cam_y==-1000)[0])

# # t_z=np.delete(t,np.where(cam_z==-1000)[0])
# # rc_z=np.delete(rc_th,np.where(cam_z==-1000)[0])
# cam_Z=np.delete(cam_z,np.where(cam_z==-1000)[0])


# fig1 = make_subplots(specs=[[{"secondary_y": True}]])

# fig1.add_trace(go.Scatter(x=t, y=rc_th,
#                     mode='lines+markers',
#                     name='Throttle'),secondary_y=False)

# fig1.add_trace(go.Scatter(x=t, y=cam_z,
#                     mode='lines+markers',
#                     name='Camera Z'),secondary_y=True)

# fig1.add_trace(go.Scatter(x=t, y=detectplot, mode='lines+markers', name='DetectionMode'), secondary_y = False)
# #fig1.show()
# fig1.update_layout(
#     xaxis_title='Time (s)',
#     yaxis_title='RC Throttle (deg)',
    
# )

# fig2 = go.Figure()
# fig2.add_trace(go.Scatter(x=t, y=cam_x,
#                     mode='lines+markers',
#                     name='Camera X'))
# fig2.add_trace(go.Scatter(x=t, y=cam_y,
#                    mode='lines+markers',
#                     name='Camera Y'))
# #fig2.show()
# fig2.update_layout(
#     xaxis_title='Time (s)',
#     yaxis_title='Camera X (m)',
    
# )

# fig3 = go.Figure()
# fig3.add_trace(go.Scatter(x=cam_X, y=cam_Y, mode='lines+markers', name='XY Plot'))
# #fig3.show()
# fig3.update_layout(
#     xaxis_title='X (m)',
#     yaxis_title='Y (m)',
#     title='Camera XY Plot'
# )

# fig4 = make_subplots(specs=[[{"secondary_y": True}]])
# fig4.add_trace(go.Scatter(x=t, y=rc_roll,
#                     mode='lines+markers',
#                     name='RCRoll'),secondary_y=False)

# fig4.add_trace(go.Scatter(x=t, y=roll,
#                     mode='lines+markers',
#                     name='SensorRoll'),secondary_y=True)

# fig4.add_trace(go.Scatter(x=t, y=detectplot, mode='lines+markers', name='DetectionMode'), secondary_y = False)
# #fig4.show()
# fig4.update_layout(
#     xaxis_title='Time (s)',
#     yaxis_title='RC Roll (deg)',
    
# )




# fig5 = make_subplots(specs=[[{"secondary_y": True}]])
# fig5.add_trace(go.Scatter(x=t, y=rc_pitch,
#                     mode='lines+markers',
#                     name='RCPitch'),secondary_y=False)

# fig5.add_trace(go.Scatter(x=t, y=pitch,
#                     mode='lines+markers',
#                     name='Sensorpitch'),secondary_y=True)

# fig5.add_trace(go.Scatter(x=t, y=detectplot, mode='lines+markers', name='DetectionMode'), secondary_y = False)
# #fig5.show()
# fig5.update_layout(
#     xaxis_title='Time (s)',
#     yaxis_title='RC Pitch (deg)',
   
# )


# fig6 = make_subplots(specs=[[{"secondary_y": True}]])
# fig6.add_trace(go.Scatter(x=t, y=rc_yaw,
#                     mode='lines+markers',
#                     name='RCYaw'),secondary_y=False)

# fig6.add_trace(go.Scatter(x=t, y=yaw,
#                     mode='lines+markers',
#                     name='SensorYaw'),secondary_y=True)

# fig6.add_trace(go.Scatter(x=t, y=detectplot, mode='lines+markers', name='DetectionMode'), secondary_y = False)
# #fig6.show()
# fig6.update_layout(
#     xaxis_title='Time (s)',
#     yaxis_title='RC Yaw (deg)',
    
# )

# # ----------------------------------------------------------
# for i in range(1,len(cam_x)):
#     if cam_x[i]==-1000:
#         cam_x[i] = est_x[i]
    
#     if cam_y[i]==-1000:
#         cam_y[i] = est_y[i]
#     if cam_z[i]==-1000:
#         cam_x[i] = 0
#         cam_y[i] = 0
#         cam_z[i] = 0
# fig7 = px.scatter_3d(x = cam_x, y = cam_y, z = cam_z)
# #fig7.show()
# fig7.update_layout(
#     xaxis_title='X (m)',
#     yaxis_title='Y (m)',
#     title='Camera XYZ Plot'
# )
# # ----------------------------------------------------------

# fig8 = make_subplots(specs=[[{"secondary_y": True}]])
# fig8.add_trace(go.Scatter(x=t, y=cam_x,
#                     mode='lines+markers',
#                     name='Cam_X'),secondary_y=False)

# fig8.add_trace(go.Scatter(x=t, y=est_x,
#                     mode='lines+markers',
#                     name='Estimated_X'),secondary_y=False)

# fig8.add_trace(go.Scatter(x=t, y=detectplot, mode='lines+markers', name='DetectionMode'), secondary_y = False)
# #fig8.show()\
# fig8.update_layout(
#     xaxis_title='Time (s)',
#     yaxis_title='RC Throttle (deg)',
    
# )

# fig9 = make_subplots(specs=[[{"secondary_y": True}]])
# fig9.add_trace(go.Scatter(x=t, y=cam_y,
#                     mode='lines+markers',
#                     name='Cam_Y'),secondary_y=False)

# fig9.add_trace(go.Scatter(x=t, y=est_y,
#                     mode='lines+markers',
#                     name='Estimated_Y'),secondary_y=False)

# fig9.add_trace(go.Scatter(x=t, y=detectplot, mode='lines+markers', name='DetectionMode'), secondary_y = False)
# #fig9.show()
# fig9.update_layout(
#     xaxis_title='Time (s)',
#     yaxis_title='RC Throttle (deg)',
    
# )


df=pd.read_csv('Data1676027365.0775366_Kpp_0.25_Kpr_0.25_Kpt_3.5.csv')
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
mode = np.array(df['mode'])
detectplot=[]
i=[]
j=[]
k=[]
for r in range(0,len(mode)):
  if mode[r]=='Controller Default':
    i.append(r)
  elif mode[r]=='Controller Detection':
    j.append(r)
  elif mode[r]=='Controller Memory':
    k.append(r)
cam_X=np.delete(cam_x,np.where(cam_x==-1000)[0])
# t_x=np.delete(t,np.where(cam_x==-1000)[0])

cam_Y=np.delete(cam_y,np.where(cam_y==-1000)[0])
# t_y=np.delete(t,np.where(cam_y==-1000)[0])

# t_z=np.delete(t,np.where(cam_z==-1000)[0])
# rc_z=np.delete(rc_th,np.where(cam_z==-1000)[0])
cam_Z=np.delete(cam_z,np.where(cam_z==-1000)[0])

t1=t[i]
t2=t[j]
t3=t[k]

figures=[]

def modefn(data,sen,title,sec):
   d1=data[i]
   d2=data[j]
   d3=data[k]
   fig = make_subplots(specs=[[{"secondary_y": True}]])
   fig.add_trace(go.Scatter(x=t1, y=d1,
                    mode='markers',
                    name='Default'),secondary_y=False)

   fig.add_trace(go.Scatter(x=t2, y=d2,
                    mode='markers',
                    name='Detection'),secondary_y=False)
   fig.add_trace(go.Scatter(x=t3, y=d3,
                    mode='markers',
                    name='Memory'),secondary_y=False)
   fig.add_trace(go.Scatter(x=t, y=sen,
                    mode='lines+markers',
                    name=sec),secondary_y=True)
   fig.update_layout(xaxis_title='Time',
                     title=title)
   return fig

figures.append(modefn(rc_th,cam_z,'Throttle','Camera'))
fig1 = make_subplots(specs=[[{"secondary_y": True}]])

fig1.add_trace(go.Scatter(x=t, y=rc_th,
                    mode='lines+markers',
                    name='Throttle'),secondary_y=False)

fig1.add_trace(go.Scatter(x=t, y=cam_z,
                    mode='lines+markers',
                    name='Camera Z'),secondary_y=True)

fig1.add_trace(go.Scatter(x=t, y=detectplot, mode='lines+markers', name='DetectionMode'), secondary_y = False)
#fig1.show()

fig2 = go.Figure()
fig2.add_trace(go.Scatter(x=t, y=cam_x,
                    mode='lines+markers',
                    name='Camera X'))
fig2.add_trace(go.Scatter(x=t, y=cam_y,
                   mode='lines+markers',
                    name='Camera Y'))
fig2.update_layout(xaxis_title='Camera-X', yaxis_title='Camera-Y', title='Camera X and Y ')
#fig2.show()

fig3 = go.Figure()
fig3.add_trace(go.Scatter(x=cam_X, y=cam_Y, mode='lines+markers', name='XY Plot'))
fig3.update_layout(xaxis_title='X', yaxis_title='Y', title='XY Plot')
#fig3.show()

#fig4.show()
figures.append(modefn(rc_roll,roll,'Roll','Sensor'))
figures.append(modefn(rc_pitch,pitch,'Pitch','Sensor'))
figures.append(modefn(rc_yaw,yaw,'Yaw','Sensor'))


# ----------------------------------------------------------
for p in range(0,len(cam_x)):
    if cam_x[p]==-1000:
        cam_x[p] = est_x[p]
    
    if cam_y[p]==-1000:
        cam_y[p] = est_y[p]
fig7 = px.scatter_3d(x = cam_x, y = cam_y, z = cam_z)
#fig7.show()
# ----------------------------------------------------------

figures.append(modefn(cam_y,est_y,'Coordinate Y','Estimated Y'))
figures.append(modefn(cam_x,est_x,'Coordinate X','Estimated X'))

@app.callback(Output("page-content", "children"), [Input("url", "pathname")])
def render_page_content(pathname):
    if pathname == "/":
        return html.Div([
     html.H1('Drone Data', style={'text-align': 'center'}),
    html.Div([
        html.Div([
            dcc.Graph(id = 'fig1', figure=figures[0])
        ], className='six columns'),
        html.Div([
            dcc.Graph(id = 'fig2',figure=fig2)
        ], className='six columns'),
    ], className='row'),
    html.Div([
        html.Div([
            dcc.Graph(id = 'fig3',figure=fig3)
        ], className='six columns'),
        html.Div([
            dcc.Graph(id = 'fig4',figure=figures[1])
        ], className='six columns'),
    ], className='row'),
    html.Div([
        html.Div([
            dcc.Graph(id = 'fig5',figure=figures[2])
        ], className='six columns'),
        html.Div([
            dcc.Graph(id = 'fig6',figure=figures[3])
        ], className='six columns'),
    ], className='row'),
    html.Div([
        html.Div([
            dcc.Graph(id = 'fig9',figure=figures[4])
        ], className='six columns')
    ], className='row'),
    html.Div([
        html.Div([
            dcc.Graph(id = 'fig9',figure=figures[5])
        ], className='six columns')
    ], className='row'),
])
    elif pathname == "/page-1":
        return html.Div([
            html.H1('XYZ Data', style={'text-align': 'center'}),
        html.Div([
            dcc.Graph(id = 'fig7',figure=fig7)
        ], className='six columns')
    ], className='row'),
    elif pathname == "/page-2":
        return html.P("Oh cool, this is page 2!")
    # If the user tries to reach a different page, return a 404 message
    return html.Div(
        [
            html.H1("404: Not found", className="text-danger"),
            html.Hr(),
            html.P(f"The pathname {pathname} was not recognised..."),
        ],
        className="p-3 bg-light rounded-3",
    )


if __name__ == "__main__":
    app.run_server(port=8888)