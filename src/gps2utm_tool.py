
from pyproj import Proj
import numpy as np

def xy2utm(x0,y0,utm_N0,utm_W0,x,y):
    utm_N=utm_N0+x-x0
    utm_W=utm_W0+y-y0
    return utm_N,utm_W

def JDWD2utm(JD,WD):
    '''
    JD=longitude
    WD=latitude
    '''
    p = Proj(proj='utm',zone=51,ellps='WGS84', preserve_units=False)
    E,N = p(JD, WD)
    utm_N=N
    utm_W=-E
    return utm_N,utm_W


tree_data_1900=[[2.5,12.5,0.5],[3.5,5.0,0.5],[4.0,-1.0,0.5],[9.0,10.0,0.5],[9.5,6.0,0.5],[10.0,3.0,0.5],[13.5,8.0,0.5]]
tree_data_1900_utm=[[2767710.38, -352850.58, 0.5], [2767711.38, -352858.08, 0.5], [2767711.88, -352864.08, 0.5], [2767716.88, -352853.08, 0.5], [2767717.38, -352857.08, 0.5], [2767717.88, -352860.08, 0.5], [2767721.38, -352855.08, 0.5]]
in_data=tree_data_1900
out_data=[]
for i in range(7):
    x0=-0.02
    y0=-7.84
    utm_N0=2767707.86
    utm_W0=-352870.92
    x=in_data[i][0]
    y=in_data[i][1]
    n,w=xy2utm(x0,y0,utm_N0,utm_W0,x,y)
    o=[np.round(n,2),np.round(w,2),in_data[i][2]]
    out_data.append(o)

print(out_data)

# =================
tree_data_1726=[[-4.58,25.74,0.5],[-3.27,19.23,0.5],[-3.36,11.97,0.5],[2.9,23.56,0.5],[2.6,19.29,0.5],[3.15,16.94,0.5],[7.27,21.5,0.5]]
tree_data_1726_utm=[[2767711.3, -352849.18, 0.5], [2767712.61, -352855.69, 0.5], [2767712.52, -352862.95, 0.5], [2767718.78, -352851.36, 0.5], [2767718.48, -352855.63, 0.5], [2767719.03, -352857.98, 0.5], [2767723.15, -352853.42, 0.5]]
in_data=tree_data_1726
out_data=[]
for i in range(7):
    x0=0.19
    y0=0.38
    utm_N0=2767716.07
    utm_W0=-352874.54
    x=in_data[i][0]
    y=in_data[i][1]
    n,w=xy2utm(x0,y0,utm_N0,utm_W0,x,y)
    o=[np.round(n,2),np.round(w,2),in_data[i][2]]
    out_data.append(o)

print(out_data)


tree_data_wow=np.load("center_list_all(1122).npy")


# for python code

print("------------------python---------")
s="["
for i in tree_data_wow:
    s=s+"["+str(i[1])+",-"+str(i[0])+","+str(i[2])+"],"
s=s[:-1]+"]"
print(s)
print("------------------python---------")
print("------------------matlab---------")
# for matlab code
s="["
for i in tree_data_wow:
    s=s+str(i[1])+",-"+str(i[0])+","+str(i[2])+";"
s=s[:-1]+"]"
print(s)

print("------------------matlab---------")



























