import numpy as np
import sys
import rospy
print("[TREEDATA.py] Start to get the trunk map data")
X_MIN,X_MAX,Y_MIN,Y_MAX=0,1,0,1
R_MAX=0
R_MIN=10
try:
    file_path=rospy.get_param("date_time_folder",default="20220110_17-32-20")+"/shapefiles/neg/center_all.npy"
    
    print("[TREEDATA.py] The trunk map data at "+file_path)
    tree_data_wow=np.load(file_path)
    X_MIN=tree_data_wow[0][1]
    X_MAX=tree_data_wow[0][1]
    Y_MIN=tree_data_wow[0][0]*(-1.0)
    Y_MAX=tree_data_wow[0][0]*(-1.0)
    TREE_DATA=[]
    for i in tree_data_wow:
        TREE_DATA.append([i[1],i[0]*(-1.0),i[2]])
        print("[TREEDATA.py] get a tree "+str([i[1],i[0]*(-1.0),i[2]]))
        X_MIN=min(X_MIN,i[1])
        X_MAX=max(X_MAX,i[1])
        Y_MIN=min(Y_MIN,i[0]*(-1.0))
        Y_MAX=max(Y_MAX,i[0]*(-1.0))
        R_MAX=max(R_MAX,i[2])
        R_MIN=min(R_MIN,i[2])

    X_MIN1=X_MIN-max(2,(X_MAX-X_MIN)*0.6)
    X_MAX=X_MAX+max(2,(X_MAX-X_MIN)*0.6)
    X_MIN=X_MIN1
    Y_MIN1=Y_MIN-max(2,(Y_MAX-Y_MIN)*0.6)
    Y_MAX=Y_MAX+max(2,(Y_MAX-Y_MIN)*0.6)
    Y_MIN=Y_MIN1

    print("[TREEDATA.py] Get X_MIN={}, X_MAX={}, Y_MIN={}, Y_MAX={}".format(X_MIN,X_MAX,Y_MIN,Y_MAX))
    print("[TREEDATA.py] Get R_MAX={}, R_MIN={}".format(R_MAX,R_MIN))
    
except:

    print("[TREEDATA.py] ERROR to get the trunk map OAQ")
    TREE_DATA=[[2767694.6000474878,-352852.16121769784,0.3],[2767687.550047488,-352862.78621769784,0.35],[2767694.800047488,-352880.1862176978,0.4],[2767700.5000474877,-352880.1362176978,0.2],[2767698.510047488,-352881.72621769784,0.2],[2767699.6500474876,-352882.8862176978,0.3],[2767697.483380821,-352883.4362176978,0.2],[2767701.3750474877,-352884.3862176978,0.15],[2767699.300047488,-352884.9362176978,0.45],[2767707.6516908277,-352848.88876166823,0.75],[2767730.1016908274,-352849.9387616683,0.15],[2767717.2516908273,-352851.1887616683,0.1],[2767721.1016908274,-352853.78876166826,0.25],[2767710.1016908274,-352856.08876166824,0.2],[2767718.1516908277,-352857.08876166824,0.25],[2767718.4016908277,-352859.48876166827,0.2],[2767729.522274709,-352863.2814219437,0.2],[2767711.022274709,-352863.5314219437,0.2],[2767730.122274709,-352870.4314219437,0.4],[2767730.222274709,-352878.5314219437,0.2],[2767702.022274709,-352888.08142194373,0.3],[2767716.822274709,-352889.6314219437,0.8],[2767726.122274709,-352889.58142194373,0.3],[2767719.072274709,-352889.8814219437,0.35],[2767727.572274709,-352890.33142194373,0.4],[2767725.9222747087,-352891.1314219437,0.35],[2767707.822274709,-352891.7314219437,0.65],[2767727.872274709,-352892.3564219437,0.225],[2767727.272274709,-352899.1814219437,0.2],[2767707.9222747087,-352904.83142194373,0.15],[2767701.572274709,-352905.1814219437,0.3],[2767670.0378069477,-352897.688491822,0.2],[2767664.7878069477,-352903.338491822,0.25],[2767651.7422344387,-352847.6228367216,0.75],[2767653.5922344383,-352851.3728367216,0.3],[2767654.3422344383,-352864.8728367216,0.35],[2767650.7422344387,-352871.07283672155,0.25],[2767697.109163938,-352891.04801889224,0.3],[2767696.909163938,-352893.5980188922,0.35],[2767652.6987745943,-352877.0904607297,0.23333333333333334],[2767654.3321079277,-352881.17379406304,0.5],[2767661.882107928,-352884.4737940631,0.55],[2767656.6821079277,-352888.62379406305,0.35],[2767660.3321079277,-352891.54879406304,0.3],[2767659.9321079277,-352895.2237940631,0.7],[2767665.3321079277,-352894.92379406304,0.35],[2767661.9321079277,-352899.92379406304,0.4],[2767666.382107928,-352886.4737940631,0.55],[2767671.5344904643,-352851.47750020324,0.1],[2767677.434490464,-352852.0775002033,0.2],[2767683.7344904644,-352852.42750020325,0.1]]
    print("[TREEDATA.py] Use Original MAP "+str(TREE_DATA))


if __name__=="__main__":
    import numpy as np
    tree_data_wow=np.load("center_list_all(1228).npy")


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

