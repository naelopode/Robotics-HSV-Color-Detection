import serial
ser = serial.Serial("COM3", 115200)
ser.timeout=None
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

fig, ax = plt.subplots()
ax.add_patch(Rectangle((-200,-100), 400, 200, color='red'))
plt.show(block=False)
while(1):
    r = ser.readline()
    g = ser.readline()
    b = ser.readline()
    if r[0]==65:
        r = r[1:len(r)-1]
        g = g[0:len(g)-1]
        b = b[0:len(b)-1]
        if r and g and b:
            rgb = [int(r)/255, int(g)/255, int(b)/255]
            ax.add_patch(Rectangle((-200,-100), 400, 200, color=rgb))
            print(rgb)
            plt.draw()
            plt.pause(0.002)
    else:
        poubelle = ser.readline()
    

    



  
