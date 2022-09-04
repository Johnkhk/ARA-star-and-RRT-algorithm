import matplotlib.pyplot as plt
import argparse
import numpy as np
def showmap(robotpos,targetpos,envmap):
    f, ax = plt.subplots()
    ax.imshow( envmap.T, interpolation="none", cmap='gray_r', origin='lower', \
                extent=(-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5) )
    ax.axis([-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5])
    ax.set_xlabel('x')
    ax.set_ylabel('y')  
    hr = ax.plot(robotpos[0], robotpos[1], 'bs')
    ht = ax.plot(targetpos[0], targetpos[1], 'rs')
    
    plt.show(block=True)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some integers.')
    
    # 3
    robotstart = np.array([74, 249])
    targetstart = np.array([399, 399])
    #7
    # robotstart = np.array([0, 0])
    # targetstart = np.array([4998, 4998])

    parser.add_argument('--num', type=str, required=True)
    args = parser.parse_args()
    mapfile = "maps/map"+args.num+".txt"
    envmap = np.loadtxt(mapfile)
    showmap(robotstart,targetstart,envmap)

