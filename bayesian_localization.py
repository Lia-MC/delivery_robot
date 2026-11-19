# Code for and simulation of a generic Bayesian localization algorithm suitable for the project design.
# The scenario that is to be simulated is specied in the Attachment. This is to be completed in
# Python (no ROS is required) and it will serve as a starting point for your localization framework. A
# simulation run of the code is to be presented to the TA. Only one completed assignment is required
# per group.

import numpy as np
import matplotlib.pyplot as plt
import numpy as np

import matplotlib
import matplotlib as mpl




scenario = [[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12],
            [1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1], 
            ["", "orange", "yellow", "green", "blue", "nothing", "green", "blue", "green", "orange", "yellow", "green", "blue"]]

# cols: -1, 0, 1
state_model = [[0.85, 0.05, 0.05], # row 1 = X - chi
               [0.1, 0.9, 0.1], 
               [0.05, 0.05, 0.85]]

# cols: blue green yellow orange
measurement_model = [[0.6, 0.2, 0.05, 0.05], # row 1 = blue
                     [0.2, 0.6, 0.05, 0.05], 
                     [0.05, 0.05, 0.65, 0.2], 
                     [0.05, 0.05, 0.15, 0.6], 
                     [0.1, 0.1, 0.1, 0.1]]

# /line_idx

# /mean_img_rgb

# ros2 launch topic
# check for /mean_img_rgb 

# initial pdf: p = 1/5 for all colours

def initialize_belief():
    belief = [1/11, 1/11, 1/11, 1/11, 1/11, 1/11, 1/11, 1/11, 1/11, 1/11, 1/11]
    return belief

def prediction(belief,u):
    belief_pred = [0] * len(belief)
    for i in range(len(belief)):
        if u == -1:
            belief_pred[i] = state_model[2][0] * belief[(i-1)%11] + state_model[1][0] * belief[i] + state_model[0][0] * belief[(i+1)%11]
        elif u == 0:
            belief_pred[i] = state_model[0][1] * belief[(i+1)%11] + state_model[1][1] * belief[i] + state_model[2][1] * belief[(i-1)%11]
        else:
            belief_pred[i] = state_model[0][2] * belief[(i+1)%11] + state_model[1][2] * belief[i] + state_model[2][2] * belief[(i-1)%11]
    return belief_pred  

def correction(belief_pred, z):
    belief_corr = [0] * len(belief_pred)
    for i in range(len(belief_corr)):
        if i == 0 or i == 8:
            a  = 2
        elif i == 1 or i== 5 or i == 9:
            a = 1
        elif i == 2 or i == 6 or i == 10:
            a = 0
        elif i == 3 or i == 4 or i == 7: 
            a = 3
        if z == "blue":
            belief_corr[i] = measurement_model[0][a] * belief_pred[i]
        elif z == "green":
            belief_corr[i] = measurement_model[1][a] * belief_pred[i]
        elif z == "yellow":
            belief_corr[i] = measurement_model[2][a] * belief_pred[i]
        elif z == "orange":
            belief_corr[i] = measurement_model[3][a] * belief_pred[i]
        else:
            belief_corr[i] = measurement_model[4][a] * belief_pred[i]
    return belief_corr

def belief_update(belief_corr):
    total= 0
    for e in belief_corr:
        total += e
    for i in range(len(belief_corr)):
        belief_corr[i] = belief_corr[i] / total
    belief_update =belief_corr
    return belief_update

if __name__ == "__main__":
    belief = initialize_belief() 
    tot_belief = [0] * 13
    # measurement model (from table 2)
    # state model (from table 1)
    belief = initialize_belief()
    tot_belief[0] = belief
    for k in range(12):
        belief_pred = prediction(belief, scenario[1][k])
        belief_corr = correction(belief_pred, scenario[2][k+1])
        belief = belief_update(belief_corr)
        tot_belief[k+1] = belief
        print(belief)

    bel = np.array(tot_belief)


    fig, ax = plt.subplots()
    im = ax.imshow(bel)


    # Loop over data dimensions and create text annotations.
    for i in range(len(bel)):
        for j in range(len(bel[i])):
            text = ax.text(j, i, round(bel[i, j], 3),
                         ha="center", va="center", color="w")

    fig.tight_layout()
    plt.show()

    # k on the y axis 
    # colour (posterior) on the x axis

# sensor readings = /mean_img_rgb
# error 

# follow the line???

# motion model 