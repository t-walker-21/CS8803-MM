import os
import numpy as np

for i in range(1,10):
    height = np.random.uniform(0.8,1.1)
    command = "rosrun gazebo_ros spawn_model -file appleModel.urdf -urdf -x -0.2 -z " + str(height) + " -y " + str(i-1) + " -model a" + str(i)
    os.system(command)