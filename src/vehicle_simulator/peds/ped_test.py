#!/usr/bin/env python3

pos_list = [


      [-13.5708026886,-68.0989227295],

      [-18.8866176605,-52.6750450134],

      [7.21017408371,-60.8674201965],

      [2.67492055893,-45.3540534973],

      [-6.96759986877,-57.2897491455],

      [-1.26445353031,-55.0669631958],

      [-13.4524078369,-59.623374939],

      [-4.78709411621,-61.093372345],

      [-8.06304550171,-53.43491745]]


for i in range(9):
    x = pos_list[i][0]
    y = pos_list[i][1]
    z = 1.2

    f = open('/home/big/catkin_ws/src/autonomous_exploration_development_environment/src/vehicle_simulator/peds/ped.sdf','r')
    sdff = f.read()

    sdff=sdff.replace("ped_0", "ped_"+str(i))
    sdff= sdff.replace("<target>999 999 999</target>", "<target>"+str(x)+" "+str(y)+" "+str(z)+"</target>")

    print(sdff)


