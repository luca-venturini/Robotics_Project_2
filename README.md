# Team Members
- 10623973 Matteo Fabris
- 10571388 Samuele Portanti
- 10609456 Luca Venturini

# Files Description
The "src" folder contains the p2_core folder with 3 nodes:
- The first node is used to broadcast the TF
- The second node broadcasts the path of the robot till that moment, reading the messages sent by amcl regarding the robot position
- The third node, saver, is used to save the map and the trajectory. We have taken the code from the github page of map server, and then we have applied some customization to it, in order to save the trajectory and the map calling services.


# TF Tree
![tf tree composed of: map -> odom -> base_link -> laser_front and laser_rear](imgs/tf_tree.png "tf tree")

# Used bags

We have used the first bag to create the map, the others two to do the localization.

Gmapping has been used for map creation.

The images of the created map and trajectories can be found in the  ```imgs``` folder

# How to start

The mapping can be started using the launchfile ```mapping.launch```, with the command:

```roslaunch p2_core mapping.launch ```

The localization can be started using the launchfile ```amcl.launch```, with the command:

```roslaunch p2_core amcl.launch```


# Services

#### The two services can be used to save the map and the trajectory, the images will be saved in the .ros folder

In order to save the map, without the trajectory, it is possibile to call the service:
```
rosservice call /save_map image_name
```

If we want to save the map and the trajectory, we run:
```
rosservice call /save_path image_name
```

# Useful Info
We have decided to create the map using Gmapping, we have tried setting different parameters, tuning for example the number of particles, the value of ranges and parameters like linearUpdate or angularUpdate.

In order to merge the two lasers we have used ira_laser_tools, which provides the functionality of merging the values of multiple lasers.
Finally for the localization we have used the amcl package, setting the parameter ```odom_model_type``` as ```omni-corrected``` and consequently tuning, among the other values, also the alphas values, to get a reasonable result.

We have decided to exploit part of the code of map_saver to generate an image of the map and to draw also the trajectory of the robot.

We have taken, for the github page of map server, the code used in map saver, and we have customized it.

In particular, the trajectory is built reading the values from the message containing the path information. Once the service is called, a linear interpolation between the points is done, in order to have a smooth trajectory, since the message containing the path has not all the points but just some "samples" retreived from time to time.

After this we order the points to read the array just once while building the map pixel by pixel, reducing the time complexity; in fact using a non ordered array increases a lot the time required to save the map.
