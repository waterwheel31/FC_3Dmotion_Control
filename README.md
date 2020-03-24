# Drone 3D Motion Plannning 



## Objective 

- Create a path from one point to another point, and create the path in 3D environment (a virtual city)
- Move a virtual drone to move along the path.

## Approach

- Applied graph to simplifiied the 3D route into nodes and edges
    -  First consider the obstacles in specified attitude  
    -  Then made edge and nodes based on Voronoi algorithm (as shown below)

![img](graph.png)

- Then applied A* search algorithm to create the optimal path 
- This uses [UdaciDrone API](https://udacity.github.io/udacidrone/) to control the drone


## Result

- The drone could smoothly moved to the specified point (see video 'video.mp4')
- The route planning was quite quick


## Further Improvements
- THe micro route between nodes is not always optimal, since nodes are located at the center between obstacles. To further opimize the route, this point can be considered.

## How to run
- Download the Simulator [from this repository](https://github.com/udacity/FCND-Simulator-Releases/releases).
- Set up Conda envrionment seeing [this repository](https://github.com/udacity/FCND-Term1-Starter-Kit) and activate it ('source activate fcnd')
- Run the simulator and choose Motion Planning environment
- Run following `python motion_planning.py` 

