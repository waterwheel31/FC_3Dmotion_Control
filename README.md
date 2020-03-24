# Drone 3D Motion Plannning 



## Objective 

- Create a path from one point to another point, and create the path in 3D environment (a virtual city)
- Move a virtual drone to move along the path.

## Approach

- Distretize the space in to 2D grid 
- Add height information to make it to 2.5D dimensions, and check feasibility of moving from one point to another point in the grid
- Then apply A* search algorithm to create the optimal path 
- Remove middle points by checking collinearity
- This uses [UdaciDrone API](https://udacity.github.io/udacidrone/) to control the drone


## Result

- The drone could fly a 10 meter box 

## How to run
- Download the Simulator [from this repository](https://github.com/udacity/FCND-Simulator-Releases/releases).
- Set up Conda envrionment seeing [this repository](https://github.com/udacity/FCND-Term1-Starter-Kit) and activate it ('source activate fcnd')
- Run the simulator and choose Motion Planning environment
- Run following `python motion_planning.py` 

