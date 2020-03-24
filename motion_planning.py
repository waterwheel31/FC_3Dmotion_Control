import argparse
import time
import msgpack
from enum import Enum, auto
import utm
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import pickle


from planning_utils import a_star, heuristic, create_grid, collinearity_float, point, create_grid_and_edges, visualization, closest_point, a_star_graph
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, grid='y'):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.global_goal_position = np.array([-122.398570, 37.796162, 0.0])  #np.array([-122.397781, 37.793920, 0.0])
        self.grid = True if grid == 'y' else False  # whether to calculate the gird (y) or load existing grid (n)

        print('grid:', self.grid)
        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            print('target_position:', self.target_position, 'local position:', self.local_position)
            local_position_reverse = self.local_position
            local_position_reverse[2] = local_position_reverse[2]*-1
            if np.linalg.norm(self.target_position[0:2] - local_position_reverse[0:2]) < 1.0 and \
               np.linalg.norm(self.target_position[2] - local_position_reverse[2]) < 5.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")

        TARGET_ALTITUDE = 100
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        with open('./colliders.csv') as f:
            s_line = f.readline()
            points = s_line.split(',')

            lat0 = float(points[0].split()[1])
            lon0 = float(points[1].split()[1])

            print('lat0', lat0, 'lon0', lon0)

            self.global_home[0] = lat0
            self.global_home[1] = lon0
            self.global_home[2] = 0.0
             
        local_pos =  global_to_local(self.global_position, self.global_home)
        self.local_position[0] = local_pos[0]
        self.local_position[1] = local_pos[1]
        self.local_position[2] = local_pos[2]
        local_goal = global_to_local(self.global_goal_position, self.global_home)
        print('local goal:', local_goal)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Create Voronoi graph
        grid = None
        nodes = None
        edges = None
        north_offset = 0
        east_offset = 0




        if self.grid:
            grid, nodes, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
            
            nodes = []
            for edge in edges:
                nodes.append(edge[0])
                nodes.append(edge[1])
            nodes = set(nodes)         
            
            grid_data = {'grid':grid, 'nodes':nodes, 'edges':edges, 'north_offset':north_offset, 'east_offset':east_offset}
            with open('pickle.pk', 'wb') as f:
                pickle.dump(grid_data, f)
        else: 
            with open('pickle.pk', 'rb') as f:
                grid_data = pickle.load(f)
                grid = grid_data['grid']
                nodes = grid_data['nodes']
                edges = grid_data['edges']
                north_offset = grid_data['north_offset']
                east_offset = grid_data['east_offset']
               
        nodes = []
        for edge in edges:
            nodes.append(edge[0])
            nodes.append(edge[1])
        nodes = set(nodes)   

        print('Found %5d edges' % len(edges))
        #print('grid:')
        #print(grid)
        print('nodes:')
        print(nodes)
        #print('edges:')
        #print(edges)
       
        #visualization(grid, edges)


        # Define a grid for a particular altitude and safety margin around obstacles
        #grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        point_start = [int(self.local_position[0] - north_offset), int(self.local_position[1]- east_offset)]
        point_goal = [int(local_goal[0] - north_offset), int(local_goal[1] - east_offset)]

        # Run A* to find a path from start to goal
        print('Local Start and Goal: ', point_start, point_goal)

        start_nearest = closest_point(nodes, point_start)
        goal_nearest = closest_point(nodes, point_goal)
        print('start nearest:', start_nearest, 'goal nearest:', goal_nearest)

        path_graph, _ = a_star_graph(edges, heuristic, start_nearest, goal_nearest)


        path = [point_start]
        for g in path_graph:
            path.append([g[0],g[1]])
        path.append(goal_nearest)
        path.append(point_goal)

        print('path:', path)


        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        
        print('waypoints:')
        print(self.waypoints)
        
        #self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--grid', type=str, default='y', help='whether to calculate the gird (y) or load existing grid (n)')
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=6000)
    drone = MotionPlanning(conn, grid=args.grid)
    time.sleep(1)

    drone.start()
