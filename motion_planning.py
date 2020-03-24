import argparse
import time
import msgpack
from enum import Enum, auto
import utm

import numpy as np

from planning_utils import a_star, heuristic, create_grid, collinearity_float, point
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

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

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

            print('points:', points)

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
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)
        self.local_position[0] = grid_start[0]
        self.local_position[1] = grid_start[1]

        # Set goal as some arbitrary position on the grid
        grid_goal = (-north_offset +100, -east_offset -10)  # +200, -40

        # Run A* to find a path from start to goal
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        

        print('type of path:', type(path))
        path_copy = path.copy()
        
        pos = 0
        for i in range(len(path)-2):
            if collinearity_float(path_copy[pos], path_copy[pos+1], path_copy[pos+2]):
                _ = path_copy.pop(pos+1)
                #print('path_copy:', path_copy)
            else: pos += 1

        path = path_copy

        print('A star calculation done')
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=6000)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
