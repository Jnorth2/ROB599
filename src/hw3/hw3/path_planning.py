import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped
from nav_interface.action import NavGoal
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import do_transform_point

import numpy as np
import cv2 as cv
import os
import heapq
import math
from pathlib import Path

# Define the Cell class
class Cell:
    def __init__(self):
        self.parent_i = 0  # Parent cell's row index
        self.parent_j = 0  # Parent cell's column index
        self.f = float('inf')  # Total cost of the cell (g + h)
        self.g = float('inf')  # Cost from start to this cell
        self.h = 0  # Heuristic cost from this cell to destination



class PathPlanner(Node):

    def __init__(self):
        super().__init__('path_planner')

        self.declare_parameter('save_path', False)
        self.save_path = self.get_parameter('save_path').value
        self.get_logger().info(f"Save Path {self.save_path}")

        self.action_client = ActionClient(self, NavGoal, 'nav_dwa_goal')
        # Wait for the action server to be available
        self.get_logger().info('Waiting for NavGoal action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('NavGoal action server connected!')

        self.create_subscription(OccupancyGrid, "map", self.map_cb, 10) 

        self.create_service(Trigger, "plan_path", self.plan_path_cb) 

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.global_waypoints = [
            Point(x=4.0, y=0.0, z=0.0),
            # Point(x=2.0, y=-2.0, z=0.0),
            # Point(x=0.0, y=-3.0, z=0.0),
            # Point(x=-4.0, y=0.0, z=0.0),
            # Point(x=0.0, y=3.0, z=0.0),
            Point(x=0.0, y=0.0, z=0.0)
        ]

        self.waypoints = []

        # Create a buffer to put the transform data in
        self.tf_buffer = Buffer()
        
		# This sets up a listener for all of the transform types created
        self.transform_listener = TransformListener(self.tf_buffer, self)

        self.global_idx = 0
        self.point_iter = 0
        self.is_done = False
        self.succeed = False
        self.sent = False
        self.send = False

        #robot Params
        self.robot_r = 0.4
        self.location = None

        #map params
        self.threshold = 30
        self.map = None
        self.np_map = None

        #path Params
        self.path = None
        self.waypoint_path = None

        #Image saving
        self.pkg_src_path = Path(__file__).resolve().parents[1]

        self.save_dir = os.path.join(self.pkg_src_path, "data")
        os.makedirs(self.save_dir, exist_ok=True)

    def timer_callback(self):
        if self.send and self.point_iter < len(self.waypoints):
            if not self.sent:
                self.send_waypoint()
            elif self.is_done:
                self.sent = False
                self.is_done = False
                self.point_iter += 1
        elif self.point_iter >= len(self.waypoints):
            self.send = False
            self.point_iter = 0


    def send_waypoint(self):
        goal_msg = NavGoal.Goal()
        goal_msg.goal.header.frame_id = "odom"
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        # self.get_logger().info(f"Index: {self.point_iter}")
        goal_msg.goal.point = self.waypoints[self.point_iter]
        send_waypoint_future = self.action_client.send_goal_async(goal_msg)
        send_waypoint_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f"Sent goal: {self.waypoints[self.point_iter].x}, {self.waypoints[self.point_iter].y}")
        self.sent = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            self.is_done = True
            return
            
        # self.get_logger().info('Goal accepted!')
        
        # Get the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status        
        self.get_logger().info("Finished Action Request")
        self.is_done = True

    def plan_path_cb(self, request, response):
        if self.global_idx >= len(self.global_waypoints):
            self.global_idx = 0
        point = self.global_waypoints[self.global_idx]
        self.global_idx += 1
        self.get_logger().info(f"Global Point: {point.x}, {point.y}")
        found_path = self.path_planning(point)
        if found_path:
            if self.save_path:
                self.plan_to_image(self.path)
            self.send = True
            response.success = True
            response.message = "Finished Planning"
        else:
            self.send = False
            response.success = False
            response.message = "Failed to find a Path"
        return response


    def map_cb(self, msg):
        self.map = msg
        temp_map = np.array(self.map.data, dtype=np.int8)
        temp_map = temp_map.reshape(self.map.info.height,
                             self.map.info.width)
        temp_map = np.flipud(temp_map)
        self.np_map = temp_map

    def transform_to_grid(self, x_global, y_global):
        """Determine the cell indexes for a set of global x, y coordinates. 

        Parameters
        ----------
        x_global : np.array(floats)
            An array of x position values in the global frame. 
        y_global : np.array(floats)
            An array of y position values in the global frame. 

        Returns
        -------
        x_map : np.array(int)
            The x indexes in the map frame.
        y_map : np.array(int)
            The y indexes in the map frame.
        """
        x_map = int((x_global - self.map.info.origin.position.x) / self.map.info.resolution)
        y_map = int((y_global - self.map.info.origin.position.y) / self.map.info.resolution)


        self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds = 1.0))
        robot_x = int((self.location.transform.translation.x - self.map.info.origin.position.x) / self.map.info.resolution)
        robot_y = int((self.location.transform.translation.y - self.map.info.origin.position.y) /self.map.info.resolution)

        return x_map, y_map, robot_x, robot_y

    def grid_to_transform(self, x_map, y_map):
        x_global = x_map * self.map.info.resolution + self.map.info.origin.position.x
        y_global = y_map * self.map.info.resolution + self.map.info.origin.position.y
        return x_global, y_global 
    
    def path_planning(self, goal : Point):

        #get goal and robot location in grid indexes
        x_idx, y_idx, robot_x, robot_y = self.transform_to_grid(goal.x, goal.y)

        self.get_logger().info(f"Map Coord: {x_idx}, {y_idx} | Robot coord: {robot_x}, {robot_y}")
        

        #Threshold map
        thresh_map = (self.np_map > 30).astype(np.uint8)

        #inflate map
        inf_map = self.inflate_map(thresh_map)
        
        #A*
        self.path = self.a_star_search(inf_map, [robot_x, robot_y], [x_idx, y_idx])
        if self.save_path:
            self.arr_to_image(thresh_map, "threshold")
            self.arr_to_image(inf_map, "inflation")
            self.start_end_image(inf_map, [x_idx, y_idx], [robot_x, robot_y], "Start_End")

        if self.path is None:
            return False
        # self.plan_to_image(path)
        self.path_to_waypoints()

        if self.save_path:
            self.plan_to_image(self.waypoint_path, "waypoint")

        return True
    
    def inflate_map(self, map):
        cell_padding = math.ceil(self.robot_r/self.map.info.resolution)
        self.get_logger().info(f"Inflate by: {cell_padding} cells")
        inf_map = map * 1
        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                if map[i, j] != 0:
                    for di in range(-cell_padding, cell_padding + 1):
                        for dj in range(-cell_padding, cell_padding + 1):
                            # print(f"x: {i + di}, y: {j+dj}")
                            # print(f"di: {di}, dj {dj}")
                            if self.is_valid(i+di, j+dj):
                                inf_map[i+di, j+dj] = 1
        return inf_map

    
    def is_valid(self, x, y):
        return (x >= 0) and (x < self.map.info.height) and (y >= 0) and (y < self.map.info.height)
    
    def is_free(self, map, x, y):
        return map[y, x] == 0
    
    def calc_h(self, x, y, dest):
        return math.sqrt((x - dest[1]) ** 2 + (y - dest[0]) ** 2)
    
    # Implement the A* search algorithm
    def a_star_search(self, map, src, dest):
        """This is an A* algorithm by geeks for geeks[1] adapted to run with a binary numpy array map. 

        Reference
        ---------
        [1] GeeksforGeeks, “A* Search Algorithm,” GeeksforGeeks, Jun. 16, 2016. https://www.geeksforgeeks.org/dsa/a-search-algorithm/
        """

        # Check if the source and destination are valid
        if not self.is_valid(src[0], src[1]) or not self.is_valid(dest[0], dest[1]):
            print("Source or destination is invalid")
            return

        # Check if the source and destination are unblocked
        if not self.is_free(map, src[0], src[1]) or not self.is_free(map, dest[0], dest[1]):
            print("Source or the destination is blocked")
            return

        # Check if we are already at the destination
        if src[0] == dest[0] and src[1] == dest[1]:
            print("We are already at the destination")
            return 

        # Initialize the closed list (visited cells)
        closed_list = [[False for _ in range(self.map.info.width)] for _ in range(self.map.info.height)]
        # Initialize the details of each cell
        cell_details = [[Cell() for _ in range(self.map.info.width)] for _ in range(self.map.info.height)]

        # Initialize the start cell details
        i = src[0]
        j = src[1]
        cell_details[i][j].f = 0
        cell_details[i][j].g = 0
        cell_details[i][j].h = 0
        cell_details[i][j].parent_i = i
        cell_details[i][j].parent_j = j

        # Initialize the open list (cells to be visited) with the start cell
        open_list = []
        heapq.heappush(open_list, (0.0, i, j))

        # Initialize the flag for whether destination is found
        found_dest = False

        # Main loop of A* search algorithm
        while len(open_list) > 0:
            # Pop the cell with the smallest f value from the open list
            p = heapq.heappop(open_list)

            # Mark the cell as visited
            i = p[1]
            j = p[2]
            closed_list[i][j] = True

            # For each direction, check the successors
            directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
            for dir in directions:
                new_i = i + dir[0]
                new_j = j + dir[1]

                # If the successor is valid, unblocked, and not visited
                if self.is_valid(new_i, new_j) and self.is_free(map, new_i, new_j) and not closed_list[new_i][new_j]:
                    # If the successor is the destination
                    if new_i == dest[0] and new_j == dest[1]:
                        # Set the parent of the destination cell
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j
                        print("The destination cell is found")
                        # Trace and print the path from source to destination
                        # trace_path(cell_details, dest)
                        found_dest = True
                        return self.trace_path(cell_details, dest)
                    else:
                        # Calculate the new f, g, and h values
                        #Check if on diagonal
                        if dir[0] != 0 and dir[1] != 0:
                            cost = 1.414
                        else:
                            cost = 1.0
                        g_new = cell_details[i][j].g + cost
                        h_new = self.calc_h(new_i, new_j, dest)
                        f_new = g_new + h_new

                        # If the cell is not in the open list or the new f value is smaller
                        if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                            # Add the cell to the open list
                            heapq.heappush(open_list, (f_new, new_i, new_j))
                            # Update the cell details
                            cell_details[new_i][new_j].f = f_new
                            cell_details[new_i][new_j].g = g_new
                            cell_details[new_i][new_j].h = h_new
                            cell_details[new_i][new_j].parent_i = i
                            cell_details[new_i][new_j].parent_j = j

        # If the destination is not found after visiting all cells
        if not found_dest:
            print("Failed to find the destination cell")

    
    # Trace the path from source to destination
    def trace_path(self, cell_details, dest):
        # print("The Path is ")
        path = []
        row = dest[0]
        col = dest[1]

        # Trace the path from destination to source using parent cells
        while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
            path.append((row, col))
            temp_row = cell_details[row][col].parent_i
            temp_col = cell_details[row][col].parent_j
            row = temp_row
            col = temp_col

        # Add the source cell to the path
        path.append((row, col))
        # Reverse the path to get the path from source to destination
        path.reverse()
        # self.get_logger().info(f"Path is {path}")

        return path
        
    def path_to_waypoints(self):
        self.waypoints = []
        self.waypoint_path = []
        for i in range(0, len(self.path)-1 , 5):
            x, y = self.grid_to_transform(self.path[i][0], self.path[i][1])
            point = Point()
            point.x = x
            point.y = -y
            self.waypoints.append(point)
            self.waypoint_path.append(self.path[i])
            # self.get_logger().info(f"Waypoint {i}: {x}, {y}")
        point = Point()
        x, y = self.grid_to_transform(self.path[-1][0], self.path[-1][1])
        point.x = x
        point.y = -y
        self.waypoints.append(point)
        self.waypoint_path.append(self.path[-1])
        # self.get_logger().info(f"Waypoint end: {x}, {y}")
        # print(f"Selected waypoints {self.waypoints}")
    
    def plan_to_image(self, path, name = ""):
        #Get map in 2d Array
        temp_map = np.array(self.map.data, dtype=np.int8)
        temp_map = temp_map.reshape(self.map.info.height,
                             self.map.info.width)
        temp_map = np.flipud(temp_map)
        #determine Seen vs unseen
        unknown_mask = np.array(temp_map < 0)
        known_mask = ~unknown_mask
        #Calc Grey Values
        grey_val = (1.0 - temp_map[known_mask]/100.0) * 255.0
        grey_val = grey_val.astype(np.uint8)
        #Create Image
        img = np.zeros((self.map.info.height, self.map.info.width, 3), dtype=np.uint8)
        img[known_mask] = np.stack([grey_val, grey_val, grey_val], axis=1)
        #Make unseen cyan
        img[unknown_mask] = np.array([255, 255, 0], dtype=np.uint8)

        #Make Path cells yellow
        path = np.array(path)
        y = path[:,0]
        x = path[:,1]
        img[x, y] = np.array([0, 255, 255], dtype=np.uint8)

        #Save
        time = self.get_clock().now().to_msg()
        cv.imwrite(os.path.join(self.save_dir, f'path_out_{name}_{time.sec}.png'), img)

    def arr_to_image(self, map, name=""):
                #Get map in 2d Array
        temp_map = map
        # temp_map = np.flipud(map)
        #determine Seen vs unseen
        unknown_mask = np.array(temp_map < 0)
        known_mask = ~unknown_mask
        #Calc Grey Values
        grey_val = (1.0 - temp_map[known_mask]) * 255.0
        grey_val = grey_val.astype(np.uint8)
        #Create Image
        img = np.zeros((self.map.info.height, self.map.info.width, 3), dtype=np.uint8)
        img[known_mask] = np.stack([grey_val, grey_val, grey_val], axis=1)
        #Make unseen cyan
        img[unknown_mask] = np.array([255, 255, 0], dtype=np.uint8)

        #Save
        time = self.get_clock().now().to_msg()
        cv.imwrite(os.path.join(self.save_dir, f'temp_out_{name}_{time.sec}.png'), img)

    def start_end_image(self, map, goal, robot, name=""):
        temp_map = map
        # temp_map = np.flipud(map)
        #determine Seen vs unseen
        unknown_mask = np.array(temp_map < 0)
        known_mask = ~unknown_mask
        #Calc Grey Values
        grey_val = (1.0 - temp_map[known_mask]) * 255.0
        grey_val = grey_val.astype(np.uint8)
        #Create Image
        img = np.zeros((self.map.info.height, self.map.info.width, 3), dtype=np.uint8)
        img[known_mask] = np.stack([grey_val, grey_val, grey_val], axis=1)
        #Make unseen cyan
        img[unknown_mask] = np.array([255, 255, 0], dtype=np.uint8)

        #Goal
        img[goal[1], goal[0]] = np.array([0, 255, 0], dtype=np.uint8)

        #Robot
        img[robot[1], robot[0]] = np.array([0, 255, 255], dtype=np.uint8)

        #Save
        time = self.get_clock().now().to_msg()
        cv.imwrite(os.path.join(self.save_dir, f'temp_out_{name}_{time.sec}.png'), img)



def main(args=None):
    rclpy.init(args=args)

    path_planner = PathPlanner()
    # path_planner.send_waypoint()

    # rclpy.spin(path_planner)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # path_planner.destroy_node()
    # rclpy.shutdown()

    executor = MultiThreadedExecutor()
    executor.add_node(path_planner)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()