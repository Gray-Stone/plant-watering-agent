import math
from geometry_msgs.msg import Point, PointStamped, TransformStamped, Vector3 , PoseStamped, Pose

from nav_msgs.msg import MapMetaData, OccupancyGrid

from typing import Optional , Callable

import numpy as np
from std_msgs.msg import ColorRGBA, Header
from collections import deque
from visualization_msgs.msg import Marker, MarkerArray
from stretch_mover_utils.marker_helper import MakeCylinderMarker , MakeCubeMarker
import time
COLOR_MSG_LIST_RGBW = [
    ColorRGBA(r=1.0,b=0.0,g=0.0,a=1.0),
    ColorRGBA(r=0.0,b=0.0,g=1.0,a=1.0),
    ColorRGBA(r=0.0,b=1.0,g=0.0,a=1.0),
    ColorRGBA(r=1.0,b=1.0,g=1.0,a=1.0),
]


class OccupancyGridHelper():
    def __init__(self , map_msg: OccupancyGrid ):
        self.map = map_msg
        self.map_info = map_msg.info
        self.map_frame = self.map.header.frame_id
        self.meter_per_cell = self.map_info.resolution

    def world_point_to_map(self , pose: Point):
        return self.world_loc_to_map( (pose.x,pose.y) ,)

    def world_loc_to_map(self , world_loc: tuple[float,float]):
        # According to doc, origin is lower left corner.
        # The map frame is x to right, y up.
        x,y = world_loc
        x_off = x - self.map_info.origin.position.x
        y_off = y - self.map_info.origin.position.y

        index_x = int(x_off / self.map_info.resolution)
        index_y = int(y_off / self.map_info.resolution)

        return index_x , index_y


    def map_loc_to_world(self,map_loc : tuple[int,int])->tuple[float,float]:
        mx , my = map_loc

        point_x = mx * self.map_info.resolution + self.map_info.origin.position.x
        point_y = my * self.map_info.resolution + self.map_info.origin.position.y
        # This will give it's lower left corner. so need to shift it by half a cell size
        point_x += self.map_info.resolution/2
        point_y += self.map_info.resolution/2
        return ( point_x , point_y)
    def map_loc_to_world_point(self,map_loc : tuple[int,int]) -> Point:
        wx,wy = self.map_loc_to_world(map_loc)
        return (Point(x=wx,y=wy))

    def ValidLoc(self,map_loc: tuple[int,int]):
        x,y = map_loc
        if x <0 or x>= self.map_info.width:
            return False
        if y<0 or y>= self.map_info.height:
            return False
        return True

    def Get4Neighbor(self, map_loc: tuple[int, int]) -> list[tuple[int, int]]:
        # TODO change this by add each after boundary check
        x, y = map_loc
        valid_nbr = []
        for loc in [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),
        ]:
            if self.ValidLoc(loc):
                valid_nbr.append(loc)

        return valid_nbr

    def GetRing(self,center_loc:tuple[int,int] , ring_radius:int) -> list[tuple[int,int]]:
        cx,cy = center_loc
        ring_list = []
        for dx in range (-ring_radius , ring_radius +1):
            for dy in range (-ring_radius , ring_radius +1):
                if int(round(np.sqrt(dx**2 + dy**2))) == ring_radius:
                    potential_xy = [cx+dx ,cy+dy]
                    if self.ValidLoc(potential_xy):
                        ring_list.append(potential_xy)
        return ring_list

    def GetRingFast(self,center_loc:tuple[int,int] , radius:int) -> list:
        x0, y0 = center_loc
        coords = []

        for i in range(radius + 1):
            candidates = [
                (x0 + i, y0 + (radius - i)),  # Top-right quadrant
                (x0 - i, y0 + (radius - i)),  # Top-left quadrant
                (x0 + i, y0 - (radius - i)),  # Bottom-right quadrant
                (x0 - i, y0 - (radius - i))   # Bottom-left quadrant
            ]
            for loc in candidates:
                if self.ValidLoc(loc):
                    coords.append(loc)
        return coords

    def CheckRingOccupied(self,
                             center_coord: tuple[int, int],
                         radius: int ,
                         occupied_threshold=90) -> tuple[bool, list[tuple[int, int]]]:
        # This is the super fast version of check empty circle
        rings = self.GetRingFast(center_coord , radius)
        for r in rings:
            grid_data = self.get_data(r)
            if grid_data > occupied_threshold :
                return False , rings
        return True , rings




    def CheckEmptyCircle(self,
                         center_coord: tuple[int, int],
                         radius: int ,
                         occupied_threshold=90) -> tuple[bool, list[tuple[int, int]]]:
        """Check if a circle area is free (<90 for each cell    )

        Args:
            center_coord (tuple[int,int]): center map coordinate 
            radius (int): cell_distance

        Returns:
            bool: weather any cell in radius is occupied 
            int: how many of cells are unknown
            list: list of cells being checked
        """

        cx,cy = center_coord
        checked_coords = []
        for dx in range (-radius , radius +1):
            for dy in range (-radius , radius +1):
                # everything in grid-number units.
                dis = np.sqrt(dx**2 + dy**2)
                if dis < radius:
                    check_xy = (cx+dx,cy+dy)
                    if not self.ValidLoc(check_xy):
                        continue
                    checked_coords.append(check_xy)
                    grid_data = self.get_data(check_xy)
                    if grid_data > occupied_threshold :
                        return False , checked_coords
                    # A free cell is in checked_coords but not counted as unknown
        return True , checked_coords

    def get_data(self, map_loc: tuple[int,int])-> Optional[int]:
        if not self.ValidLoc(map_loc):
            return None
        x,y = map_loc
        index = y * self.map_info.width + x
        return self.map.data[index]

    def color_sphere_gen(self,map_locs:list[tuple[int,int]] , scale = None , id = 10 , color = None)->Marker:
        m = Marker()
        if scale is None:
            scale = self.meter_per_cell * 1.2
        m.scale = Vector3(x=scale,y=scale,z=scale)
        m.type = Marker.SPHERE_LIST
        m.header = self.map.header
        m.id = id
        for loc in map_locs:
            data = self.get_data(loc)
            m.points.append(self.map_loc_to_world_point(loc))
            if color is not None:
                m.colors.append(color)
            elif data == -1:
                m.colors.append(COLOR_MSG_LIST_RGBW[2])
            elif data ==0 :
                m.colors.append(COLOR_MSG_LIST_RGBW[3])
            elif data>99:
                m.colors.append(COLOR_MSG_LIST_RGBW[0])
            else:
                c = COLOR_MSG_LIST_RGBW[2]
                c.r = data / 100
                m.colors.append(c)
        return m

    def coord_hash(self,coord: list[int,int]):
        return (coord[0] * 5000 + coord[1])
    def find_reachable_frontiers(self,
                                 current_pose: PointStamped,
                                 clearance_radius: float,
                                 debug_marker_pub_func: Callable[[Marker],None] = None)->deque:
        start_coord = self.world_point_to_map(current_pose.point)
        clearance_rad_grid = int(math.ceil(clearance_radius / self.meter_per_cell))

        wave_front = deque()
        visited_cells = set()
        for i in range (1,clearance_rad_grid):
            maybe_cells = self.GetRingFast(start_coord , i)
            for c in maybe_cells:
                data  =self.get_data(c)
                if  data <0 or data > 90:
                    continue
                wave_front.append(c)
            if len(wave_front)>0:
                print(f"Start location picked when ring of {i}")
                break
        else:
            # Really bad thing happend, We can't even get a starting point!
            raise RuntimeError(
                f"Wavefront starting point {current_pose} does not have valid free cell"
                f" within robot clearance size {clearance_radius}"
            )

        wave_front.append(start_coord) # pop left later
        frontier_cells = deque()

        iter_count = 0
        last_m_dis = 0
        while len(wave_front)>0:
            current_point = wave_front.popleft()
            m_dis = abs(start_coord[0] - current_point[0]) + abs(start_coord[1] - current_point[1])

            visited_cells.add(self.coord_hash(current_point))

            # Grow the wavefront to any cell that is
            neighbors = self.Get4Neighbor(current_point)

            # Wave front is grow to the tip of the unknown. So all unoccupied cells should count.
            for n in neighbors:

                if self.coord_hash(n) in visited_cells:
                    continue
                if n in wave_front:
                    continue
                # Coord is not hash-able, so must do this.
                n_data = self.get_data(n)
                # In theory, we wont't even get near occupied cell, but if clearance_rad is too small,
                # It is still possible, so check anyway
                if n_data > 90:
                    continue
                elif n_data <0:
                    # This is a unknown! It's a frontier!
                    frontier_cells.append(n)
                    continue

                clear , check_cells =self.CheckEmptyCircle(n , clearance_rad_grid)
                if not clear:
                    continue
                # Not the cell is free, not near walls, and not visited. It can be expanded to.
                # if iter_count % 20 ==0:
                #     debug_marker_pub_func( MakeCubeMarker(9 ,self.map_loc_to_world_point(n) , color=COLOR_MSG_LIST_RGBW[1]) )
                #     debug_marker_pub_func(
                #         self.color_sphere_gen(check_cells,
                #                             id=14,
                #                             scale=0.01,
                #                             color=ColorRGBA(r=1.0, b=1.0, g=0.0, a=1.0)))
                #     debug_marker_pub_func( MakeCubeMarker(8 ,self.map_loc_to_world_point(current_point) , color=COLOR_MSG_LIST_RGBW[0]) )
                    

                wave_front.append(n)

            iter_count +=1

            if m_dis > last_m_dis + 5:
                last_m_dis = m_dis
                if debug_marker_pub_func is not None:

                    # debug_marker_pub_func(self.color_sphere_gen(check_cells , color=COLOR_MSG_LIST_RGBW[0] , id=15))
                    debug_marker_pub_func(self.color_sphere_gen(wave_front , id=12))
                    debug_marker_pub_func(self.color_sphere_gen(frontier_cells , scale=self.meter_per_cell * 1.6))

        if debug_marker_pub_func is not None:
            debug_marker_pub_func(self.color_sphere_gen(wave_front , id=12))
            debug_marker_pub_func(self.color_sphere_gen(frontier_cells , scale=self.meter_per_cell * 1.6))
        return frontier_cells
        # If a clear cell have its neighbor
