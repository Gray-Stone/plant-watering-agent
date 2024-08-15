from geometry_msgs.msg import Point, PointStamped, TransformStamped, Vector3 , PoseStamped, Pose

from nav_msgs.msg import MapMetaData, OccupancyGrid

from typing import Optional

import numpy as np
from std_msgs.msg import ColorRGBA, Header

from visualization_msgs.msg import Marker, MarkerArray

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

    def CheckEmptyCircle(self,center_coord:tuple[int,int] ,radius:int) -> bool:
        """Check if a circle area is free (<90 for each cell    )

        Args:
            center_coord (tuple[int,int]): center map coordinate 
            radius (int): cell_distance

        Returns:
            bool: _description_
        """


        cx,cy = center_coord
        checked_coords = []
        for dx in range (-radius , radius +1):
            for dy in range (-radius , radius +1):
                # everything in grid-number units.
                dis = np.sqrt(dx**2 + dy**2)
                if dis < radius:
                    check_xy = (cx+dx,cy+dy)
                    checked_coords.append(check_xy)
                    if self.get_data(check_xy) > 90:
                        return False , checked_coords
        return True , checked_coords

    def get_data(self, map_loc: tuple[int,int])-> Optional[int]:
        if not self.ValidLoc(map_loc):
            return None
        x,y = map_loc
        index = y * self.map_info.width + x
        return self.map.data[index]

    def color_sphere_gen(self,map_locs:list[tuple[int,int]] , scale = 0.1 , id = 10 , color = None)->Marker:
        m = Marker()
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
