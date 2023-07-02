#!/usr/bin/env python
#-*- coding: utf-8 -*-

MAP_NAME = "square"

SCREEN_WIDTH = 1000
SCREEN_HEIGHT = 720 

INIT_POSE = [
    [5.5, 1.15, 0.0]
]

ROAD_WIDTH = 220
WALL_SIZE = 10

LOGO = [280, 305]

OBS = [
   #############반드시 기본적으로 있어야 하는 벽##############
    [0, 0, WALL_SIZE, SCREEN_HEIGHT],                     #
    [0, 0, SCREEN_WIDTH, WALL_SIZE],                      #
    [0, SCREEN_HEIGHT-WALL_SIZE, SCREEN_WIDTH, WALL_SIZE],#
    [SCREEN_WIDTH-WALL_SIZE, 0, WALL_SIZE, SCREEN_HEIGHT],#
   ########################################################
   [  WALL_SIZE + ROAD_WIDTH, WALL_SIZE + ROAD_WIDTH, SCREEN_WIDTH - 2*(WALL_SIZE + ROAD_WIDTH), WALL_SIZE ],
   [  WALL_SIZE + ROAD_WIDTH, SCREEN_HEIGHT - (2*WALL_SIZE + ROAD_WIDTH), SCREEN_WIDTH - 2*(WALL_SIZE + ROAD_WIDTH), WALL_SIZE ],
   [  WALL_SIZE + ROAD_WIDTH, 2*WALL_SIZE + ROAD_WIDTH, WALL_SIZE, SCREEN_HEIGHT - 2*(2*WALL_SIZE + ROAD_WIDTH)],
   [ SCREEN_WIDTH - (2*WALL_SIZE + ROAD_WIDTH), 2*WALL_SIZE + ROAD_WIDTH, WALL_SIZE, SCREEN_HEIGHT - 2*(2*WALL_SIZE + ROAD_WIDTH) ]
]

WARP = [
   {"zone" : [430, 11, 5, 220], "mov_pos" : [-1, -1, 90.0]},
   {"zone" : [435, 11, 5, 220], "mov_pos" : [-1, -1, 180.0]}   
]

GOAL = [
   #[230, 11, 22, 220]
]
