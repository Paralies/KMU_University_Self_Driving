#!/usr/bin/env python
#-*- coding: utf-8 -*-

MAP_NAME = "snake"

SCREEN_WIDTH = 1860
SCREEN_HEIGHT = 1025

INIT_POSE = [
    [6.5, 5.12, 90.0],
    [12.3, 5.12, 90.0],
]

ROAD_WIDTH = 220
G = 245 + 81

LOGO = [720, 850]

WALL_SIZE = 10
OBS = [
   #############반드시 기본적으로 있어야 하는 벽##############
    [0, 0, WALL_SIZE, SCREEN_HEIGHT],                     #
    [0, 0, SCREEN_WIDTH, WALL_SIZE],                      #
    [0, SCREEN_HEIGHT-WALL_SIZE, SCREEN_WIDTH, WALL_SIZE],#
    [SCREEN_WIDTH-WALL_SIZE, 0, WALL_SIZE, SCREEN_HEIGHT],#
   ########################################################
   [ WALL_SIZE + 1*ROAD_WIDTH + G*0, WALL_SIZE + ROAD_WIDTH, G, SCREEN_HEIGHT - (2*WALL_SIZE + ROAD_WIDTH) ],
   [ WALL_SIZE + 2*ROAD_WIDTH + G*1, WALL_SIZE,              G, SCREEN_HEIGHT - (2*WALL_SIZE + ROAD_WIDTH) ],
   [ WALL_SIZE + 3*ROAD_WIDTH + G*2, WALL_SIZE + ROAD_WIDTH, G, SCREEN_HEIGHT - (2*WALL_SIZE + ROAD_WIDTH) ]
]

WARP = [
    {"zone" : [WALL_SIZE + 3*ROAD_WIDTH + G*3, int(SCREEN_HEIGHT/2), 220, 22], "mov_pos" : [(WALL_SIZE + 3*ROAD_WIDTH + G*3 + 110)*0.01, (int(SCREEN_HEIGHT/2)-75)*0.01, 90.0]},
    {"zone" : [WALL_SIZE, int(SCREEN_HEIGHT/2), 220, 22],                      "mov_pos" : [(WALL_SIZE + 110)*0.01, (int(SCREEN_HEIGHT/2)-75)*0.01, 90.0]} 
]

GOAL = [
    #[WALL_SIZE + 3*ROAD_WIDTH + G*3, int(SCREEN_HEIGHT/2), 220, 22],
    #[WALL_SIZE, int(SCREEN_HEIGHT/2), 220, 22]
]
