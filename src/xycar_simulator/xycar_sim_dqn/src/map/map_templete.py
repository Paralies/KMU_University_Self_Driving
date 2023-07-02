#!/usr/bin/env python
#-*- coding: utf-8 -*-

MAP_NAME = "map_templete" # 파일 이름과 같아야 함.
SCREEN_WIDTH = 800       # 창 너비 
SCREEN_HEIGHT = 600      # 창 높이

INIT_POSE = [             # 초기 위치 및 자세 설정 - 랜덤위치 할시 0번 고정됨, 아니면 랜덤으로 실행 [x, y, yaw]
    [1.0, 1.0, 90.0],     # 해당 리스트에 [x, y, yaw] 추가할 수록 시작할 수 있는 시작위치가 늘어남
    [1.0, 1.0, 270.0],
]

# 추가할 장애물 OBS 리스트에 [x, y, 너비, 높이] 리스트를 추가하면 해당 위치에 장애물이 나타남.
WALL_SIZE = 10
OBS = [
   #############반드시 기본적으로 있어야 하는 벽##############
    [0, 0, WALL_SIZE, SCREEN_HEIGHT],                     #
    [0, 0, SCREEN_WIDTH, WALL_SIZE],                      #
    [0, SCREEN_HEIGHT-WALL_SIZE, SCREEN_WIDTH, WALL_SIZE],#
    [SCREEN_WIDTH-WALL_SIZE, 0, WALL_SIZE, SCREEN_HEIGHT],#
   ########################################################
    [0, 0, 0, 0]
]

# 순간이동 설정
WARP = [
    {"zone" : [0, 0, 220, 22], "mov_pos" : [float('inf'), float('inf'), 180.0]}  # zone 에 들어가 있는 리스트 대로 워프존이 만들어짐
]                                                              # mov_pos 에 들어가 있는 리스트 대로 차량 위치와 포즈가 설정됨

GOAL = [
    #[WALL_SIZE + 3*ROAD_WIDTH + G*3, int(SCREEN_HEIGHT/2), 220, 22],
    #[WALL_SIZE, int(SCREEN_HEIGHT/2), 220, 22]
]
