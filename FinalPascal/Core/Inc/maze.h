#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>

/* ================================================================
   THE COMPASS (Directional Tracking)
   ================================================================ */
typedef enum {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
} Direction_t;

extern volatile Direction_t pascal_dir;

/* ================================================================
   THE COORDINATES (Positional Tracking)
   ================================================================ */
extern volatile int8_t pascal_x;
extern volatile int8_t pascal_y;

/* ================================================================
   THE MEMORY (Wall Mapping)
   ================================================================ */
#define WALL_NORTH 0x01   // 0001
#define WALL_EAST  0x02   // 0010
#define WALL_SOUTH 0x04   // 0100
#define WALL_WEST  0x08   // 1000

// 16x16 grid holding the bitwise walls for every cell
extern volatile uint8_t maze_map[16][16];

/* ================================================================
   FUNCTION PROTOTYPES
   ================================================================ */
void Maze_Init(void);

// Compass Updates
void Update_Compass_TurnRight(void);
void Update_Compass_TurnLeft(void);
void Update_Compass_TurnAround(void);

// Positional Updates
void Update_Position_Forward(void);

// Mapping Updates
void Map_Current_Cell(uint16_t front_sensor, uint16_t left_sensor, uint16_t right_sensor);

// maze.h
void Maze_Print_Map(void);
#endif /* MAZE_H */
