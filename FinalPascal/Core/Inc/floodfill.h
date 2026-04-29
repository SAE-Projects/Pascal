#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <stdint.h>
#include "maze.h"

// The distance array (Distance to center)
extern uint8_t dist[16][16];

// Standard Micromouse Targets (Center 4 cells)
#define TARGET_1_X 5
#define TARGET_1_Y 1
#define TARGET_2_X 5
#define TARGET_2_Y 1
#define TARGET_3_X 5
#define TARGET_3_Y 1
#define TARGET_4_X 5
#define TARGET_4_Y 1

void Floodfill_Reset(void);
void Floodfill_Recalculate(void);
Direction_t Floodfill_Get_Best_Dir(int8_t x, int8_t y, Direction_t current_dir);

#endif
