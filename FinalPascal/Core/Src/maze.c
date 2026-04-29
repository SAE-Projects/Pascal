#include "maze.h"
#include <stdio.h>
#include "floodfill.h"
#include "navigator.h"
#include "string.h"
/* --- GLOBAL STATE VARIABLES --- */
// Pascal always starts at (0,0) facing NORTH
volatile Direction_t pascal_dir = NORTH;
volatile int8_t pascal_x = 0;
volatile int8_t pascal_y = 0;

// Initialize the 16x16 map to completely empty (0)
volatile uint8_t maze_map[16][16] = {0};

/* ================================================================
   INITIALIZATION
   ================================================================ */
void Maze_Init(void) {
    pascal_dir = NORTH;
    pascal_x = 0;
    pascal_y = 0;

    // Force every byte of the map to 0
    memset((void*)maze_map, 0, sizeof(maze_map));

    // Also clear the distance map for good measure
    memset((void*)dist, 255, sizeof(dist));

    printf("Maze RAM Wiped Clean.\r\n");
}

/* ================================================================
   THE COMPASS LOGIC
   ================================================================ */

/* Turning Right adds 1 to the direction. Modulo 4 wraps WEST(3) back to NORTH(0). */
void Update_Compass_TurnRight(void) {
    pascal_dir = (pascal_dir + 1) % 4;
}

/* Turning Left subtracts 1. Adding 3 and using modulo has the exact same effect. */
void Update_Compass_TurnLeft(void) {
    pascal_dir = (pascal_dir + 3) % 4;
}

/* Turning Around (U-Turn) adds 2 to the direction. */
void Update_Compass_TurnAround(void) {
    pascal_dir = (pascal_dir + 2) % 4;
}

void Maze_Print_Distances(void) {
    printf("\n--- PASCAL DISTANCE MAP ---\n");
    for (int y = 15; y >= 0; y--) {
        for (int x = 0; x < 16; x++) {
            printf("%3d ", dist[x][y]);
        }
        printf("\n");
    }
}

/* ================================================================
   THE COORDINATE LOGIC
   ================================================================ */

/* Called immediately after Pascal physically drives 180mm into the next square. */
void Update_Position_Forward(void) {
    switch(pascal_dir) {
        case NORTH: pascal_y += 1; break;
        case EAST:  pascal_x += 1; break;
        case SOUTH: pascal_y -= 1; break;
        case WEST:  pascal_x -= 1; break;
    }

    // Safety check to prevent array out-of-bounds crashes
    if (pascal_x > 15) pascal_x = 15;
    if (pascal_x < 0)  pascal_x = 0;
    if (pascal_y > 15) pascal_y = 15;
    if (pascal_y < 0)  pascal_y = 0;
}

/* ================================================================
   THE WALL MAPPING LOGIC
   ================================================================ */

/* Maps the walls of the current (X, Y) cell based on sensor thresholds.
 * Assumes a calibrated 0-1000 scale where > 500 means a wall is close.
 */
void Map_Current_Cell(uint16_t front_sensor, uint16_t left_sensor, uint16_t right_sensor) {

    uint8_t walls = 0; // Start with no walls

    // Check which physical sensors see a wall
    uint8_t see_front = (front_sensor > FRONT_WALL_THRESHOLD);
    uint8_t see_left  = (left_sensor  > SIDE_WALL_THRESHOLD);
    uint8_t see_right = (right_sensor > SIDE_WALL_THRESHOLD);

    // Translate physical sensor view to absolute North/South/East/West walls
    switch (pascal_dir) {
        case NORTH:
            if (see_front) walls |= WALL_NORTH;
            if (see_left)  walls |= WALL_WEST;
            if (see_right) walls |= WALL_EAST;
            break;

        case EAST:
            if (see_front) walls |= WALL_EAST;
            if (see_left)  walls |= WALL_NORTH;
            if (see_right) walls |= WALL_SOUTH;
            break;

        case SOUTH:
            if (see_front) walls |= WALL_SOUTH;
            if (see_left)  walls |= WALL_EAST;
            if (see_right) walls |= WALL_WEST;
            break;

        case WEST:
            if (see_front) walls |= WALL_WEST;
            if (see_left)  walls |= WALL_SOUTH;
            if (see_right) walls |= WALL_NORTH;
            break;
    }

    // 1. Save the walls into the map at Pascal's current position
    maze_map[pascal_x][pascal_y] |= walls;

    // 2. CRITICAL FIX: Two-Sided Walls!
    // If Pascal sees a North wall, the cell above him must get a South wall.
    if ((walls & WALL_NORTH) && pascal_y < 15) maze_map[pascal_x][pascal_y + 1] |= WALL_SOUTH;
    if ((walls & WALL_EAST)  && pascal_x < 15) maze_map[pascal_x + 1][pascal_y] |= WALL_WEST;
    if ((walls & WALL_SOUTH) && pascal_y > 0)  maze_map[pascal_x][pascal_y - 1] |= WALL_NORTH;
    if ((walls & WALL_WEST)  && pascal_x > 0)  maze_map[pascal_x - 1][pascal_y] |= WALL_EAST;
}

// maze.c

void Maze_Print_Map(void) {
    printf("\n--- PASCAL MAPPED MAZE ---\n");

    for (int y = 15; y >= 0; y--) {
        // 1. Print North walls
        for (int x = 0; x < 16; x++) {
            printf("+");
            if (maze_map[x][y] & WALL_NORTH) printf("---");
            else printf("   ");
        }
        printf("+\n");

        // 2. Print West/East walls and Bot Position
        for (int x = 0; x < 16; x++) {
            if (maze_map[x][y] & WALL_WEST) printf("|");
            else printf(" ");

            if (x == pascal_x && y == pascal_y) {
                // FIXED: Draw directional arrow so you can see where he is facing!
                if (pascal_dir == NORTH) printf(" ^ ");
                else if (pascal_dir == EAST) printf(" > ");
                else if (pascal_dir == SOUTH) printf(" v ");
                else printf(" < ");
            } else {
                printf("   ");
            }
        }
        // FIXED: Print the final East wall of the row properly!
        if (maze_map[15][y] & WALL_EAST) printf("|\n");
        else printf(" \n");
    }

    // 3. Print Bottom floor
    for (int x = 0; x < 16; x++) {
        printf("+");
        if (maze_map[x][0] & WALL_SOUTH) printf("---");
        else printf("   ");
    }
    printf("+\n");
}
