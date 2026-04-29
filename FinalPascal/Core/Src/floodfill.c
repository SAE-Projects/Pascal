#include "floodfill.h"

uint8_t dist[16][16];

// Simple Queue for BFS
typedef struct {
    int8_t x, y;
} Cell_t;

// FIX: Increased from 256 to 512 to prevent buffer overflow
// if a cell is accidentally queued twice before being evaluated.
Cell_t queue[512];
uint16_t head = 0;
uint16_t tail = 0;

void enqueue(int8_t x, int8_t y) {
    queue[tail].x = x;
    queue[tail].y = y;
    tail++;
}

Cell_t dequeue() {
    return queue[head++];
}

void Floodfill_Recalculate(void) {
    // 1. Initialize all distances to 255 (Unreachable)
    for (int x = 0; x < 16; x++) {
        for (int y = 0; y < 16; y++) {
            dist[x][y] = 255;
        }
    }

    head = 0; tail = 0;

    // 2. Seed the queue with the center targets
    dist[TARGET_1_X][TARGET_1_Y] = 0; enqueue(TARGET_1_X, TARGET_1_Y);
    dist[TARGET_2_X][TARGET_2_Y] = 0; enqueue(TARGET_2_X, TARGET_2_Y);
    dist[TARGET_3_X][TARGET_3_Y] = 0; enqueue(TARGET_3_X, TARGET_3_Y);
    dist[TARGET_4_X][TARGET_4_Y] = 0; enqueue(TARGET_4_X, TARGET_4_Y);

    // 3. Flood the maze
    while (head < tail) {
        Cell_t curr = dequeue();
        uint8_t d = dist[curr.x][curr.y];
        uint8_t walls = maze_map[curr.x][curr.y];

        // Check North
        if (!(walls & WALL_NORTH) && curr.y < 15 && dist[curr.x][curr.y+1] == 255) {
            dist[curr.x][curr.y+1] = d + 1;
            enqueue(curr.x, curr.y+1);
        }
        // Check East
        if (!(walls & WALL_EAST) && curr.x < 15 && dist[curr.x+1][curr.y] == 255) {
            dist[curr.x+1][curr.y] = d + 1;
            enqueue(curr.x+1, curr.y);
        }
        // Check South
        if (!(walls & WALL_SOUTH) && curr.y > 0 && dist[curr.x][curr.y-1] == 255) {
            dist[curr.x][curr.y-1] = d + 1;
            enqueue(curr.x, curr.y-1);
        }
        // Check West
        if (!(walls & WALL_WEST) && curr.x > 0 && dist[curr.x-1][curr.y] == 255) {
            dist[curr.x-1][curr.y] = d + 1;
            enqueue(curr.x-1, curr.y);
        }
    }
}

Direction_t Floodfill_Get_Best_Dir(int8_t x, int8_t y, Direction_t current_dir) {
    uint8_t min_dist = 255;
    uint8_t walls = maze_map[x][y];

    // Get the distance of all open neighbors (255 if blocked or out of bounds)
    uint8_t dist_n = (!(walls & WALL_NORTH) && y < 15) ? dist[x][y+1] : 255;
    uint8_t dist_e = (!(walls & WALL_EAST)  && x < 15) ? dist[x+1][y] : 255;
    uint8_t dist_s = (!(walls & WALL_SOUTH) && y > 0)  ? dist[x][y-1] : 255;
    uint8_t dist_w = (!(walls & WALL_WEST)  && x > 0)  ? dist[x-1][y] : 255;

    // Find the absolute minimum distance among reachable neighbors
    if (dist_n < min_dist) min_dist = dist_n;
    if (dist_e < min_dist) min_dist = dist_e;
    if (dist_s < min_dist) min_dist = dist_s;
    if (dist_w < min_dist) min_dist = dist_w;

    // FIX: Tie-Breaker Logic (MOMENTUM!)
    // If the path straight ahead is equal to the minimum distance, always go straight!
    // This stops Pascal from doing a "staircase" zig-zag across open blocks.
    if (current_dir == NORTH && dist_n == min_dist) return NORTH;
    if (current_dir == EAST  && dist_e == min_dist) return EAST;
    if (current_dir == SOUTH && dist_s == min_dist) return SOUTH;
    if (current_dir == WEST  && dist_w == min_dist) return WEST;

    // If straight isn't an option, just pick the first available optimal turn
    if (dist_n == min_dist) return NORTH;
    if (dist_e == min_dist) return EAST;
    if (dist_s == min_dist) return SOUTH;
    if (dist_w == min_dist) return WEST;

    // Fallback (Should only happen if completely boxed in, requiring a U-Turn eventually)
    return current_dir;
}
