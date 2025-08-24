/**
 * Rotating Wireframe Cube
 * Uses Waveshare LCD_1IN28 driver with working DisplayPoint function
 */

#include "pico/stdlib.h"
#include "LCD_1in28.h"
#include "DEV_Config.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define PI 3.14159265359f

// 3D cube vertices (8 corners)
typedef struct {
    float x, y, z;
} vertex_t;

// Initialize cube vertices
vertex_t cube[8] = {
    {-40, -40, -40}, {40, -40, -40}, {40, 40, -40}, {-40, 40, -40},  // Back
    {-40, -40,  40}, {40, -40,  40}, {40, 40,  40}, {-40, 40,  40}   // Front
};

// Edge connections (12 edges)
const int edges[12][2] = {
    {0,1}, {1,2}, {2,3}, {3,0},  // Back face
    {4,5}, {5,6}, {6,7}, {7,4},  // Front face
    {0,4}, {1,5}, {2,6}, {3,7}   // Connecting
};

// Draw a line using Bresenham's algorithm with LCD_1IN28_DisplayPoint
void draw_line(int x0, int y0, int x1, int y1, uint16_t color) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (1) {
        // Only draw if within screen bounds
        if (x0 >= 0 && x0 < 240 && y0 >= 0 && y0 < 240) {
            LCD_1IN28_DisplayPoint(x0, y0, color);
        }
        
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

// Rotate point around Y axis
void rotate_y(vertex_t *v, float angle) {
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    float x = v->x;
    float z = v->z;
    v->x = x * cos_a - z * sin_a;
    v->z = x * sin_a + z * cos_a;
}

// Rotate point around X axis
void rotate_x(vertex_t *v, float angle) {
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    float y = v->y;
    float z = v->z;
    v->y = y * cos_a - z * sin_a;
    v->z = y * sin_a + z * cos_a;
}

// Rotate point around Z axis
void rotate_z(vertex_t *v, float angle) {
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    float x = v->x;
    float y = v->y;
    v->x = x * cos_a - y * sin_a;
    v->y = x * sin_a + y * cos_a;
}

// Project 3D to 2D screen coordinates
void project(vertex_t v, int *x, int *y) {
    // Simple perspective projection
    float distance = 150.0f;
    float z = v.z + distance;
    
    // Project to screen with center at (120, 120)
    *x = (int)(120 + (v.x * 100) / z);
    *y = (int)(120 + (v.y * 100) / z);
}

// Draw the wireframe cube
void draw_cube(vertex_t *vertices) {
    int screen[8][2];
    
    // Project all vertices
    for (int i = 0; i < 8; i++) {
        project(vertices[i], &screen[i][0], &screen[i][1]);
    }
    
    // Draw all edges
    for (int i = 0; i < 12; i++) {
        int v0 = edges[i][0];
        int v1 = edges[i][1];
        
        // Different colors for different edge groups
        uint16_t color = 0xFFFF;  // White default
        if (i < 4) color = 0xF800;      // Red - back face
        else if (i < 8) color = 0x07E0; // Green - front face
        else color = 0x001F;             // Blue - connecting
        
        draw_line(screen[v0][0], screen[v0][1], 
                 screen[v1][0], screen[v1][1], color);
    }
}

// Draw a circle
void draw_circle(int cx, int cy, int r, uint16_t color) {
    for (int angle = 0; angle < 360; angle += 3) {
        int x = cx + (int)(r * cosf(angle * PI / 180));
        int y = cy + (int)(r * sinf(angle * PI / 180));
        if (x >= 0 && x < 240 && y >= 0 && y < 240) {
            LCD_1IN28_DisplayPoint(x, y, color);
        }
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Rotating Wireframe Cube ===\n");
    
    // Initialize hardware
    printf("Initializing hardware...\n");
    if (DEV_Module_Init() != 0) {
        printf("Hardware init failed!\n");
        return -1;
    }
    
    // Set backlight
    DEV_SET_PWM(100);
    
    // Initialize LCD
    printf("Initializing LCD...\n");
    LCD_1IN28_Init(HORIZONTAL);
    LCD_1IN28_Clear(0x0000);  // Black background
    
    printf("Starting animation...\n");
    
    float angle_x = 0, angle_y = 0, angle_z = 0;
    int frame = 0;
    
    while (1) {
        // Clear screen
        LCD_1IN28_Clear(0x0000);
        
        // Reset cube to original position
        vertex_t rotated[8] = {
            {-40, -40, -40}, {40, -40, -40}, {40, 40, -40}, {-40, 40, -40},
            {-40, -40,  40}, {40, -40,  40}, {40, 40,  40}, {-40, 40,  40}
        };
        
        // Apply rotations
        for (int i = 0; i < 8; i++) {
            rotate_x(&rotated[i], angle_x);
            rotate_y(&rotated[i], angle_y);
            rotate_z(&rotated[i], angle_z);
        }
        
        // Draw reference circle
        draw_circle(120, 120, 115, 0x07FF);  // Cyan circle
        
        // Draw the cube
        draw_cube(rotated);
        
        // Draw frame counter dots
        for (int i = 0; i < (frame % 10); i++) {
            LCD_1IN28_DisplayPoint(10 + i * 3, 10, 0xFFE0);  // Yellow
        }
        
        // Update rotation angles
        angle_x += 0.03f;
        angle_y += 0.05f;
        angle_z += 0.01f;
        
        // Keep angles in range
        if (angle_x > 2 * PI) angle_x -= 2 * PI;
        if (angle_y > 2 * PI) angle_y -= 2 * PI;
        if (angle_z > 2 * PI) angle_z -= 2 * PI;
        
        frame++;
        printf("\rFrame: %d", frame);
        
        // Delay for animation (roughly 20 FPS)
        sleep_ms(50);
    }
    
    return 0;
}