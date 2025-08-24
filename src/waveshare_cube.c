/**
 * Waveshare LCD Demo - Using Official Driver
 * Simple rotating cube using Waveshare's LCD_1in28 library
 */

#include "pico/stdlib.h"
#include "LCD_1in28.h"
#include "DEV_Config.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#define PI 3.14159265359f

// 3D cube vertices
static float vertices[8][3] = {
    {-40, -40, -40}, {40, -40, -40}, {40, 40, -40}, {-40, 40, -40},
    {-40, -40,  40}, {40, -40,  40}, {40, 40,  40}, {-40, 40,  40}
};

// Edges connecting vertices
static const int edges[12][2] = {
    {0,1}, {1,2}, {2,3}, {3,0},  // Back face
    {4,5}, {5,6}, {6,7}, {7,4},  // Front face
    {0,4}, {1,5}, {2,6}, {3,7}   // Connecting edges
};

// Colors
#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define BLUE    0x001F
#define YELLOW  0xFFE0
#define CYAN    0x07FF

// Draw a line using Waveshare's DisplayPoint
void draw_line(int x0, int y0, int x1, int y1, UWORD color) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (1) {
        LCD_1IN28_DisplayPoint(x0, y0, color);
        
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

// Rotate vertices around Y axis
void rotate_y(float angle) {
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    
    for (int i = 0; i < 8; i++) {
        float x = vertices[i][0];
        float z = vertices[i][2];
        vertices[i][0] = x * cos_a - z * sin_a;
        vertices[i][2] = x * sin_a + z * cos_a;
    }
}

// Rotate vertices around X axis
void rotate_x(float angle) {
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    
    for (int i = 0; i < 8; i++) {
        float y = vertices[i][1];
        float z = vertices[i][2];
        vertices[i][1] = y * cos_a - z * sin_a;
        vertices[i][2] = y * sin_a + z * cos_a;
    }
}

// Draw the cube
void draw_cube(void) {
    // Project 3D to 2D and draw edges
    for (int i = 0; i < 12; i++) {
        int v0 = edges[i][0];
        int v1 = edges[i][1];
        
        // Simple perspective projection
        float z0 = vertices[v0][2] + 150;
        float z1 = vertices[v1][2] + 150;
        
        int x0 = (int)(120 + vertices[v0][0] * 100 / z0);
        int y0 = (int)(120 + vertices[v0][1] * 100 / z0);
        int x1 = (int)(120 + vertices[v1][0] * 100 / z1);
        int y1 = (int)(120 + vertices[v1][1] * 100 / z1);
        
        // Different colors for different edge groups
        UWORD color = WHITE;
        if (i < 4) color = RED;       // Back face
        else if (i < 8) color = GREEN; // Front face
        else color = BLUE;             // Connecting edges
        
        draw_line(x0, y0, x1, y1, color);
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Waveshare LCD Cube Demo ===\n");
    printf("Using official Waveshare driver\n\n");
    
    // Initialize using Waveshare's functions
    printf("Initializing hardware...\n");
    if (DEV_Module_Init() != 0) {
        printf("Failed to initialize hardware!\n");
        return -1;
    }
    
    // Set PWM for backlight
    DEV_SET_PWM(100);  // Full brightness
    
    // Initialize LCD
    printf("Initializing LCD...\n");
    LCD_1IN28_Init(HORIZONTAL);
    
    // Clear screen to black
    LCD_1IN28_Clear(BLACK);
    
    printf("Starting animation...\n");
    
    float angle = 0;
    int frame = 0;
    
    while (1) {
        // Clear screen
        LCD_1IN28_Clear(BLACK);
        
        // Reset vertices to original positions
        float original[8][3] = {
            {-40, -40, -40}, {40, -40, -40}, {40, 40, -40}, {-40, 40, -40},
            {-40, -40,  40}, {40, -40,  40}, {40, 40,  40}, {-40, 40,  40}
        };
        memcpy(vertices, original, sizeof(vertices));
        
        // Rotate cube
        rotate_y(angle);
        rotate_x(angle * 0.7f);
        
        // Draw cube
        draw_cube();
        
        // Draw a reference circle
        for (int a = 0; a < 360; a += 5) {
            int x = 120 + (int)(115 * cosf(a * PI / 180));
            int y = 120 + (int)(115 * sinf(a * PI / 180));
            LCD_1IN28_DisplayPoint(x, y, CYAN);
        }
        
        // Update angle
        angle += 0.05f;
        if (angle > 2 * PI) angle -= 2 * PI;
        
        frame++;
        printf("\rFrame: %d", frame);
        
        // Delay for animation
        sleep_ms(50);
    }
    
    return 0;
}