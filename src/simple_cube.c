/**
 * Simple Rotating Cube Test
 * No IMU - just tests LCD drawing with a rotating wireframe cube
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// LCD Pin definitions
#define LCD_DC_PIN      8
#define LCD_CS_PIN      9
#define LCD_CLK_PIN     10
#define LCD_MOSI_PIN    11
#define LCD_RST_PIN     12
#define LCD_BL_PIN      25

// Display dimensions
#define LCD_WIDTH  240
#define LCD_HEIGHT 240

// Colors (RGB565)
#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define BLUE    0x001F
#define YELLOW  0xFFE0
#define CYAN    0x07FF
#define MAGENTA 0xF81F

// Simple 3D cube vertices
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

// LCD functions
static void lcd_write_command(uint8_t cmd) {
    gpio_put(LCD_DC_PIN, 0);
    spi_write_blocking(spi1, &cmd, 1);
}

static void lcd_write_data(uint8_t data) {
    gpio_put(LCD_DC_PIN, 1);
    spi_write_blocking(spi1, &data, 1);
}

static void lcd_write_data16(uint16_t data) {
    uint8_t buf[2];
    buf[0] = data >> 8;
    buf[1] = data & 0xFF;
    gpio_put(LCD_DC_PIN, 1);
    spi_write_blocking(spi1, buf, 2);
}

static void lcd_reset(void) {
    gpio_put(LCD_RST_PIN, 1);
    sleep_ms(100);
    gpio_put(LCD_RST_PIN, 0);
    sleep_ms(100);
    gpio_put(LCD_RST_PIN, 1);
    gpio_put(LCD_CS_PIN, 0);  // Keep CS LOW
    sleep_ms(100);
}

static void lcd_init_reg(void) {
    // Minimal Waveshare init sequence
    lcd_write_command(0xEF);
    lcd_write_command(0xEB);
    lcd_write_data(0x14);
    
    lcd_write_command(0xFE);
    lcd_write_command(0xEF);
    
    lcd_write_command(0xEB);
    lcd_write_data(0x14);
    
    lcd_write_command(0x84);
    lcd_write_data(0x40);
    
    lcd_write_command(0x85);
    lcd_write_data(0xFF);
    
    lcd_write_command(0x86);
    lcd_write_data(0xFF);
    
    lcd_write_command(0x87);
    lcd_write_data(0xFF);
    
    lcd_write_command(0x88);
    lcd_write_data(0x0A);
    
    lcd_write_command(0x89);
    lcd_write_data(0x21);
    
    lcd_write_command(0x8A);
    lcd_write_data(0x00);
    
    lcd_write_command(0x8B);
    lcd_write_data(0x80);
    
    lcd_write_command(0x8C);
    lcd_write_data(0x01);
    
    lcd_write_command(0x8D);
    lcd_write_data(0x01);
    
    lcd_write_command(0x8E);
    lcd_write_data(0xFF);
    
    lcd_write_command(0x8F);
    lcd_write_data(0xFF);
    
    lcd_write_command(0xB6);
    lcd_write_data(0x00);
    lcd_write_data(0x20);
    
    lcd_write_command(0x36);
    lcd_write_data(0x08);
    
    lcd_write_command(0x3A);
    lcd_write_data(0x05);  // 16-bit color
    
    lcd_write_command(0x90);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    
    lcd_write_command(0xBD);
    lcd_write_data(0x06);
    
    lcd_write_command(0xBC);
    lcd_write_data(0x00);
    
    lcd_write_command(0xFF);
    lcd_write_data(0x60);
    lcd_write_data(0x01);
    lcd_write_data(0x04);
    
    lcd_write_command(0xC3);
    lcd_write_data(0x13);
    lcd_write_command(0xC4);
    lcd_write_data(0x13);
    
    lcd_write_command(0xC9);
    lcd_write_data(0x22);
    
    lcd_write_command(0xBE);
    lcd_write_data(0x11);
    
    lcd_write_command(0xE1);
    lcd_write_data(0x10);
    lcd_write_data(0x0E);
    
    lcd_write_command(0xDF);
    lcd_write_data(0x21);
    lcd_write_data(0x0C);
    lcd_write_data(0x02);
    
    lcd_write_command(0xF0);
    lcd_write_data(0x45);
    lcd_write_data(0x09);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    lcd_write_data(0x26);
    lcd_write_data(0x2A);
    
    lcd_write_command(0xF1);
    lcd_write_data(0x43);
    lcd_write_data(0x70);
    lcd_write_data(0x72);
    lcd_write_data(0x36);
    lcd_write_data(0x37);
    lcd_write_data(0x6F);
    
    lcd_write_command(0xF2);
    lcd_write_data(0x45);
    lcd_write_data(0x09);
    lcd_write_data(0x08);
    lcd_write_data(0x08);
    lcd_write_data(0x26);
    lcd_write_data(0x2A);
    
    lcd_write_command(0xF3);
    lcd_write_data(0x43);
    lcd_write_data(0x70);
    lcd_write_data(0x72);
    lcd_write_data(0x36);
    lcd_write_data(0x37);
    lcd_write_data(0x6F);
    
    lcd_write_command(0xED);
    lcd_write_data(0x1B);
    lcd_write_data(0x0B);
    
    lcd_write_command(0xAE);
    lcd_write_data(0x77);
    
    lcd_write_command(0xCD);
    lcd_write_data(0x63);
    
    lcd_write_command(0x70);
    lcd_write_data(0x07);
    lcd_write_data(0x07);
    lcd_write_data(0x04);
    lcd_write_data(0x0E);
    lcd_write_data(0x0F);
    lcd_write_data(0x09);
    lcd_write_data(0x07);
    lcd_write_data(0x08);
    lcd_write_data(0x03);
    
    lcd_write_command(0xE8);
    lcd_write_data(0x34);
    
    lcd_write_command(0x62);
    for (int i = 0; i < 12; i++) {
        lcd_write_data(0x18);
        lcd_write_data(0x0D);
        lcd_write_data(0x71);
        lcd_write_data(0xED);
        lcd_write_data(0x70);
        lcd_write_data(0x70);
    }
    
    lcd_write_command(0x63);
    for (int i = 0; i < 12; i++) {
        lcd_write_data(0x18);
        lcd_write_data(0x11);
        lcd_write_data(0x71);
        lcd_write_data(0xF1);
        lcd_write_data(0x70);
        lcd_write_data(0x70);
    }
    
    lcd_write_command(0x64);
    lcd_write_data(0x28);
    lcd_write_data(0x29);
    lcd_write_data(0xF1);
    lcd_write_data(0x01);
    lcd_write_data(0xF1);
    lcd_write_data(0x00);
    lcd_write_data(0x07);
    
    lcd_write_command(0x66);
    for (int i = 0; i < 10; i++) {
        lcd_write_data(0x3C);
    }
    
    lcd_write_command(0x67);
    for (int i = 0; i < 10; i++) {
        lcd_write_data(0x00);
    }
    
    lcd_write_command(0x74);
    lcd_write_data(0x10);
    lcd_write_data(0x85);
    lcd_write_data(0x80);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x4E);
    lcd_write_data(0x00);
    
    lcd_write_command(0x98);
    lcd_write_data(0x3E);
    lcd_write_data(0x07);
    
    lcd_write_command(0x35);  // Tearing effect ON
    lcd_write_command(0x21);  // Inversion ON
    
    lcd_write_command(0x11);  // Sleep out
    sleep_ms(120);
    
    lcd_write_command(0x29);  // Display ON
    sleep_ms(20);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    lcd_write_command(0x2A);
    lcd_write_data(x0 >> 8);
    lcd_write_data(x0 & 0xFF);
    lcd_write_data(x1 >> 8);
    lcd_write_data(x1 & 0xFF);
    
    lcd_write_command(0x2B);
    lcd_write_data(y0 >> 8);
    lcd_write_data(y0 & 0xFF);
    lcd_write_data(y1 >> 8);
    lcd_write_data(y1 & 0xFF);
    
    lcd_write_command(0x2C);
}

static void lcd_clear(uint16_t color) {
    lcd_set_window(0, 0, 239, 239);
    gpio_put(LCD_DC_PIN, 1);
    
    uint8_t data[2];
    data[0] = color >> 8;
    data[1] = color & 0xFF;
    
    for (int i = 0; i < 240 * 240; i++) {
        spi_write_blocking(spi1, data, 2);
    }
}

static void lcd_draw_pixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT) return;
    
    lcd_set_window(x, y, x, y);
    lcd_write_data16(color);
}

// Simple line drawing
static void lcd_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t dx = abs(x1 - x0);
    int16_t dy = abs(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;
    
    while (1) {
        lcd_draw_pixel(x0, y0, color);
        
        if (x0 == x1 && y0 == y1) break;
        
        int16_t e2 = 2 * err;
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
static void rotate_y(float angle) {
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
static void rotate_x(float angle) {
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
static void draw_cube(void) {
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
        uint16_t color = WHITE;
        if (i < 4) color = RED;       // Back face
        else if (i < 8) color = GREEN; // Front face
        else color = BLUE;             // Connecting edges
        
        lcd_draw_line(x0, y0, x1, y1, color);
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Simple Rotating Cube Test ===\n");
    printf("Testing LCD drawing with rotating wireframe\n\n");
    
    // Initialize LCD pins
    gpio_init(LCD_DC_PIN);
    gpio_set_dir(LCD_DC_PIN, GPIO_OUT);
    
    gpio_init(LCD_CS_PIN);
    gpio_set_dir(LCD_CS_PIN, GPIO_OUT);
    gpio_put(LCD_CS_PIN, 1);
    
    gpio_init(LCD_RST_PIN);
    gpio_set_dir(LCD_RST_PIN, GPIO_OUT);
    
    // Initialize SPI
    spi_init(spi1, 62500000);  // 62.5MHz
    gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);
    
    // Initialize backlight
    gpio_set_function(LCD_BL_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(LCD_BL_PIN);
    pwm_set_wrap(slice, 255);
    pwm_set_gpio_level(LCD_BL_PIN, 255);  // Full brightness
    pwm_set_enabled(slice, true);
    
    printf("Initializing LCD...\n");
    lcd_reset();
    lcd_init_reg();
    
    printf("Starting animation...\n");
    
    // Animation loop
    float angle = 0;
    int frame = 0;
    
    while (1) {
        // Clear screen
        lcd_clear(BLACK);
        
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
        
        // Draw frame counter in corner
        for (int i = 0; i < (frame % 10); i++) {
            lcd_draw_pixel(10 + i * 3, 10, YELLOW);
            lcd_draw_pixel(10 + i * 3, 11, YELLOW);
        }
        
        // Update angle
        angle += 0.05f;
        if (angle > 6.28f) angle -= 6.28f;
        
        frame++;
        printf("\rFrame: %d, Angle: %.2f", frame, angle);
        
        // ~20 FPS
        sleep_ms(50);
    }
    
    return 0;
}