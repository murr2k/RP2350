/**
 * Minimal LCD Test - Just clear screen to colors
 * Using Waveshare's exact initialization
 */

#include "pico/stdlib.h"
#include "LCD_1in28.h"
#include "DEV_Config.h"
#include <stdio.h>

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n=== Minimal LCD Test ===\n");
    printf("Just cycling colors to verify LCD works\n\n");
    
    // Initialize hardware using Waveshare's function
    printf("Initializing hardware...\n");
    if (DEV_Module_Init() != 0) {
        printf("Failed to initialize hardware!\n");
        return -1;
    }
    
    // Set backlight to full
    DEV_SET_PWM(100);
    
    // Initialize LCD
    printf("Initializing LCD...\n");
    LCD_1IN28_Init(HORIZONTAL);
    
    // Clear to white first
    LCD_1IN28_Clear(0xFFFF);
    
    printf("Cycling colors...\n");
    
    // Color cycle
    uint16_t colors[] = {
        0xF800,  // Red
        0x07E0,  // Green  
        0x001F,  // Blue
        0xFFFF,  // White
        0x0000,  // Black
        0xFFE0,  // Yellow
        0xF81F,  // Magenta
        0x07FF   // Cyan
    };
    
    const char *names[] = {
        "RED", "GREEN", "BLUE", "WHITE",
        "BLACK", "YELLOW", "MAGENTA", "CYAN"
    };
    
    int idx = 0;
    while (1) {
        printf("Displaying: %s (0x%04X)\n", names[idx], colors[idx]);
        LCD_1IN28_Clear(colors[idx]);
        sleep_ms(2000);
        idx = (idx + 1) % 8;
    }
    
    return 0;
}