/*
 * oled.h
 *
 *  Created on: Dec 11, 2025
 *      Author: jochman03
 */

#ifndef OLED_H
#define OLED_H
#include "main.h"
#include "stm32f411xe.h"

//  OLED display width in pixels.
#define OLED_WIDTH   128

// OLED display height in pixels.
#define OLED_HEIGHT   32

// Number of display memory pages (8 pixels per page).
#define OLED_PAGES   (OLED_HEIGHT / 8)

/**
 * @brief Initializes the OLED display.
 *
 * @param hi2c Pointer to I2C handle.
 */
void OLED_Init(I2C_HandleTypeDef* hi2c);

/**
 * Updates the OLED display with the current frame buffer.
 */
void OLED_Update(void);

/**
 * Clears the OLED frame buffer.
 */
void OLED_Clear(void);

/**
 * Fills the OLED frame buffer.
 */
void OLED_Fill(void);

/**
 * Draws a single pixel on the OLED display buffer.
 *
 * @param x     X coordinate (0 .. OLED_WIDTH - 1).
 * @param y     Y coordinate (0 .. OLED_HEIGHT - 1).
 * @param color Pixel color (0 = OFF, 1 = ON).
 */
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color);

/**
 * Draws a single character on the OLED display buffer.
 *
 * @param x    X coordinate of the character.
 * @param y    Y coordinate of the character.
 * @param c    ASCII character to draw.
 * @param size Font size / scaling factor.
 */
void OLED_DrawChar(uint8_t x, uint8_t y, char c, uint8_t size);

/**
 * Writes a null-terminated text string to the OLED display buffer.
 *
 * @param txt      Pointer to text string.
 * @param x        X coordinate of the text start.
 * @param y        Y coordinate of the text start.
 * @param fontSize Font size / scaling factor.
 */
void OLED_WriteText(char* txt, uint8_t x, uint8_t y, uint8_t fontSize);
#endif
