#include "oled.h"
#include "fonts.h"

#define OLED_I2C_ADDR  (0x3C << 1)

#define CMD_DISPLAY_OFF 0xAE
#define CMD_DISPLAY_ON  0xAF
#define CMD_SET_CONTRAST 0x81
#define CMD_NORMAL_DISPLAY 0xA6
#define CMD_DISPLAY_ALL_ON_RESUME 0xA4
#define CMD_MEMORY_MODE 0x20
#define CMD_COLUMN_ADDR 0x21
#define CMD_PAGE_ADDR   0x22
#define CMD_CHARGE_PUMP 0x8D
#define CMD_SEG_REMAP   0xA1
#define CMD_COM_SCAN_DEC 0xC8
#define CMD_COM_PINS   0xDA
#define CMD_SET_MULTIPLEX 0xA8
#define CMD_DISPLAY_OFFSET 0xD3
#define CMD_SET_START_LINE 0x40
#define CMD_CLOCK_DIV 0xD5

static I2C_HandleTypeDef* oled_i2c;
static uint8_t oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8];

static void oled_write_command(uint8_t cmd) {
	uint8_t data[2] = { 0x00, cmd };
	HAL_I2C_Master_Transmit(oled_i2c, OLED_I2C_ADDR, data, 2, HAL_MAX_DELAY);
}

static void oled_write_data(uint8_t* data, uint16_t size) {
	HAL_I2C_Mem_Write(oled_i2c, OLED_I2C_ADDR, 0x40, I2C_MEMADD_SIZE_8BIT, data,
			size, HAL_MAX_DELAY);
}

void OLED_Init(I2C_HandleTypeDef* hi2c) {
	oled_i2c = hi2c;
	HAL_Delay(200);
	oled_write_command(CMD_DISPLAY_OFF);
	oled_write_command(CMD_CLOCK_DIV);
	oled_write_command(0x80);
	oled_write_command(CMD_SET_MULTIPLEX);
	oled_write_command(OLED_HEIGHT - 1);
	oled_write_command(CMD_DISPLAY_OFFSET);
	oled_write_command(0x00);
	oled_write_command(CMD_SET_START_LINE | 0x00);
	oled_write_command(CMD_CHARGE_PUMP);
	oled_write_command(0x14);
	oled_write_command(CMD_MEMORY_MODE);
	oled_write_command(0x02);
	oled_write_command(CMD_SEG_REMAP);
	oled_write_command(CMD_COM_SCAN_DEC);
	oled_write_command(CMD_COM_PINS);
	oled_write_command(0x02);
	oled_write_command(CMD_SET_CONTRAST);
	oled_write_command(0x8F);
	oled_write_command(CMD_DISPLAY_ALL_ON_RESUME);
	oled_write_command(CMD_NORMAL_DISPLAY);
	oled_write_command(CMD_DISPLAY_ON);

	HAL_Delay(100);

	OLED_Clear();
	OLED_Update();
}

void OLED_Update(void) {
	for (uint8_t page = 0; page < OLED_PAGES; page++) {
		oled_write_command(CMD_PAGE_ADDR);
		oled_write_command(page);
		oled_write_command(page);

		oled_write_command(CMD_COLUMN_ADDR);
		oled_write_command(0);
		oled_write_command(OLED_WIDTH - 1);

		oled_write_data(&oled_buffer[OLED_WIDTH * page], OLED_WIDTH);
	}
}

void OLED_Clear(void) {
	for (uint32_t i = 0; i < sizeof(oled_buffer); i++) {
		oled_buffer[i] = 0x00;
	}
}

void OLED_Fill(void) {
	for (uint32_t i = 0; i < sizeof(oled_buffer); i++) {
		oled_buffer[i] = 0xFF;
	}
}

void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color) {
	if (x >= OLED_WIDTH || y >= OLED_HEIGHT) {
		return;
	}

	uint16_t index = x + (y / 8) * OLED_WIDTH;

	if (color) {
		oled_buffer[index] |= (1 << (y % 8));
	} else {
		oled_buffer[index] &= ~(1 << (y % 8));
	}
}

void OLED_DrawChar(uint8_t x, uint8_t y, char c, uint8_t size) {
	if (c < 32) {
		return;
	}

	if (size == 6) {
		const uint8_t* ch = &(ssd1306xled_font6x8[4 + 6 * (c - 32)]);
		for (uint8_t i = 0; i < 6; i++) {
			for (uint8_t j = 0; j < 8; j++) {
				OLED_DrawPixel(x + i, y + j, (ch[i] & (1 << j)) ? 1 : 0);
			}
		}
	} else if (size == 8) {
		const uint8_t* ch = &(ssd1306xled_font8x16[4 + 16 * (c - 32)]);
		const uint8_t* upper = ch;
		const uint8_t* lower = ch + 8;
		uint8_t color;
		for (uint8_t col = 0; col < 8; col++) {
			for (uint8_t row = 0; row < 8; row++) {
				color = (upper[col] & (1 << row)) ? 1 : 0;
				OLED_DrawPixel(x + col, y + row, color);
			}
			for (uint8_t row = 0; row < 8; row++) {
				color = (lower[col] & (1 << row)) ? 1 : 0;
				OLED_DrawPixel(x + col, y + 8 + row, color);
			}
		}
	}
}

void OLED_WriteText(char* txt, uint8_t x, uint8_t y, uint8_t fontSize) {
	uint8_t cx = x;
	uint8_t cy = y;
	while (*txt != '\0') {
		if (*txt == '\n') {
			cy += fontSize;
			cx = x;
		} else {
			OLED_DrawChar(cx, cy, *txt, fontSize);
			cx += fontSize;
		}
		txt++;
	}
}
