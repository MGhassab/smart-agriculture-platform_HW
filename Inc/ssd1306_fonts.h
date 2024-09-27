#include <stdint.h>

#ifndef __SSD1306_FONTS_H__
#define __SSD1306_FONTS_H__

typedef struct {
	const uint8_t FontWidth;    /*!< Font width in pixels */
	uint8_t FontHeight;   /*!< Font height in pixels */
	const uint16_t *data; /*!< Pointer to data font data array */
} FontDef;


extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

extern const uint8_t rssiSingal_5[];
extern const uint8_t rssiSingal_4[];
extern const uint8_t rssiSingal_3[];
extern const uint8_t rssiSingal_2[];
extern const uint8_t rssiSingal_1[];
extern const uint8_t line[128];

extern const uint8_t loraSignal_4[];
extern const uint8_t loraSignal_3[];
extern const uint8_t loraSignal_2[];
extern const uint8_t loraSignal_1[];
extern const uint8_t tapOn [];
extern const uint8_t tapOff[];
extern const uint8_t ldm[];
extern const uint8_t deleteMainScrenn[];
extern const uint8_t noSignal[];
extern const uint8_t checkRight[];
extern const uint8_t serverCnection[]; 

#endif // __SSD1306_FONTS_H__
