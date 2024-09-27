#ifndef	_EE24_H
#define	_EE24_H

/*
  Author:     Nima Askari
  WebSite:    http://www.github.com/NimaLTD
  Instagram:  http://instagram.com/github.NimaLTD
  Youtube:    https://www.youtube.com/channel/UCUhY7qY1klJm1d2kulr9ckw
  
  Version:    2.2.0
  
  
  Reversion History:
  
  (2.2.0)
  Add erase chip function.
  
  (2.1.0)
  Fix write bytes.
  
  (2.0.0)
  Rewrite again.

*/

#include <stdbool.h>
#include <stdint.h>
#include "ee24Config.h"
#include "main.h"

bool    ee24_isConnected(I2C_HandleTypeDef* i2c);
bool    ee24_write(I2C_HandleTypeDef* i2c, uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);	
bool    ee24_read(I2C_HandleTypeDef* i2c, uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);
bool    ee24_eraseChip(I2C_HandleTypeDef* i2c);

#endif
