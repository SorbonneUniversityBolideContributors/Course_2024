/*
 * bno055.h
 *
 *  Created on: May 18, 2023
 *      Author: ajuton
 */

#ifndef INC_COVAPSY_BNO055_H_
#define INC_COVAPSY_BNO055_H_

#define ADRESSE_BNO 0x28 //adresse 7 bits du BNO055
#define PAGE_SWAP	0x07
#define ACC_CONF	0x08
#define GYR_CONF_0	0x0A
#define GYR_CONF_1	0x0B
#define MAG_CONF	0x09
#define TEMP_SOURCE	0x40
#define UNIT_SEL	0x3B
#define PWR_MODE	0x3E
#define HEADING		0x1A
#define MODE_REG	0x3D
#define FUSION_MODE	0x0C

#define EULER_HEADING_16bits 0x1A
#define EULER_ROLL_16bits 0x1C
#define EULER_PITCH_16bits 0x1E

#define PWR_MODE 0x3E

#define TEMPERATURE_8bits 0x34
#include "stdint.h"


void bno055_init(void);
int16_t bno055_lecture_16bits(uint8_t num_registre);
uint8_t bno055_lecture_8bits(uint8_t num_registre);

#endif /* INC_COVAPSY_BNO055_H_ */
