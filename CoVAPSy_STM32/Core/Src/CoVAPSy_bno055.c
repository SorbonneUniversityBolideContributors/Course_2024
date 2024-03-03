#include <CoVAPSy_bno055.h>
#include "i2c.h"
// Initialisation du BNO055
void bno055_init(void) {
	uint8_t donnees_Tx_i2c[2];
	uint8_t donnees_Rx_i2c[2];


	/**
	******************************************************************************
	* This code uses a lot of HAL. Could probably do with a simpler bare metal
	* implementation. Current performance exceeds that of the sensor, so it is
	* not a priority. But could be a fun exercise in embedded programming :)))
	******************************************************************************
	*/
	HAL_Delay(2000);
	donnees_Tx_i2c[0] = 0x39;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)		;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 1, 1000);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Rx_i2c,	1, 20);

	donnees_Tx_i2c[0] = 0x37;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)		;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 1, 1000);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Rx_i2c,	1, 20);



	donnees_Tx_i2c[0] = 0x3F;
	donnees_Tx_i2c[1] = 0;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)		;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Rx_i2c,	1, 20);

	HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);


	HAL_Delay(800); //delai 2000 ms

	// A envoyer lors du premier test
	donnees_Tx_i2c[0] = PAGE_SWAP;
	donnees_Tx_i2c[1] = 1;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) ;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);


	donnees_Tx_i2c[0] = ACC_CONF;
	donnees_Tx_i2c[1] = 0x08;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) ;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = GYR_CONF_0;
	donnees_Tx_i2c[1] = 0x23;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)	;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = GYR_CONF_1;
	donnees_Tx_i2c[1] = 0;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) ;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = MAG_CONF;
	donnees_Tx_i2c[1] = 0x1B;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) ;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = PAGE_SWAP;
	donnees_Tx_i2c[1] = 0;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) ;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = TEMP_SOURCE;
	donnees_Tx_i2c[1] = 0x01;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) ;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = UNIT_SEL;
	donnees_Tx_i2c[1] = 0x01;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) ;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = PWR_MODE;
	donnees_Tx_i2c[1] = 0x00;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) ;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = MODE_REG;
	donnees_Tx_i2c[1] = FUSION_MODE;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) ;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);
}

int16_t bno055_lecture_16bits(uint8_t num_registre) {
	uint8_t donnees_Tx_i2c[1];
	uint8_t donnees_Rx_i2c[2];
	donnees_Tx_i2c[0] = num_registre;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 1, 1000);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Rx_i2c,	2, 1);
	return ((uint16_t)(donnees_Rx_i2c[1]<<8) + donnees_Rx_i2c[0]);
}
uint8_t bno055_lecture_8bits(uint8_t num_registre) {
	uint8_t donnees_Tx_i2c[1];
	uint8_t donnees_Rx_i2c[1];
	donnees_Tx_i2c[0]=num_registre;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)ADRESSE_BNO<<1, donnees_Tx_i2c, 1, 1000);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)ADRESSE_BNO<<1, donnees_Rx_i2c, 1, 1);
	return (donnees_Rx_i2c[0]);

}

