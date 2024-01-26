/*
 * data.h
 *
 *  Created on: Jan 19, 2024
 *      Author: quentinrolland
 */

#ifndef INC_DATA_H_
#define INC_DATA_H_


#include <string.h>

#define SENSOR_TYPE_FOURCHE 0x01
#define SENSOR_TYPE_GYROSCOPE 0x02
#define SENSOR_TYPE_ACCELEROMETRE 0x03
#define SENSOR_TYPE_SONAR 0x04
#define SENSOR_TYPE_IR_GCH 0x05
#define SENSOR_TYPE_IR_DRT 0x06

enum State {
	OK,
	KO
};

typedef struct {
    uint8_t sensorType;
    int16_t roll;
    int16_t yaw;
    int16_t pitch;
    int16_t acceleration;
    int16_t distance_US;
    uint16_t telemetre_gauche;
    uint16_t telemetre_droit;
    uint32_t vitesse_mesuree_mm_s;
} SensorData;

/*
void sendSensorData(SensorData data) {
    uint8_t SPI_TxBuffer[sizeof(SensorData)];
    memcpy(SPI_TxBuffer, &data, sizeof(SensorData));
    // Envoyer SPI_TxBuffer via le bus SPI
}
*/



#endif /* INC_DATA_H_ */
