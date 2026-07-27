/*
 * ism330dhcx.h
 *
 * Minimal SPI driver for the SparkFun ISM330DHCX 6DoF IMU (SEN-19764)
 * Target: STM32 Nucleo-F446RE, SPI2 (Mode 3: CPOL=1, CPHA=2EDGE), NSS software
 * CS pin: PB6 (adjust IMU_CS_Pin / IMU_CS_GPIO_Port if your CubeMX label differs)
 *
 * NOTE: The I2C address jumper on the back of the SparkFun board MUST be cut
 * (both pads) before wiring SPI, or WHO_AM_I / all reads will return garbage.
 */

#ifndef ISM330DHCX_H_
#define ISM330DHCX_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ---- Register map (subset needed for bring-up) ---- */
#define ISM330DHCX_WHO_AM_I        0x0Fu
#define ISM330DHCX_WHO_AM_I_VAL    0x6Bu

#define ISM330DHCX_CTRL1_XL        0x10u   /* accel ODR / full scale */
#define ISM330DHCX_CTRL2_G         0x11u   /* gyro ODR / full scale  */
#define ISM330DHCX_CTRL3_C         0x12u   /* BDU, IF_INC, etc.      */

#define ISM330DHCX_OUTX_L_G        0x22u   /* gyro output, start of 12-byte burst */
#define ISM330DHCX_OUTX_L_A        0x28u   /* accel output (contiguous after gyro) */

/* ---- Init register values (already derived/verified) ---- */
#define ISM330DHCX_CTRL3_C_INIT    0x44u   /* BDU=1, IF_INC=1 */
#define ISM330DHCX_CTRL1_XL_INIT   0x58u   /* 208 Hz, +/-4g   */
#define ISM330DHCX_CTRL2_G_INIT    0x54u   /* 208 Hz, 500 dps */

/* ---- Sensitivity (little-endian raw counts -> physical units) ---- */
#define ISM330DHCX_ACCEL_SENS_G    0.000122f   /* g/LSB   @ +/-4g   */
#define ISM330DHCX_GYRO_SENS_DPS   0.01750f    /* dps/LSB @ 500 dps */

typedef struct {
    float gx, gy, gz;   /* deg/s */
    float ax, ay, az;   /* g     */
} ISM330DHCX_Data;

/* Low-level */
uint8_t ISM330DHCX_ReadReg(uint8_t reg);
void    ISM330DHCX_WriteReg(uint8_t reg, uint8_t val);
void    ISM330DHCX_ReadBurst(uint8_t startReg, uint8_t *buf, uint8_t len);

/* High-level */
HAL_StatusTypeDef ISM330DHCX_Init(void);
void ISM330DHCX_ReadData(ISM330DHCX_Data *data);

#endif /* ISM330DHCX_H_ */
