/*
 * ism330dhcx.c
 *
 * Drop into Core/Src. Requires Core/Inc/ism330dhcx.h.
 * Assumes CubeMX generated `hspi2` (SPI2 handle) in main.c/main.h and that
 * the CS pin was labeled "IMU_CS" in the .ioc (produces IMU_CS_Pin /
 * IMU_CS_GPIO_Port in main.h). Rename below if your label differs.
 */

#include "ism330dhcx.h"
#include "main.h"   /* for hspi2, IMU_CS_Pin, IMU_CS_GPIO_Port */

extern SPI_HandleTypeDef hspi2;

static inline void IMU_CS_Low(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static inline void IMU_CS_High(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

/* ST SPI convention: MSB of the address byte = 1 for read, 0 for write. */

uint8_t ISM330DHCX_ReadReg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80u), 0x00u };
    uint8_t rx[2] = { 0 };

    IMU_CS_Low();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, HAL_MAX_DELAY);
    IMU_CS_High();

    return rx[1];   /* first byte received is garbage — chip only starts driving SDO at bit 8 */
}

void ISM330DHCX_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), val };

    IMU_CS_Low();
    HAL_SPI_Transmit(&hspi2, tx, 2, HAL_MAX_DELAY);
    IMU_CS_High();
}

void ISM330DHCX_ReadBurst(uint8_t startReg, uint8_t *buf, uint8_t len)
{
    uint8_t tx = startReg | 0x80u;

    IMU_CS_Low();
    HAL_SPI_Transmit(&hspi2, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, buf, len, HAL_MAX_DELAY);
    IMU_CS_High();
}

/*
 * Bring-up sequence:
 *   1. WHO_AM_I (0x0F) must read back 0x6B before touching anything else.
 *   2. CTRL3_C written FIRST -> enables BDU + auto-increment (IF_INC),
 *      required before the burst read used in ISM330DHCX_ReadData().
 *   3. CTRL1_XL / CTRL2_G bring the accel/gyro out of power-down.
 */
HAL_StatusTypeDef ISM330DHCX_Init(void)
{
    if (ISM330DHCX_ReadReg(ISM330DHCX_WHO_AM_I) != ISM330DHCX_WHO_AM_I_VAL) {
        return HAL_ERROR;
    }

    ISM330DHCX_WriteReg(ISM330DHCX_CTRL3_C, ISM330DHCX_CTRL3_C_INIT);
    ISM330DHCX_WriteReg(ISM330DHCX_CTRL1_XL, ISM330DHCX_CTRL1_XL_INIT);
    ISM330DHCX_WriteReg(ISM330DHCX_CTRL2_G, ISM330DHCX_CTRL2_G_INIT);

    return HAL_OK;
}

/*
 * Single 12-byte burst from OUTX_L_G (0x22): gyro X/Y/Z then accel X/Y/Z,
 * each little-endian int16. Registers are contiguous 0x22-0x2D so one
 * burst grabs both sensors in a single CS-low window (BDU keeps it atomic).
 */
void ISM330DHCX_ReadData(ISM330DHCX_Data *data)
{
    uint8_t raw[12];
    int16_t gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw;

    ISM330DHCX_ReadBurst(ISM330DHCX_OUTX_L_G, raw, 12);

    gx_raw = (int16_t)((raw[1]  << 8) | raw[0]);
    gy_raw = (int16_t)((raw[3]  << 8) | raw[2]);
    gz_raw = (int16_t)((raw[5]  << 8) | raw[4]);
    ax_raw = (int16_t)((raw[7]  << 8) | raw[6]);
    ay_raw = (int16_t)((raw[9]  << 8) | raw[8]);
    az_raw = (int16_t)((raw[11] << 8) | raw[10]);

    data->gx = gx_raw * ISM330DHCX_GYRO_SENS_DPS;
    data->gy = gy_raw * ISM330DHCX_GYRO_SENS_DPS;
    data->gz = gz_raw * ISM330DHCX_GYRO_SENS_DPS;
    data->ax = ax_raw * ISM330DHCX_ACCEL_SENS_G;
    data->ay = ay_raw * ISM330DHCX_ACCEL_SENS_G;
    data->az = az_raw * ISM330DHCX_ACCEL_SENS_G;
}
