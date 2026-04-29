#include "ism330dhcx_i2c.h"

/* ================================================================
   SENSITIVITY CONSTANTS
   ================================================================ */
#define SENS_ACCEL   0.000061f   /* ±2g   */
#define SENS_GYRO    0.035000f   /* 1000dps */
#define I2C_TIMEOUT  5u          /* ms     */

/* ================================================================
   LAYER 1 — REGISTER ACCESS
   ================================================================ */

ISM330_Status_t ISM330_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(
        hi2c, ISM330_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1u, I2C_TIMEOUT);
    return (ret == HAL_OK) ? ISM330_OK : ISM330_ERR_I2C;
}

ISM330_Status_t ISM330_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        hi2c, ISM330_ADDR, reg, I2C_MEMADD_SIZE_8BIT, val, 1u, I2C_TIMEOUT);
    return (ret == HAL_OK) ? ISM330_OK : ISM330_ERR_I2C;
}

ISM330_Status_t ISM330_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        hi2c, ISM330_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, I2C_TIMEOUT);
    return (ret == HAL_OK) ? ISM330_OK : ISM330_ERR_I2C;
}

ISM330_Status_t ISM330_ModifyReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t current = 0u;
    ISM330_Status_t ret = ISM330_ReadReg(hi2c, reg, &current);
    if (ret != ISM330_OK) { return ret; }
    current = (uint8_t)((current & ~mask) | (val & mask));
    return ISM330_WriteReg(hi2c, reg, current);
}

/* ================================================================
   LAYER 2 — SENSOR CONTROL
   ================================================================ */

ISM330_Status_t ISM330_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0u;
    ISM330_Status_t ret;

    /* 1. Verify connection */
    ret = ISM330_ReadReg(hi2c, ISM330_REG_WHO_AM_I, &who);
    if (ret != ISM330_OK)           { return ISM330_ERR_I2C; }
    if (who != ISM330_WHO_AM_I_VAL) { return ISM330_ERR_ID;  }

    /* 2. Reset and configure general settings */
    ret = ISM330_WriteReg(hi2c, ISM330_REG_CTRL3_C, ISM330_CTRL3_DEFAULT);
    if (ret != ISM330_OK) { return ret; }
    HAL_Delay(10u);

    /* 3. Configure Accelerometer (833Hz, ±2g) */
    ret = ISM330_WriteReg(hi2c, ISM330_REG_CTRL1_XL, ISM330_XL_ODR_833Hz | ISM330_XL_FS_2G);
    if (ret != ISM330_OK) { return ret; }

    /* 4. Configure Gyroscope (833Hz, 1000dps) */
    ret = ISM330_WriteReg(hi2c, ISM330_REG_CTRL2_G, ISM330_G_ODR_833Hz | ISM330_G_FS_1000dps);
    if (ret != ISM330_OK) { return ret; }

    /* 5. Enable the Gyroscope Hardware Low-Pass Filter (LPF1) */
    ret = ISM330_WriteReg(hi2c, ISM330_REG_CTRL4_C, ISM330_CTRL4_LPF1_EN);
    if (ret != ISM330_OK) { return ret; }

    /* 6. Set the filter cutoff to ~33Hz to block high-frequency motor vibration */
    ret = ISM330_WriteReg(hi2c, ISM330_REG_CTRL6_C, ISM330_CTRL6_FTYPE_33HZ);
    if (ret != ISM330_OK) { return ret; }

    return ISM330_OK;
}

ISM330_Status_t ISM330_SoftReset(I2C_HandleTypeDef *hi2c)
{
    ISM330_Status_t ret = ISM330_WriteReg(hi2c, ISM330_REG_CTRL3_C, ISM330_CTRL3_SW_RESET);
    if (ret != ISM330_OK) { return ret; }

    uint8_t ctrl = ISM330_CTRL3_SW_RESET;
    uint32_t start = HAL_GetTick();

    /* 100ms timeout using standard HAL_GetTick */
    while ((ctrl & ISM330_CTRL3_SW_RESET) && ((HAL_GetTick() - start) < 100u)) {
        ISM330_ReadReg(hi2c, ISM330_REG_CTRL3_C, &ctrl);
        HAL_Delay(1u);
    }
    HAL_Delay(10u);
    return ISM330_OK;
}

ISM330_Status_t ISM330_ReadRaw(I2C_HandleTypeDef *hi2c, ISM330_Raw_t *raw)
{
    uint8_t buf[14];
    ISM330_Status_t ret = ISM330_ReadRegs(hi2c, ISM330_REG_OUT_TEMP_L, buf, 14u);
    if (ret != ISM330_OK) { return ret; }

    raw->temp = (int16_t)((uint16_t)buf[1]  << 8 | buf[0]);
    raw->gx   = (int16_t)((uint16_t)buf[3]  << 8 | buf[2]);
    raw->gy   = (int16_t)((uint16_t)buf[5]  << 8 | buf[4]);
    raw->gz   = (int16_t)((uint16_t)buf[7]  << 8 | buf[6]);
    raw->ax   = (int16_t)((uint16_t)buf[9]  << 8 | buf[8]);
    raw->ay   = (int16_t)((uint16_t)buf[11] << 8 | buf[10]);
    raw->az   = (int16_t)((uint16_t)buf[13] << 8 | buf[12]);

    return ISM330_OK;
}

/* Used ONLY during setup/calibration. NEVER use inside the TIM6 interrupt. */
ISM330_Status_t ISM330_WaitAndRead(I2C_HandleTypeDef *hi2c, ISM330_Raw_t *raw)
{
    uint8_t  status = 0u;
    uint32_t start  = HAL_GetTick();

    do {
        ISM330_Status_t ret = ISM330_ReadReg(hi2c, ISM330_REG_STATUS, &status);
        if (ret != ISM330_OK) { return ISM330_ERR_I2C; }

        /* 10ms timeout */
        if ((HAL_GetTick() - start) >= 10u) { return ISM330_ERR_TOUT; }

        /* If data is NOT ready, yield for 1ms */
        if ((status & ISM330_STATUS_BOTH) != ISM330_STATUS_BOTH)
        {
            HAL_Delay(1u);
        }

    } while ((status & ISM330_STATUS_BOTH) != ISM330_STATUS_BOTH);

    return ISM330_ReadRaw(hi2c, raw);
}

void ISM330_ToPhysical(ISM330_Raw_t *raw, ISM330_Phys_t *phys)
{
    phys->ax   = (float)raw->ax * SENS_ACCEL;
    phys->ay   = (float)raw->ay * SENS_ACCEL;
    phys->az   = (float)raw->az * SENS_ACCEL;
    phys->gx   = (float)raw->gx * SENS_GYRO;
    phys->gy   = (float)raw->gy * SENS_GYRO;
    phys->gz   = (float)raw->gz * SENS_GYRO;
}

/* ================================================================
   LAYER 3 — UTILITY
   ================================================================ */

const char *ISM330_StatusStr(ISM330_Status_t s)
{
    switch (s) {
        case ISM330_OK:       return "OK";
        case ISM330_ERR_I2C:  return "I2C error — check wiring";
        case ISM330_ERR_ID:   return "Wrong WHO_AM_I — check I2C address";
        case ISM330_ERR_TOUT: return "Data-ready timeout";
        default:              return "Unknown error";
    }
}
