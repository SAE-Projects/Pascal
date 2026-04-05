/**
 * ============================================================
 *  ism330dhcx_i2c.c
 *  ISM330DHCX IMU driver — implementation
 *
 *  See ism330dhcx_i2c.h for full documentation.
 * ============================================================
 */

#include "ism330dhcx_i2c.h"

/* ============================================================
   SENSITIVITY CONSTANTS
   ============================================================
   These depend on the full-scale (FS) setting chosen in Init.
   Change them if you change FS in ISM330_Init().

   Accelerometer sensitivity (g per LSB):
     ±2g  → 0.000061 g/LSB
     ±4g  → 0.000122 g/LSB
     ±8g  → 0.000244 g/LSB
     ±16g → 0.000488 g/LSB

   Gyroscope sensitivity (dps per LSB):
     125 dps  → 0.004375 dps/LSB
     250 dps  → 0.008750 dps/LSB
     500 dps  → 0.017500 dps/LSB
     1000 dps → 0.035000 dps/LSB
     2000 dps → 0.070000 dps/LSB
   ============================================================ */
#define SENS_ACCEL_2G      0.000061f    /* g / LSB  — matches ISM330_XL_FS_2G  */
#define SENS_GYRO_250DPS   0.008750f    /* dps / LSB — matches ISM330_G_FS_250dps */

/* I2C timeout in milliseconds */
#define I2C_TIMEOUT_MS     100u

/* ============================================================
   LAYER 1 — RAW REGISTER ACCESS
   ============================================================ */

/**
 * Write a single byte to a register using HAL_I2C_Mem_Write.
 *
 * What HAL_I2C_Mem_Write does on the wire:
 *   START → [ADDR|W] → ACK → [REG] → ACK → [VAL] → ACK → STOP
 */
ISM330_Status_t ISM330_WriteReg(I2C_HandleTypeDef *hi2c,
                                 uint8_t reg, uint8_t val)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(
        hi2c,
        ISM330_ADDR,          /* 8-bit address (7-bit << 1) */
        (uint16_t)reg,        /* register pointer            */
        I2C_MEMADD_SIZE_8BIT, /* register address is 1 byte  */
        &val,                 /* data to send                */
        1u,                   /* number of bytes             */
        I2C_TIMEOUT_MS
    );
    return (ret == HAL_OK) ? ISM330_OK : ISM330_ERR_I2C;
}

/**
 * Read a single byte from a register using HAL_I2C_Mem_Read.
 *
 * What HAL_I2C_Mem_Read does on the wire:
 *   START → [ADDR|W] → ACK → [REG] → ACK
 *   → REPEATED START → [ADDR|R] → ACK → [DATA] → NACK → STOP
 *
 * The "repeated START" (Sr) is the key: it keeps the bus busy
 * while switching from write (sending the register pointer)
 * to read (receiving the data). HAL handles this automatically.
 */
ISM330_Status_t ISM330_ReadReg(I2C_HandleTypeDef *hi2c,
                                uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        hi2c,
        ISM330_ADDR,
        (uint16_t)reg,
        I2C_MEMADD_SIZE_8BIT,
        val,
        1u,
        I2C_TIMEOUT_MS
    );
    return (ret == HAL_OK) ? ISM330_OK : ISM330_ERR_I2C;
}

/**
 * Burst-read multiple consecutive registers.
 *
 * The ISM330DHCX automatically increments the internal register
 * pointer after each byte when IF_INC (CTRL3_C bit 2) is set.
 * ISM330_Init() enables this, so you can read all 14 output bytes
 * in a single I2C transaction instead of 14 separate reads.
 *
 * On the wire:
 *   START → [ADDR|W] → ACK → [START_REG] → ACK
 *   → Sr → [ADDR|R] → ACK → [D0] ACK [D1] ACK ... [Dn-1] NACK → STOP
 */
ISM330_Status_t ISM330_ReadRegs(I2C_HandleTypeDef *hi2c,
                                 uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        hi2c,
        ISM330_ADDR,
        (uint16_t)reg,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        len,
        I2C_TIMEOUT_MS
    );
    return (ret == HAL_OK) ? ISM330_OK : ISM330_ERR_I2C;
}

/**
 * Read-modify-write a register.
 *
 * Reads the current value, clears the bits in 'mask', sets
 * the new bits from 'val', then writes back.
 * Use this to change one setting without disturbing others.
 *
 * Example — enable BDU bit only:
 *   ISM330_ModifyReg(hi2c, ISM330_REG_CTRL3_C, ISM330_CTRL3_BDU, ISM330_CTRL3_BDU);
 */
ISM330_Status_t ISM330_ModifyReg(I2C_HandleTypeDef *hi2c,
                                  uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t current = 0u;
    ISM330_Status_t ret = ISM330_ReadReg(hi2c, reg, &current);
    if (ret != ISM330_OK) { return ret; }

    current = (uint8_t)((current & ~mask) | (val & mask));
    return ISM330_WriteReg(hi2c, reg, current);
}

/* ============================================================
   LAYER 2 — HIGH-LEVEL SENSOR API
   ============================================================ */

/**
 * Initialise the sensor.
 *
 * Step-by-step what happens:
 *   1. Read WHO_AM_I (0x0F) — must return 0x6B to confirm the
 *      chip is present and the I2C address is correct.
 *   2. Write CTRL3_C:
 *        BDU (bit 6) = 1 — Block Data Update.
 *          Without BDU, the MCU could read the low byte of a sample
 *          and then the sensor updates mid-read, giving you the high
 *          byte of the NEXT sample. BDU freezes the output registers
 *          until both bytes have been read.
 *        IF_INC (bit 2) = 1 — Auto-Increment.
 *          Allows burst reads: reading N bytes starting at register R
 *          automatically gives R, R+1, R+2 ... without re-sending
 *          the register address for each byte.
 *   3. Write CTRL1_XL (accelerometer):
 *        ODR = 416 Hz — sample rate.
 *        FS  = ±2g    — full-scale range.
 *   4. Write CTRL2_G (gyroscope):
 *        ODR = 416 Hz
 *        FS  = 250 dps
 */
ISM330_Status_t ISM330_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0u;
    ISM330_Status_t ret;

    /* Step 1: WHO_AM_I check */
    ret = ISM330_ReadReg(hi2c, ISM330_REG_WHO_AM_I, &who);
    if (ret != ISM330_OK)              { return ISM330_ERR_I2C; }
    if (who != ISM330_WHO_AM_I_VAL)    { return ISM330_ERR_ID;  }

    /* Step 2: General config — BDU + auto-increment */
    ret = ISM330_WriteReg(hi2c, ISM330_REG_CTRL3_C, ISM330_CTRL3_DEFAULT);
    if (ret != ISM330_OK) { return ret; }
    HAL_Delay(10u);   /* brief settle after config */

    /* Step 3: Accelerometer — 416 Hz, ±2g */
    ret = ISM330_WriteReg(hi2c, ISM330_REG_CTRL1_XL,
                          ISM330_XL_ODR_416Hz | ISM330_XL_FS_2G);
    if (ret != ISM330_OK) { return ret; }

    /* Step 4: Gyroscope — 416 Hz, 250 dps */
    ret = ISM330_WriteReg(hi2c, ISM330_REG_CTRL2_G,
                          ISM330_G_ODR_416Hz | ISM330_G_FS_250dps);
    if (ret != ISM330_OK) { return ret; }

    return ISM330_OK;
}

/**
 * Software reset.
 * Sets bit 0 (SW_RESET) of CTRL3_C, then waits for the bit to
 * clear (the chip clears it automatically when reset is done).
 */
ISM330_Status_t ISM330_SoftReset(I2C_HandleTypeDef *hi2c)
{
    ISM330_Status_t ret = ISM330_WriteReg(hi2c, ISM330_REG_CTRL3_C,
                                           ISM330_CTRL3_SW_RESET);
    if (ret != ISM330_OK) { return ret; }

    /* Wait for reset to complete (chip clears the bit) */
    uint8_t ctrl = ISM330_CTRL3_SW_RESET;
    uint32_t deadline = HAL_GetTick() + 100u;
    while ((ctrl & ISM330_CTRL3_SW_RESET) && HAL_GetTick() < deadline) {
        ISM330_ReadReg(hi2c, ISM330_REG_CTRL3_C, &ctrl);
        HAL_Delay(1u);
    }
    HAL_Delay(10u);  /* extra settle */
    return ISM330_OK;
}

/**
 * Burst-read all 14 output bytes (temp + gyro + accel) in one transaction.
 *
 * Memory map starting at OUT_TEMP_L (0x20):
 *   buf[0]  = 0x20 OUT_TEMP_L    \
 *   buf[1]  = 0x21 OUT_TEMP_H    / → raw->temp
 *   buf[2]  = 0x22 OUTX_L_G     \
 *   buf[3]  = 0x23 OUTX_H_G     / → raw->gx
 *   buf[4]  = 0x24 OUTY_L_G     \
 *   buf[5]  = 0x25 OUTY_H_G     / → raw->gy
 *   buf[6]  = 0x26 OUTZ_L_G     \
 *   buf[7]  = 0x27 OUTZ_H_G     / → raw->gz
 *   buf[8]  = 0x28 OUTX_L_A     \
 *   buf[9]  = 0x29 OUTX_H_A     / → raw->ax
 *   buf[10] = 0x2A OUTY_L_A     \
 *   buf[11] = 0x2B OUTY_H_A     / → raw->ay
 *   buf[12] = 0x2C OUTZ_L_A     \
 *   buf[13] = 0x2D OUTZ_H_A     / → raw->az
 *
 * Each value is little-endian: LOW byte first, HIGH byte second.
 * Combine as: (int16_t)((H << 8) | L)
 */
ISM330_Status_t ISM330_ReadRaw(I2C_HandleTypeDef *hi2c, ISM330_Raw_t *raw)
{
    uint8_t buf[14];

    ISM330_Status_t ret = ISM330_ReadRegs(hi2c, ISM330_REG_OUT_TEMP_L, buf, 14u);
    if (ret != ISM330_OK) { return ret; }

    /* Reconstruct signed 16-bit values from little-endian bytes */
    raw->temp = (int16_t)((uint16_t)buf[1]  << 8 | buf[0]);
    raw->gx   = (int16_t)((uint16_t)buf[3]  << 8 | buf[2]);
    raw->gy   = (int16_t)((uint16_t)buf[5]  << 8 | buf[4]);
    raw->gz   = (int16_t)((uint16_t)buf[7]  << 8 | buf[6]);
    raw->ax   = (int16_t)((uint16_t)buf[9]  << 8 | buf[8]);
    raw->ay   = (int16_t)((uint16_t)buf[11] << 8 | buf[10]);
    raw->az   = (int16_t)((uint16_t)buf[13] << 8 | buf[12]);

    return ISM330_OK;
}

/**
 * Wait for fresh data, then read.
 *
 * STATUS_REG (0x1E):
 *   bit 0 (XLDA) = 1 when new accel sample is ready
 *   bit 1 (GDA)  = 1 when new gyro  sample is ready
 *
 * Both bits are cleared automatically when you read the output
 * registers. Polling here ensures you never read a stale sample.
 *
 * With 416 Hz ODR, new data arrives every ~2.4 ms.
 * The 100 ms timeout is far more than enough.
 */
ISM330_Status_t ISM330_WaitAndRead(I2C_HandleTypeDef *hi2c, ISM330_Raw_t *raw)
{
    uint8_t status = 0u;
    uint32_t deadline = HAL_GetTick() + 100u;

    do {
        ISM330_Status_t ret = ISM330_ReadReg(hi2c, ISM330_REG_STATUS, &status);
        if (ret != ISM330_OK) { return ISM330_ERR_I2C; }
        if (HAL_GetTick() >= deadline) { return ISM330_ERR_TOUT; }
    } while ((status & ISM330_STATUS_BOTH_READY) != ISM330_STATUS_BOTH_READY);

    return ISM330_ReadRaw(hi2c, raw);
}

/**
 * Convert raw integer values to physical units.
 *
 * The sensor ADC produces signed 16-bit integers.
 * To get physical units, multiply by the sensitivity factor.
 *
 * Accel:
 *   With ±2g FS, the full 16-bit range (−32768 to +32767) maps
 *   to −2g to +2g. Sensitivity = 2g / 32768 ≈ 0.000061 g/LSB.
 *
 * Gyro:
 *   With 250 dps FS, sensitivity = 250 / 32768 ≈ 0.00875 dps/LSB.
 *
 * Temperature:
 *   From datasheet: T(°C) = raw_value / 256 + 25
 *   At room temperature (25°C), raw ≈ 0.
 */
void ISM330_ToPhysical(ISM330_Raw_t *raw, ISM330_Phys_t *phys)
{
    phys->ax   = (float)raw->ax * SENS_ACCEL_2G;
    phys->ay   = (float)raw->ay * SENS_ACCEL_2G;
    phys->az   = (float)raw->az * SENS_ACCEL_2G;

    phys->gx   = (float)raw->gx * SENS_GYRO_250DPS;
    phys->gy   = (float)raw->gy * SENS_GYRO_250DPS;
    phys->gz   = (float)raw->gz * SENS_GYRO_250DPS;

    phys->temp = (float)raw->temp / 256.0f + 25.0f;
}

/* ============================================================
   LAYER 3 — SOFTWARE FILTERING
   ============================================================ */

/**
 * Initialise the low-pass filter state.
 *
 * Sets all internal state to zero and stores the alpha value.
 * Must be called once before ISM330_LPF_Update().
 */
void ISM330_LPF_Init(ISM330_Filtered_t *f, float alpha)
{
    f->alpha = alpha;
    f->ax = 0.0f; f->ay = 0.0f; f->az = 0.0f;
    f->gx = 0.0f; f->gy = 0.0f; f->gz = 0.0f;
}

/**
 * Exponential Moving Average (EMA) low-pass filter.
 *
 * Formula: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 *
 * WHY THIS WORKS:
 *   A micromouse experiences vibration from motors, surface bumps,
 *   and encoder clicks. These create high-frequency noise in the IMU
 *   output. The low-pass filter attenuates (reduces) high frequencies
 *   while letting slow, meaningful changes through.
 *
 * CHOOSING ALPHA:
 *   alpha = 0.05  — heavy smoothing, slow to react (good for heading hold)
 *   alpha = 0.15  — balanced (recommended starting point)
 *   alpha = 0.30  — light smoothing, faster response
 *   alpha = 1.00  — no filtering
 *
 * LIMITATION:
 *   EMA adds lag. For angle integration (heading), prefer the
 *   Complementary Filter or a Kalman Filter which fuse accel + gyro.
 *
 * Call once per sensor read cycle (e.g. every 2.4 ms at 416 Hz).
 */
void ISM330_LPF_Update(ISM330_Filtered_t *f, ISM330_Phys_t *phys)
{
    float inv = 1.0f - f->alpha;

    f->ax = f->alpha * phys->ax + inv * f->ax;
    f->ay = f->alpha * phys->ay + inv * f->ay;
    f->az = f->alpha * phys->az + inv * f->az;

    f->gx = f->alpha * phys->gx + inv * f->gx;
    f->gy = f->alpha * phys->gy + inv * f->gy;
    f->gz = f->alpha * phys->gz + inv * f->gz;
}

/* ============================================================
   LAYER 4 — UTILITY
   ============================================================ */

const char *ISM330_StatusStr(ISM330_Status_t s)
{
    switch (s) {
        case ISM330_OK:       return "OK";
        case ISM330_ERR_I2C:  return "I2C error";
        case ISM330_ERR_ID:   return "Wrong WHO_AM_I (check wiring/address)";
        case ISM330_ERR_TOUT: return "Data-ready timeout";
        default:              return "Unknown error";
    }
}
