#include "ism330dhcx_i2c.h"


/* External reference to our high-res timer from main.c */
extern uint32_t DWT_GetMicros(void);

/* Helper function for microsecond-level blocking delays */
void DWT_Delay(uint32_t us)
{
    uint32_t start = DWT_GetMicros();
    while ((DWT_GetMicros() - start) < us) {
        /* Wait */
    }
}
/* ================================================================
   SENSITIVITY CONSTANTS
   Match these to whatever FS you set in ISM330_Init().

   Accel (g per LSB):   ±2g=0.000061  ±4g=0.000122  ±8g=0.000244  ±16g=0.000488
   Gyro  (dps per LSB): 125=0.004375  250=0.008750   500=0.017500  1000=0.035000
   ================================================================ */
#define SENS_ACCEL   0.000061f   /* ±2g   */
#define SENS_GYRO    0.035000f   /* 500dps */
#define I2C_TIMEOUT  5u        /* ms     */

/* ================================================================
   LAYER 1 — REGISTER ACCESS
   ================================================================ */

/* Write one byte to a register.
   On the wire: START → ADDR+W → REG → VAL → STOP */
ISM330_Status_t ISM330_WriteReg(I2C_HandleTypeDef *hi2c,
                                 uint8_t reg, uint8_t val)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(
        hi2c, ISM330_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
        &val, 1u, I2C_TIMEOUT);
    return (ret == HAL_OK) ? ISM330_OK : ISM330_ERR_I2C;
}

/* Read one byte from a register.
   On the wire: START → ADDR+W → REG → REPEATED START → ADDR+R → DATA → STOP */
ISM330_Status_t ISM330_ReadReg(I2C_HandleTypeDef *hi2c,
                                uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        hi2c, ISM330_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
        val, 1u, I2C_TIMEOUT);
    return (ret == HAL_OK) ? ISM330_OK : ISM330_ERR_I2C;
}

/* Burst-read len consecutive registers starting at reg.
   Works because IF_INC (CTRL3_C bit 2) is set in ISM330_Init.
   The sensor auto-increments the register pointer after each byte. */
ISM330_Status_t ISM330_ReadRegs(I2C_HandleTypeDef *hi2c,
                                 uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        hi2c, ISM330_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
        buf, len, I2C_TIMEOUT);
    return (ret == HAL_OK) ? ISM330_OK : ISM330_ERR_I2C;
}

/* Read-modify-write: change only the bits set in mask, leave others alone. */
ISM330_Status_t ISM330_ModifyReg(I2C_HandleTypeDef *hi2c,
                                  uint8_t reg, uint8_t mask, uint8_t val)
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

/* Initialise the sensor — call once at startup.
   1. Read WHO_AM_I → must be 0x6B (confirms chip present + address correct)
   2. CTRL3_C = 0x44 → enables BDU (no mixed samples) + IF_INC (burst reads)
   3. CTRL1_XL      → accel 416 Hz, ±2g
   4. CTRL2_G       → gyro  416 Hz, 250 dps */

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
    ret = ISM330_WriteReg(hi2c, ISM330_REG_CTRL1_XL,
                          ISM330_XL_ODR_833Hz | ISM330_XL_FS_2G);
    if (ret != ISM330_OK) { return ret; }

    /* 4. Configure Gyroscope (833Hz, 250dps) */
    ret = ISM330_WriteReg(hi2c, ISM330_REG_CTRL2_G,
                          ISM330_G_ODR_833Hz | ISM330_G_FS_1000dps);
    if (ret != ISM330_OK) { return ret; }

    /* --- THE PRO UPGRADE: HARDWARE FILTERS --- */

    /* 5. Enable the Gyroscope Hardware Low-Pass Filter (LPF1) */
    ret = ISM330_WriteReg(hi2c, ISM330_REG_CTRL4_C, ISM330_CTRL4_LPF1_EN);
    if (ret != ISM330_OK) { return ret; }

    /* 6. Set the filter cutoff to ~33Hz to block high-frequency motor vibration */
    ret = ISM330_WriteReg(hi2c, ISM330_REG_CTRL6_C, ISM330_CTRL6_FTYPE_33HZ);
    if (ret != ISM330_OK) { return ret; }

    return ISM330_OK;
}

/* Software reset — clears all registers back to power-on defaults.
   Useful if MCU resets but sensor keeps power (avoids stale config).
   The chip clears SW_RESET bit automatically when done. */

ISM330_Status_t ISM330_SoftReset(I2C_HandleTypeDef *hi2c)
{
    ISM330_Status_t ret = ISM330_WriteReg(hi2c, ISM330_REG_CTRL3_C,
                                           ISM330_CTRL3_SW_RESET);
    if (ret != ISM330_OK) { return ret; }

    uint8_t ctrl = ISM330_CTRL3_SW_RESET;

    /* 100ms timeout = 100,000 microseconds */
    uint32_t deadline = DWT_GetMicros() + 100000u;

    while ((ctrl & ISM330_CTRL3_SW_RESET) && (DWT_GetMicros() < deadline)) {
        ISM330_ReadReg(hi2c, ISM330_REG_CTRL3_C, &ctrl);
        DWT_Delay(1000u); /* 1ms wait using microsecond timer */
    }
    HAL_Delay(10u);
    return ISM330_OK;
}
/* Burst-read all 14 output bytes in one I2C transaction.
   Registers 0x20–0x2D (auto-incremented):
     buf[0-1]   → temp   (little-endian: L byte first, H byte second)
     buf[2-3]   → gyro X
     buf[4-5]   → gyro Y
     buf[6-7]   → gyro Z
     buf[8-9]   → accel X
     buf[10-11] → accel Y
     buf[12-13] → accel Z
   Reconstruct each int16: (int16_t)((H << 8) | L) */
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

/* Poll STATUS_REG until both accel (XLDA) and gyro (GDA) data are ready,
   then burst-read. At 416 Hz ODR new data arrives every ~2.4 ms.
   Timeout after 100 ms — returns ISM330_ERR_TOUT if sensor stops responding. */
ISM330_Status_t ISM330_WaitAndRead(I2C_HandleTypeDef *hi2c, ISM330_Raw_t *raw)
{
    uint8_t  status   = 0u;
    uint32_t deadline = DWT_GetMicros() + 10000u; /* 10ms timeout */

    do {
        ISM330_Status_t ret = ISM330_ReadReg(hi2c, ISM330_REG_STATUS, &status);
        if (ret != ISM330_OK)            { return ISM330_ERR_I2C; }
        if (DWT_GetMicros() >= deadline) { return ISM330_ERR_TOUT; }

        /* If data is NOT ready, sleep for 250 microseconds to keep the I2C bus quiet */
        if ((status & ISM330_STATUS_BOTH) != ISM330_STATUS_BOTH)
        {
            DWT_Delay(250u);
        }

    } while ((status & ISM330_STATUS_BOTH) != ISM330_STATUS_BOTH);

    return ISM330_ReadRaw(hi2c, raw);
}

/* Convert raw int16 values to physical units.
   Accel:  raw × 0.000061  → g      (for ±2g FS)
   Gyro:   raw × 0.008750  → dps    (for 250 dps FS)
   Temp:   raw / 256 + 25  → °C     (from datasheet formula) */
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
   LAYER 3 — SOFTWARE FILTER
   ================================================================ */

/* Initialise filter state to zero. Must be called before LPF_Update. */

void ISM330_LPF_Init(ISM330_Filtered_t *f, float alpha)
{
    f->alpha = alpha;
    f->ax = 0.0f; f->ay = 0.0f; f->az = 0.0f;
    f->gx = 0.0f; f->gy = 0.0f; f->gz = 0.0f;
}

/* Exponential moving average low-pass filter.
   Formula: filtered = alpha * new + (1 - alpha) * filtered_prev
   Call once per IMU_Read() cycle.
   alpha 0.05 = heavy smooth / 0.15 = balanced / 1.0 = no filter */
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

/* ================================================================
   LAYER 4 — UTILITY
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
