#ifndef ISM330DHCX_I2C_H
#define ISM330DHCX_I2C_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
   I2C ADDRESS
   Your breakout board has SA0 tied to GND → address is 0x6A.
   HAL needs the 7-bit address shifted left by 1 (8-bit format).
   If the I2C scanner finds 0x6B instead, change 0x6A to 0x6B.
   ================================================================ */
#define ISM330_ADDR             (0x6B << 1)

/* ================================================================
   WHO_AM_I — read register 0x0F, expect this value back
   If you get a different value the I2C address is wrong.
   ================================================================ */
#define ISM330_WHO_AM_I_VAL     0x6Bu

/* ================================================================
   REGISTER ADDRESSES
   ================================================================ */
#define ISM330_REG_WHO_AM_I     0x0F
#define ISM330_REG_CTRL1_XL     0x10   /* accel: ODR + full-scale      */
#define ISM330_REG_CTRL2_G      0x11   /* gyro:  ODR + full-scale      */
#define ISM330_REG_CTRL3_C      0x12   /* BDU, auto-increment, SW reset*/
#define ISM330_REG_STATUS       0x1E   /* data-ready flags             */
#define ISM330_REG_OUT_TEMP_L   0x20   /* start of 14-byte output block*/
#define ISM330_REG_OUTX_L_G     0x22   /* gyro  X low byte             */
#define ISM330_REG_OUTX_L_A     0x28   /* accel X low byte             */

/* ================================================================
   CTRL1_XL — accelerometer output data rate (ODR) and full-scale
   Write to register 0x10.
   Bits [7:4] = ODR   Bits [3:2] = FS
   ================================================================ */
#define ISM330_XL_ODR_OFF       0x00u  /* power down                  */
#define ISM330_XL_ODR_12Hz5     0x10u
#define ISM330_XL_ODR_26Hz      0x20u
#define ISM330_XL_ODR_52Hz      0x30u
#define ISM330_XL_ODR_104Hz     0x40u
#define ISM330_XL_ODR_208Hz     0x50u
#define ISM330_XL_ODR_416Hz     0x60u  /* used by default             */
#define ISM330_XL_ODR_833Hz     0x70u

#define ISM330_XL_FS_2G         0x00u  /* ±2g  — most sensitive       */
#define ISM330_XL_FS_4G         0x08u
#define ISM330_XL_FS_8G         0x0Cu
#define ISM330_XL_FS_16G        0x04u

/* ================================================================
   CTRL2_G — gyroscope ODR and full-scale
   Write to register 0x11.
   Bits [7:4] = ODR   Bits [3:1] = FS
   ================================================================ */
#define ISM330_G_ODR_OFF        0x00u  /* power down                  */
#define ISM330_G_ODR_12Hz5      0x10u
#define ISM330_G_ODR_26Hz       0x20u
#define ISM330_G_ODR_52Hz       0x30u
#define ISM330_G_ODR_104Hz      0x40u
#define ISM330_G_ODR_208Hz      0x50u
#define ISM330_G_ODR_416Hz      0x60u  /* used by default             */
#define ISM330_G_ODR_833Hz      0x70u

#define ISM330_G_FS_125dps      0x02u
#define ISM330_G_FS_250dps      0x00u  /* used by default             */
#define ISM330_G_FS_500dps      0x04u
#define ISM330_G_FS_1000dps     0x08u
#define ISM330_G_FS_2000dps     0x0Cu

/* ================================================================
   CTRL3_C — general control
   Write to register 0x12.
   BDU    (bit 6) = 1 → freeze output registers until both bytes read
   IF_INC (bit 2) = 1 → auto-increment address for burst reads
   ================================================================ */
#define ISM330_CTRL3_BDU        (1u << 6)
#define ISM330_CTRL3_IF_INC     (1u << 2)
#define ISM330_CTRL3_SW_RESET   (1u << 0)
#define ISM330_CTRL3_DEFAULT    (ISM330_CTRL3_BDU | ISM330_CTRL3_IF_INC)  /* 0x44 */

/* ================================================================
   STATUS_REG — data-ready flags
   Read register 0x1E before reading output registers.
   Bit 0 (XLDA) = 1 → new accel sample ready
   Bit 1 (GDA)  = 1 → new gyro  sample ready
   ================================================================ */
#define ISM330_STATUS_XLDA      (1u << 0)
#define ISM330_STATUS_GDA       (1u << 1)
#define ISM330_STATUS_BOTH      (ISM330_STATUS_XLDA | ISM330_STATUS_GDA)

/* ================================================================
   DATA STRUCTURES
   ================================================================ */

/* Raw 16-bit values straight from the sensor registers.
   These are signed two's-complement integers.
   Call ISM330_ToPhysical() to convert them to real units. */
typedef struct {
    int16_t ax, ay, az;   /* raw accelerometer X Y Z */
    int16_t gx, gy, gz;   /* raw gyroscope    X Y Z */
    int16_t temp;          /* raw temperature        */
} ISM330_Raw_t;

/* Physical-unit values after sensitivity conversion.
   These are the values you actually use in your code. */
typedef struct {
    float ax, ay, az;     /* acceleration in g  (1g = 9.81 m/s²)  */
    float gx, gy, gz;     /* angular rate in degrees per second    */
} ISM330_Phys_t;

/* Software low-pass filter state.
   Holds the previous filtered value and the alpha coefficient.
   alpha: 0.05 = heavy smooth | 0.15 = balanced | 1.0 = no filter */
typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float alpha;
} ISM330_Filtered_t;

/* ================================================================
   STATUS / ERROR CODES
   ================================================================ */
typedef enum {
    ISM330_OK       = 0,  /* success                              */
    ISM330_ERR_I2C  = 1,  /* I2C bus error — check wiring         */
    ISM330_ERR_ID   = 2,  /* wrong WHO_AM_I — check I2C address   */
    ISM330_ERR_TOUT = 3,  /* data-ready timeout                   */
} ISM330_Status_t;

/* ================================================================
   FUNCTION PROTOTYPES
   ================================================================ */

/* --- Register access (low level) --- */
ISM330_Status_t ISM330_WriteReg  (I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val);
ISM330_Status_t ISM330_ReadReg   (I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *val);
ISM330_Status_t ISM330_ReadRegs  (I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint16_t len);
ISM330_Status_t ISM330_ModifyReg (I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t mask, uint8_t val);

/* --- Sensor control (high level) --- */
ISM330_Status_t ISM330_Init        (I2C_HandleTypeDef *hi2c);
ISM330_Status_t ISM330_SoftReset   (I2C_HandleTypeDef *hi2c);
ISM330_Status_t ISM330_ReadRaw     (I2C_HandleTypeDef *hi2c, ISM330_Raw_t *raw);
ISM330_Status_t ISM330_WaitAndRead (I2C_HandleTypeDef *hi2c, ISM330_Raw_t *raw);
void            ISM330_ToPhysical  (ISM330_Raw_t *raw, ISM330_Phys_t *phys);

/* --- Software filter --- */
void ISM330_LPF_Init   (ISM330_Filtered_t *f, float alpha);
void ISM330_LPF_Update (ISM330_Filtered_t *f, ISM330_Phys_t *phys);

/* --- Utility --- */
const char *ISM330_StatusStr (ISM330_Status_t s);

#ifdef __cplusplus
}
#endif

#endif /* ISM330DHCX_I2C_H */
