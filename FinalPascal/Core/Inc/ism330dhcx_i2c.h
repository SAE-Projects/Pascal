#ifndef ISM330DHCX_I2C_H
#define ISM330DHCX_I2C_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
   I2C ADDRESS
   ================================================================ */
#define ISM330_ADDR             (0x6B << 1)
#define ISM330_WHO_AM_I_VAL     0x6Bu

/* ================================================================
   REGISTER ADDRESSES
   ================================================================ */
#define ISM330_REG_WHO_AM_I     0x0F
#define ISM330_REG_CTRL1_XL     0x10
#define ISM330_REG_CTRL2_G      0x11
#define ISM330_REG_CTRL3_C      0x12
#define ISM330_REG_CTRL4_C      0x13
#define ISM330_CTRL4_LPF1_EN    (1u << 1)
#define ISM330_REG_CTRL6_C      0x15
#define ISM330_CTRL6_FTYPE_33HZ 0x03u
#define ISM330_REG_STATUS       0x1E
#define ISM330_REG_OUT_TEMP_L   0x20
#define ISM330_REG_OUTX_L_G     0x22
#define ISM330_REG_OUTX_L_A     0x28

/* ================================================================
   CTRL1_XL (Accel) & CTRL2_G (Gyro) CONFIG
   ================================================================ */
#define ISM330_XL_ODR_833Hz     0x70u
#define ISM330_XL_FS_2G         0x00u

#define ISM330_G_ODR_833Hz      0x70u
#define ISM330_G_FS_1000dps     0x08u

/* ================================================================
   CTRL3_C & STATUS MASKS
   ================================================================ */
#define ISM330_CTRL3_BDU        (1u << 6)
#define ISM330_CTRL3_IF_INC     (1u << 2)
#define ISM330_CTRL3_SW_RESET   (1u << 0)
#define ISM330_CTRL3_DEFAULT    (ISM330_CTRL3_BDU | ISM330_CTRL3_IF_INC)

#define ISM330_STATUS_XLDA      (1u << 0)
#define ISM330_STATUS_GDA       (1u << 1)
#define ISM330_STATUS_BOTH      (ISM330_STATUS_XLDA | ISM330_STATUS_GDA)

/* ================================================================
   DATA STRUCTURES
   ================================================================ */

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
} ISM330_Raw_t;

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} ISM330_Phys_t;

/* ================================================================
   STATUS / ERROR CODES
   ================================================================ */
typedef enum {
    ISM330_OK       = 0,
    ISM330_ERR_I2C  = 1,
    ISM330_ERR_ID   = 2,
    ISM330_ERR_TOUT = 3,
} ISM330_Status_t;

/* ================================================================
   FUNCTION PROTOTYPES
   ================================================================ */
ISM330_Status_t ISM330_WriteReg  (I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val);
ISM330_Status_t ISM330_ReadReg   (I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *val);
ISM330_Status_t ISM330_ReadRegs  (I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint16_t len);
ISM330_Status_t ISM330_ModifyReg (I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t mask, uint8_t val);

ISM330_Status_t ISM330_Init        (I2C_HandleTypeDef *hi2c);
ISM330_Status_t ISM330_SoftReset   (I2C_HandleTypeDef *hi2c);
ISM330_Status_t ISM330_ReadRaw     (I2C_HandleTypeDef *hi2c, ISM330_Raw_t *raw);
ISM330_Status_t ISM330_WaitAndRead (I2C_HandleTypeDef *hi2c, ISM330_Raw_t *raw);
void            ISM330_ToPhysical  (ISM330_Raw_t *raw, ISM330_Phys_t *phys);

const char *ISM330_StatusStr (ISM330_Status_t s);

#ifdef __cplusplus
}
#endif

#endif /* ISM330DHCX_I2C_H */
