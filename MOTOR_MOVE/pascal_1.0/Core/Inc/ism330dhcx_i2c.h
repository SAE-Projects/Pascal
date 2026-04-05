/**
 * ============================================================
 *  ism330dhcx_i2c.h
 *  ISM330DHCX IMU driver — I2C interface for STM32 HAL
 *
 *  Target MCU : STM32G431CBU6  (any STM32 HAL works)
 *  Interface  : I2C (HAL_I2C_Mem_Read / HAL_I2C_Mem_Write)
 *  Author     : Your Name
 * ============================================================
 *
 *  HOW THIS FILE IS ORGANISED
 *  --------------------------
 *  1. I2C address
 *  2. Register map (every register used)
 *  3. Bit-field definitions for each config register
 *  4. Data structs  (raw int16  and  physical float)
 *  5. Status enum
 *  6. Function prototypes
 *
 *  QUICK START
 *  -----------
 *  ISM330_Init(&hi2c1);                // once, in main()
 *  ISM330_WaitAndRead(&hi2c1, &raw);   // poll loop
 *  ISM330_ToPhysical(&raw, &phys);     // convert to g / dps
 * ============================================================
 */

#ifndef ISM330DHCX_I2C_H
#define ISM330DHCX_I2C_H

#include "stm32g4xx_hal.h"   /* swap for stm32f4xx_hal.h etc. on other MCUs */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
   1.  I2C ADDRESS
   ============================================================
   The 7-bit address depends on the SA0/SDO pin:
     SA0 = GND  →  0x6A
     SA0 = VDD  →  0x6B
   HAL expects the address left-shifted by 1 (8-bit format).
   ============================================================ */
#define ISM330_I2C_ADDR_SA0_LOW    (0x6A << 1)   /* default if SA0 tied to GND */
#define ISM330_I2C_ADDR_SA0_HIGH   (0x6B << 1)

/* Change this to match your hardware wiring */
#define ISM330_ADDR                 ISM330_I2C_ADDR_SA0_LOW

/* ============================================================
   2.  REGISTER MAP
   ============================================================ */

/* --- Identification ---------------------------------------- */
#define ISM330_REG_WHO_AM_I         0x0F   /* read-only; expected value 0x6B */

/* --- Configuration registers ------------------------------ */
#define ISM330_REG_CTRL1_XL         0x10   /* accelerometer ODR + FS */
#define ISM330_REG_CTRL2_G          0x11   /* gyroscope ODR + FS     */
#define ISM330_REG_CTRL3_C          0x12   /* BDU, auto-increment, SW reset */
#define ISM330_REG_CTRL4_C          0x13   /* I2C/SPI config         */
#define ISM330_REG_CTRL5_C          0x14   /* self-test              */
#define ISM330_REG_CTRL6_C          0x15   /* gyro high-perf mode    */
#define ISM330_REG_CTRL7_G          0x16   /* gyro HPF               */
#define ISM330_REG_CTRL8_XL         0x17   /* accel LPF2 / slope     */
#define ISM330_REG_CTRL9_XL         0x18
#define ISM330_REG_CTRL10_C         0x19

/* --- Status register --------------------------------------- */
#define ISM330_REG_STATUS           0x1E

/* --- Output registers (all little-endian: L first, H second) */
#define ISM330_REG_OUT_TEMP_L       0x20   /* temperature low  */
#define ISM330_REG_OUT_TEMP_H       0x21   /* temperature high */
#define ISM330_REG_OUTX_L_G         0x22   /* gyro X low       */
#define ISM330_REG_OUTX_H_G         0x23
#define ISM330_REG_OUTY_L_G         0x24   /* gyro Y low       */
#define ISM330_REG_OUTY_H_G         0x25
#define ISM330_REG_OUTZ_L_G         0x26   /* gyro Z low       */
#define ISM330_REG_OUTZ_H_G         0x27
#define ISM330_REG_OUTX_L_A         0x28   /* accel X low      */
#define ISM330_REG_OUTX_H_A         0x29
#define ISM330_REG_OUTY_L_A         0x2A   /* accel Y low      */
#define ISM330_REG_OUTY_H_A         0x2B
#define ISM330_REG_OUTZ_L_A         0x2C   /* accel Z low      */
#define ISM330_REG_OUTZ_H_A         0x2D

/* --- FIFO registers ---------------------------------------- */
#define ISM330_REG_FIFO_CTRL1       0x07
#define ISM330_REG_FIFO_CTRL2       0x08
#define ISM330_REG_FIFO_CTRL3       0x09
#define ISM330_REG_FIFO_CTRL4       0x0A
#define ISM330_REG_FIFO_STATUS1     0x3A
#define ISM330_REG_FIFO_STATUS2     0x3B
#define ISM330_REG_FIFO_DATA_OUT_L  0x79
#define ISM330_REG_FIFO_DATA_OUT_H  0x7A

/* --- Interrupt registers ----------------------------------- */
#define ISM330_REG_INT1_CTRL        0x0D   /* INT1 signal routing */
#define ISM330_REG_INT2_CTRL        0x0E   /* INT2 signal routing */
#define ISM330_REG_MD1_CFG          0x5E   /* INT1 function routing */
#define ISM330_REG_MD2_CFG          0x5F   /* INT2 function routing */

/* ============================================================
   3.  BIT-FIELD DEFINITIONS
   ============================================================ */

/* --- WHO_AM_I fixed value ---------------------------------- */
#define ISM330_WHO_AM_I_VAL         0x6Bu

/* --- CTRL1_XL  [7:4]=ODR_XL  [3:2]=FS_XL  [1]=LPF2_XL_EN  [0]=0 */
#define ISM330_XL_ODR_OFF           0x00u  /* power-down          */
#define ISM330_XL_ODR_12Hz5         0x10u
#define ISM330_XL_ODR_26Hz          0x20u
#define ISM330_XL_ODR_52Hz          0x30u
#define ISM330_XL_ODR_104Hz         0x40u
#define ISM330_XL_ODR_208Hz         0x50u
#define ISM330_XL_ODR_416Hz         0x60u  /* recommended for robot */
#define ISM330_XL_ODR_833Hz         0x70u
#define ISM330_XL_ODR_1667Hz        0x80u
#define ISM330_XL_ODR_3333Hz        0x90u
#define ISM330_XL_ODR_6667Hz        0xA0u

#define ISM330_XL_FS_2G             0x00u  /* ±2 g  — most sensitive */
#define ISM330_XL_FS_16G            0x04u  /* ±16g                   */
#define ISM330_XL_FS_4G             0x08u
#define ISM330_XL_FS_8G             0x0Cu

/* --- CTRL2_G  [7:4]=ODR_G  [3:1]=FS_G  [0]=FS_125 */
#define ISM330_G_ODR_OFF            0x00u
#define ISM330_G_ODR_12Hz5          0x10u
#define ISM330_G_ODR_26Hz           0x20u
#define ISM330_G_ODR_52Hz           0x30u
#define ISM330_G_ODR_104Hz          0x40u
#define ISM330_G_ODR_208Hz          0x50u
#define ISM330_G_ODR_416Hz          0x60u  /* recommended for robot */
#define ISM330_G_ODR_833Hz          0x70u
#define ISM330_G_ODR_1667Hz         0x80u
#define ISM330_G_ODR_3333Hz         0x90u
#define ISM330_G_ODR_6667Hz         0xA0u

#define ISM330_G_FS_250dps          0x00u
#define ISM330_G_FS_500dps          0x04u
#define ISM330_G_FS_1000dps         0x08u
#define ISM330_G_FS_2000dps         0x0Cu
#define ISM330_G_FS_125dps          0x02u  /* set bit0=1 for 125 dps */

/* --- CTRL3_C  [7]=BOOT  [6]=BDU  [5]=H_LACTIVE  [4]=PP_OD
               [3]=SIM    [2]=IF_INC  [1]=0  [0]=SW_RESET     */
#define ISM330_CTRL3_BDU            (1u << 6)  /* block data update   */
#define ISM330_CTRL3_IF_INC         (1u << 2)  /* auto-increment reg  */
#define ISM330_CTRL3_SW_RESET       (1u << 0)  /* software reset      */

/* Recommended value for normal operation */
#define ISM330_CTRL3_DEFAULT        (ISM330_CTRL3_BDU | ISM330_CTRL3_IF_INC)  /* 0x44 */

/* --- STATUS_REG  [2]=TDA  [1]=GDA  [0]=XLDA */
#define ISM330_STATUS_XLDA          (1u << 0)  /* accel data ready */
#define ISM330_STATUS_GDA           (1u << 1)  /* gyro  data ready */
#define ISM330_STATUS_TDA           (1u << 2)  /* temp  data ready */
#define ISM330_STATUS_BOTH_READY    (ISM330_STATUS_XLDA | ISM330_STATUS_GDA)

/* --- INT1_CTRL bits (route data-ready to INT1 pin) */
#define ISM330_INT1_DRDY_G          (1u << 1)  /* gyro  data-ready on INT1 */
#define ISM330_INT1_DRDY_XL         (1u << 0)  /* accel data-ready on INT1 */

/* ============================================================
   4.  DATA STRUCTURES
   ============================================================ */

/**
 * Raw 16-bit output from the sensor registers.
 * Values are signed two's-complement integers.
 * Do not use these directly — call ISM330_ToPhysical().
 */
typedef struct {
    int16_t ax;    /*!< raw accelerometer X  */
    int16_t ay;    /*!< raw accelerometer Y  */
    int16_t az;    /*!< raw accelerometer Z  */
    int16_t gx;    /*!< raw gyroscope X      */
    int16_t gy;    /*!< raw gyroscope Y      */
    int16_t gz;    /*!< raw gyroscope Z      */
    int16_t temp;  /*!< raw temperature      */
} ISM330_Raw_t;

/**
 * Physical-unit output after sensitivity conversion.
 * These are the values you use in your application.
 */
typedef struct {
    float ax;    /*!< accelerometer X in g (1g = 9.81 m/s²) */
    float ay;    /*!< accelerometer Y in g                   */
    float az;    /*!< accelerometer Z in g                   */
    float gx;    /*!< gyroscope X in degrees/second          */
    float gy;    /*!< gyroscope Y in degrees/second          */
    float gz;    /*!< gyroscope Z in degrees/second          */
    float temp;  /*!< temperature in °C                      */
} ISM330_Phys_t;

/**
 * Filtered output from the software low-pass filter.
 * Updated by ISM330_LPF_Update().
 */
typedef struct {
    float ax, ay, az;   /*!< filtered accel in g   */
    float gx, gy, gz;   /*!< filtered gyro in dps  */
    float alpha;         /*!< filter coefficient 0–1 (lower = smoother) */
} ISM330_Filtered_t;

/* ============================================================
   5.  STATUS / ERROR CODES
   ============================================================ */
typedef enum {
    ISM330_OK        = 0,  /*!< success                          */
    ISM330_ERR_I2C   = 1,  /*!< I2C HAL error                    */
    ISM330_ERR_ID    = 2,  /*!< WHO_AM_I mismatch                */
    ISM330_ERR_TOUT  = 3,  /*!< data-ready timeout               */
} ISM330_Status_t;

/* ============================================================
   6.  FUNCTION PROTOTYPES
   ============================================================

   LAYER 1 — raw register access (use these to build custom configs)
   ----------------------------------------------------------------- */

/**
 * @brief  Write one byte to a register.
 * @param  hi2c   Pointer to HAL I2C handle (e.g. &hi2c1)
 * @param  reg    Register address (use ISM330_REG_* constants)
 * @param  val    Byte to write
 * @retval ISM330_OK or ISM330_ERR_I2C
 */
ISM330_Status_t ISM330_WriteReg(I2C_HandleTypeDef *hi2c,
                                 uint8_t reg, uint8_t val);

/**
 * @brief  Read one byte from a register.
 * @param  hi2c   I2C handle
 * @param  reg    Register address
 * @param  val    Pointer to store the result
 * @retval ISM330_OK or ISM330_ERR_I2C
 */
ISM330_Status_t ISM330_ReadReg(I2C_HandleTypeDef *hi2c,
                                uint8_t reg, uint8_t *val);

/**
 * @brief  Burst-read multiple consecutive registers.
 *         Requires IF_INC bit set in CTRL3_C (set by ISM330_Init).
 * @param  hi2c   I2C handle
 * @param  reg    Starting register address
 * @param  buf    Buffer to store bytes
 * @param  len    Number of bytes to read
 * @retval ISM330_OK or ISM330_ERR_I2C
 */
ISM330_Status_t ISM330_ReadRegs(I2C_HandleTypeDef *hi2c,
                                 uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief  Read-modify-write: change specific bits without altering others.
 * @param  hi2c   I2C handle
 * @param  reg    Register address
 * @param  mask   Bit mask of bits to modify (1 = change, 0 = keep)
 * @param  val    New values for the masked bits
 * @retval ISM330_OK or ISM330_ERR_I2C
 */
ISM330_Status_t ISM330_ModifyReg(I2C_HandleTypeDef *hi2c,
                                  uint8_t reg, uint8_t mask, uint8_t val);

/* LAYER 2 — high-level sensor API
   --------------------------------- */

/**
 * @brief  Initialise the ISM330DHCX.
 *         Verifies WHO_AM_I, sets BDU + auto-increment,
 *         configures accel at 416 Hz ±2g and gyro at 416 Hz 250 dps.
 * @param  hi2c   I2C handle
 * @retval ISM330_OK, ISM330_ERR_I2C, or ISM330_ERR_ID
 */
ISM330_Status_t ISM330_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Soft-reset the device (clears all registers to default).
 *         Call before ISM330_Init() if the MCU restarts without power cycle.
 * @param  hi2c   I2C handle
 * @retval ISM330_OK or ISM330_ERR_I2C
 */
ISM330_Status_t ISM330_SoftReset(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Burst-read all sensor registers into an ISM330_Raw_t struct.
 *         Reads 14 bytes starting at OUT_TEMP_L (0x20).
 * @param  hi2c   I2C handle
 * @param  raw    Pointer to struct to fill
 * @retval ISM330_OK or ISM330_ERR_I2C
 */
ISM330_Status_t ISM330_ReadRaw(I2C_HandleTypeDef *hi2c, ISM330_Raw_t *raw);

/**
 * @brief  Poll STATUS_REG until both accel and gyro data are ready,
 *         then burst-read all sensor data.
 *         Has a 100 ms timeout — returns ISM330_ERR_TOUT if it expires.
 * @param  hi2c   I2C handle
 * @param  raw    Pointer to struct to fill
 * @retval ISM330_OK, ISM330_ERR_I2C, or ISM330_ERR_TOUT
 */
ISM330_Status_t ISM330_WaitAndRead(I2C_HandleTypeDef *hi2c, ISM330_Raw_t *raw);

/**
 * @brief  Convert raw int16 values to physical units.
 *         Uses sensitivities matching the default Init config (±2g, 250dps).
 *         If you change ODR/FS, update SENS_ACCEL / SENS_GYRO in the .c file.
 * @param  raw    Pointer to raw data (input)
 * @param  phys   Pointer to physical-unit struct (output)
 */
void ISM330_ToPhysical(ISM330_Raw_t *raw, ISM330_Phys_t *phys);

/* LAYER 3 — software filtering
   ------------------------------ */

/**
 * @brief  Initialise a low-pass filter state struct.
 *         Must be called once before ISM330_LPF_Update().
 * @param  f      Pointer to filter struct
 * @param  alpha  Smoothing factor 0.0–1.0
 *                0.05 = heavy smoothing (slow response)
 *                0.20 = moderate (good for micromouse heading)
 *                0.50 = light smoothing
 *                1.00 = no filtering (pass-through)
 */
void ISM330_LPF_Init(ISM330_Filtered_t *f, float alpha);

/**
 * @brief  Apply exponential moving average (low-pass) filter.
 *         Call this every time you read new physical data.
 *         Formula: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 * @param  f      Filter state (updated in-place)
 * @param  phys   Latest physical-unit reading (input)
 */
void ISM330_LPF_Update(ISM330_Filtered_t *f, ISM330_Phys_t *phys);

/* LAYER 4 — utility
   ------------------- */

/**
 * @brief  Return a human-readable string for a status code.
 *         Useful for UART debug output.
 */
const char *ISM330_StatusStr(ISM330_Status_t s);

#ifdef __cplusplus
}
#endif

#endif /* ISM330DHCX_I2C_H */
