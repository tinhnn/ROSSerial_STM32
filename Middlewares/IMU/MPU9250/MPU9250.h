/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>

/*******************************************************************************
 * MPU9250 Register [https://invensense.tdk.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf]
 ******************************************************************************/
#define SELF_TEST_X_GYRO                0x00            /* Gyroscope Self-Test Registers */
#define SELF_TEST_Y_GYRO                0x01            /* Gyroscope Self-Test Registers */
#define SELF_TEST_Z_GYRO                0x02            /* Gyroscope Self-Test Registers */

#define SELF_TEST_X_ACCEL               0x0D            /* Accelerometer Self-Test Registers */
#define SELF_TEST_Y_ACCEL               0x0E            /* Accelerometer Self-Test Registers */
#define SELF_TEST_Z_ACCEL               0x0F            /* Accelerometer Self-Test Registers */

#define XG_OFFSET_H                     0x13            /* Gyro Offset Registers */
#define XG_OFFSET_L                     0x14            /* Gyro Offset Registers */
#define YG_OFFSET_H                     0x15            /* Gyro Offset Registers */
#define YG_OFFSET_L                     0x16            /* Gyro Offset Registers */
#define ZG_OFFSET_H                     0x17            /* Gyro Offset Registers */
#define ZG_OFFSET_L                     0x18            /* Gyro Offset Registers */

#define SMPLRT_DIV                      0x19            /* Sample Rate Divider */
#define CONFIG                          0x1A            /* Configuration */
#define GYRO_CONFIG                     0x1B            /* Gyroscope Configuration */
#define ACCEL_CONFIG                    0x1C            /* Accelerometer Configuration */
#define ACCEL_CONFIG2                   0x1D            /* Accelerometer Configuration 2 */
#define LP_ACCEL_ODR                    0x1E            /* Low Power Accelerometer ODR Control */
#define WOM_THR                         0x1F            /* Wake-on Motion Threshold */

#define FIFO_EN                         0x23            /* FIFO Enable */
#define I2C_MST_CTRL                    0x24            /* I2C Master Control */
#define I2C_SLV0_ADDR                   0x25            /* I2C Slave 0 Control */
#define I2C_SLV0_REG                    0x26            /* I2C Slave 0 Control */
#define I2C_SLV0_CTRL                   0x27            /* I2C Slave 0 Control */
#define I2C_SLV1_ADDR                   0x28            /* I2C Slave 1 Control */
#define I2C_SLV1_REG                    0x29            /* I2C Slave 1 Control */
#define I2C_SLV1_CTRL                   0x2A            /* I2C Slave 1 Control */
#define I2C_SLV2_ADDR                   0x2B            /* I2C Slave 2 Control */
#define I2C_SLV2_REG                    0x2C            /* I2C Slave 2 Control */
#define I2C_SLV2_CTRL                   0x2D            /* I2C Slave 2 Control */
#define I2C_SLV3_ADDR                   0x2E            /* I2C Slave 3 Control */
#define I2C_SLV3_REG                    0x2F            /* I2C Slave 3 Control */
#define I2C_SLV3_CTRL                   0x30            /* I2C Slave 3 Control */
#define I2C_SLV4_ADDR                   0x31            /* I2C Slave 4 Control */
#define I2C_SLV4_REG                    0x32            /* I2C Slave 4 Control */
#define I2C_SLV4_DO                     0x33            /* I2C Slave 4 Control */
#define I2C_SLV4_CTRL                   0x34            /* I2C Slave 4 Control */
#define I2C_SLV4_DI                     0x35            /* I2C Slave 4 Control */
#define I2C_MST_STATUS                  0x36            /* I2C Master Status */
#define INT_PIN_CFG                     0x37            /* INT Pin / Bypass Enable Configuration */

#define INT_ENABLE                      0x38            /* Interrupt Enable */
#define INT_STATUS                      0x3A            /* Interrupt Status */

#define ACCEL_XOUT_H                    0x3B            /* Accelerometer Measurements */
#define ACCEL_XOUT_L                    0x3C            /* Accelerometer Measurements */
#define ACCEL_YOUT_H                    0x3D            /* Accelerometer Measurements */
#define ACCEL_YOUT_L                    0x3E            /* Accelerometer Measurements */
#define ACCEL_ZOUT_H                    0x3F            /* Accelerometer Measurements */
#define ACCEL_ZOUT_L                    0x40            /* Accelerometer Measurements */

#define TEMP_OUT_H                      0x41            /* Temperature Measurement */
#define TEMP_OUT_L                      0x42            /* Temperature Measurement */

#define GYRO_XOUT_H                     0x43            /* Gyroscope Measurements */
#define GYRO_XOUT_L                     0x44            /* Gyroscope Measurements */
#define GYRO_YOUT_H                     0x45            /* Gyroscope Measurements */
#define GYRO_YOUT_L                     0x46            /* Gyroscope Measurements */
#define GYRO_ZOUT_H                     0x47            /* Gyroscope Measurements */
#define GYRO_ZOUT_L                     0x48            /* Gyroscope Measurements */

#define EXT_SENS_DATA_00                0x49            /* External Sensor Data */
#define EXT_SENS_DATA_01                0x4A            /* External Sensor Data */
#define EXT_SENS_DATA_02                0x4B            /* External Sensor Data */
#define EXT_SENS_DATA_03                0x4C            /* External Sensor Data */
#define EXT_SENS_DATA_04                0x4D            /* External Sensor Data */
#define EXT_SENS_DATA_05                0x4E            /* External Sensor Data */
#define EXT_SENS_DATA_06                0x4F            /* External Sensor Data */
#define EXT_SENS_DATA_07                0x50            /* External Sensor Data */
#define EXT_SENS_DATA_08                0x51            /* External Sensor Data */
#define EXT_SENS_DATA_09                0x52            /* External Sensor Data */
#define EXT_SENS_DATA_10                0x53            /* External Sensor Data */
#define EXT_SENS_DATA_11                0x54            /* External Sensor Data */
#define EXT_SENS_DATA_12                0x55            /* External Sensor Data */
#define EXT_SENS_DATA_13                0x56            /* External Sensor Data */
#define EXT_SENS_DATA_14                0x57            /* External Sensor Data */
#define EXT_SENS_DATA_15                0x58            /* External Sensor Data */
#define EXT_SENS_DATA_16                0x59            /* External Sensor Data */
#define EXT_SENS_DATA_17                0x5A            /* External Sensor Data */
#define EXT_SENS_DATA_18                0x5B            /* External Sensor Data */
#define EXT_SENS_DATA_19                0x5C            /* External Sensor Data */
#define EXT_SENS_DATA_20                0x5D            /* External Sensor Data */
#define EXT_SENS_DATA_21                0x5E            /* External Sensor Data */
#define EXT_SENS_DATA_22                0x5F            /* External Sensor Data */
#define EXT_SENS_DATA_23                0x60            /* External Sensor Data */

#define I2C_SLV0_DO                     0x63            /* I2C Slave 0 Data Out */
#define I2C_SLV1_DO                     0x64            /* I2C Slave 1 Data Out */
#define I2C_SLV2_DO                     0x65            /* I2C Slave 2 Data Out */
#define I2C_SLV3_DO                     0x66            /* I2C Slave 3 Data Out */
#define I2C_MST_DELAY_CTRL              0x67            /* I2C Master Delay Control */
#define SIGNAL_PATH_RESET               0x68            /* Signal Path Reset */
#define MOT_DETECT_CTRL                 0x69            /* Accelerometer Interrupt Control */
#define USER_CTRL                       0x6A            /* User Control */
#define PWR_MGMT_1                      0x6B            /* Power Management 1 */
#define PWR_MGMT_2                      0x6C            /* Power Management 2 */

#define FIFO_COUNTH                     0x72            /* FIFO Count Registers */
#define FIFO_COUNTL                     0x73            /* FIFO Count Registers */
#define FIFO_R_W                        0x74            /* FIFO Read Write */
#define WHO_AM_I                        0x75            /* Who Am I */

#define XA_OFFSET_H                     0x77            /* Accelerometer Offset Registers */
#define XA_OFFSET_L                     0x78            /* Accelerometer Offset Registers */

#define YA_OFFSET_H                     0x7A            /* Accelerometer Offset Registers */
#define YA_OFFSET_L                     0x7B            /* Accelerometer Offset Registers */

#define ZA_OFFSET_H                     0x7D            /* Accelerometer Offset Registers */
#define ZA_OFFSET_L                     0x7E            /* Accelerometer Offset Registers */
//
#define I2C_READ                        0x80            /*  */

/**********************   Register Map for Magnetometer   *********************/
#define AK8963_WIA                      0x00            /* Device ID */
#define AK8963_INFO                     0x01            /* Information */
#define AK8963_ST1                      0x02            /* Status 1 */
#define AK8963_XOUT_L                   0x03            /* Measurement data */
#define AK8963_XOUT_H                   0x04            /* Measurement data */
#define AK8963_YOUT_L                   0x05            /* Measurement data */
#define AK8963_YOUT_H                   0x06            /* Measurement data */
#define AK8963_ZOUT_L                   0x07            /* Measurement data */
#define AK8963_ZOUT_H                   0x08            /* Measurement data */
#define AK8963_ST2                      0x09            /* Status 2 */
#define AK8963_CNTL                     0x0A            /* Control */
#define MPU9250_AK8963_CNTL2            0x0B            /* Control 2*/
#define AK8963_RSV                      0x0B            /* Reserved: DO NOT ACCESS */
#define AK8963_ASTC                     0x0C            /* Self-test */
#define AK8963_TS1                      0x0D            /* Test 1: DO NOT ACCESS */
#define AK8963_TS2                      0x0E            /* Test 2: DO NOT ACCESS */
#define AK8963_I2CDIS                   0x0F            /* I2C disable */
#define AK8963_ASAX                     0x10            /* X-axis sensitivity adjustment value */
#define AK8963_ASAY                     0x11            /* Y-axis sensitivity adjustment value */
#define AK8963_ASAZ                     0x12            /* Z-axis sensitivity adjustment value */

/**********************   Register Map for Magnetometer   *********************/
#define AK8963_I2C_ADDR                 0x0C            /*  */
#define AK8963_POWER_DOWN               0x10            /*  */
#define AK8963_FUSE_ROM_ACCESS          0x1F            /*  */
#define AK8963_SINGLE_MEASUREMENT       0x11            /*  */
#define AK8963_CONTINUOUS_MEASUREMENT   0x16            /*  */
#define AK8963_DATA_READY               (0x01)          /*  */
#define AK8963_DATA_OVERRUN             (0x02)          /*  */
#define AK8963_OVERFLOW                 (0x80)          /*  */
#define AK8963_DATA_ERROR               (0x40)          /*  */
#define AK8963_CNTL2_SRST               0x01            /*  */

//
#define I2C_SLV4_EN                     0x80            /*  */
#define I2C_SLV4_DONE                   0x40            /*  */
#define I2C_SLV4_NACK                   0x10            /*  */
//
#define I2C_IF_DIS                      (0x10)          /*  */
#define I2C_MST_EN                      (0x20)          /*  */
#define FIFO_RST                        (0x04)          /*  */
#define FIFO_ENABLE                     (0x40)          /*  */
//
#define RESET                           0x80            /*  */
#define CLOCK_MASK                      0xF8            /*  */
#define CLOCK_INTERNAL                  0x00            /*  */
#define CLOCK_PLL                       0x01            /*  */
#define CLOCK_PLLGYROZ                  0x03            /*  */
#define FS_SEL_MASK                     0xE7            /*  */
#define SLEEP_MASK                      0x40            /*  */
//
#define XYZ_GYRO                        0xC7            /*  */
#define XYZ_ACCEL                       0xF8            /*  */
//
#define RAW_RDY_EN                      (0x01)          /*  */
#define RAW_DATA_RDY_INT                (0x01)          /*  */
#define FIFO_OVERFLOW                   (0x10)          /*  */
//
#define INT_ANYRD_2CLEAR                (0x10)          /*  */
#define LATCH_INT_EN                    (0x20)          /*  */
//
#define MAX_FIFO                        (1024)          /*  */
#define FIFO_SIZE_1024                  (0x40)          /*  */
#define FIFO_SIZE_2048                  (0x80)          /*  */
#define FIFO_SIZE_4096                  (0xC0)          /*  */

#define TEMP_OUT                        (0x80)          /*  */
#define GYRO_XOUT                       (0x40)          /*  */
#define GYRO_YOUT                       (0x20)          /*  */
#define GYRO_ZOUT                       (0x10)          /*  */
#define ACCEL                           (0x08)          /*  */

/*******************************************************************************
 * MPU9250
 ******************************************************************************/
class MPU9250
{
    public:
        MPU9250();

        float quat[4];
    private:
};

#endif /* MPU9250_H */
