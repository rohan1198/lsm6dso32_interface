#ifndef LSM6DSO32_REGISTERS_HPP
#define LSM6DSO32_REGISTERS_HPP

#include <cstdint>

namespace lsm6dso32 {

// Device identification register value
constexpr uint8_t WHO_AM_I_VALUE = 0x6C;

// Default I2C address (0x6A shifted left by 1)
constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x6A;

// Register addresses
enum class Register : uint8_t {
    FUNC_CFG_ACCESS          = 0x01,
    PIN_CTRL                 = 0x02,
    FIFO_CTRL1              = 0x07,
    FIFO_CTRL2              = 0x08,
    FIFO_CTRL3              = 0x09,
    FIFO_CTRL4              = 0x0A,
    COUNTER_BDR_REG1        = 0x0B,
    COUNTER_BDR_REG2        = 0x0C,
    INT1_CTRL               = 0x0D,
    INT2_CTRL               = 0x0E,
    WHO_AM_I                = 0x0F,
    CTRL1_XL                = 0x10,
    CTRL2_G                 = 0x11,
    CTRL3_C                 = 0x12,
    CTRL4_C                 = 0x13,
    CTRL5_C                 = 0x14,
    CTRL6_C                 = 0x15,
    CTRL7_G                 = 0x16,
    CTRL8_XL                = 0x17,
    CTRL9_XL                = 0x18,
    CTRL10_C                = 0x19,
    ALL_INT_SRC             = 0x1A,
    WAKE_UP_SRC             = 0x1B,
    STATUS_REG              = 0x1E,
    OUT_TEMP_L              = 0x20,
    OUT_TEMP_H              = 0x21,
    OUTX_L_G                = 0x22,
    OUTX_H_G                = 0x23,
    OUTY_L_G                = 0x24,
    OUTY_H_G                = 0x25,
    OUTZ_L_G                = 0x26,
    OUTZ_H_G                = 0x27,
    OUTX_L_A                = 0x28,
    OUTX_H_A                = 0x29,
    OUTY_L_A                = 0x2A,
    OUTY_H_A                = 0x2B,
    OUTZ_L_A                = 0x2C,
    OUTZ_H_A                = 0x2D
};

// Output Data Rates (ODR) for Accelerometer
enum class AccelODR : uint8_t {
    POWER_DOWN      = 0x00,
    Hz1_6          = 0x0B,
    Hz12_5         = 0x01,
    Hz26           = 0x02,
    Hz52           = 0x03,
    Hz104          = 0x04,
    Hz208          = 0x05,
    Hz416          = 0x06,
    Hz833          = 0x07,
    Hz1666         = 0x08,
    Hz3333         = 0x09,
    Hz6666         = 0x0A
};

// Full-scale selection for Accelerometer
enum class AccelScale : uint8_t {
    G4             = 0x00,  // ±4g
    G32            = 0x01,  // ±32g
    G8             = 0x02,  // ±8g
    G16            = 0x03   // ±16g
};

// Output Data Rates (ODR) for Gyroscope
enum class GyroODR : uint8_t {
    POWER_DOWN      = 0x00,
    Hz12_5         = 0x01,
    Hz26           = 0x02,
    Hz52           = 0x03,
    Hz104          = 0x04,
    Hz208          = 0x05,
    Hz416          = 0x06,
    Hz833          = 0x07,
    Hz1666         = 0x08,
    Hz3333         = 0x09,
    Hz6666         = 0x0A
};

// Full-scale selection for Gyroscope
enum class GyroScale : uint8_t {
    DPS250         = 0x00,  // ±250 dps
    DPS500         = 0x01,  // ±500 dps
    DPS1000        = 0x02,  // ±1000 dps
    DPS2000        = 0x03   // ±2000 dps
};

// Operating modes
enum class OperatingMode : uint8_t {
    HIGH_PERFORMANCE = 0x00,
    LOW_POWER       = 0x01
};

// Filter configurations
enum class AccelFilter : uint8_t {
    ODR_DIV_4      = 0x00,
    ODR_DIV_10     = 0x01,
    ODR_DIV_20     = 0x02,
    ODR_DIV_45     = 0x03,
    ODR_DIV_100    = 0x04,
    ODR_DIV_200    = 0x05,
    ODR_DIV_400    = 0x06,
    ODR_DIV_800    = 0x07,
    DISABLED       = 0x08
};

// Interrupt configurations
struct InterruptConfig {
    static constexpr uint8_t GYRO_DATA_READY    = 0x01;
    static constexpr uint8_t ACCEL_DATA_READY   = 0x02;
    static constexpr uint8_t FIFO_THRESHOLD     = 0x08;
    static constexpr uint8_t FIFO_FULL          = 0x10;
    static constexpr uint8_t FIFO_OVERRUN       = 0x20;
};

// Status register bit masks
struct StatusBits {
    static constexpr uint8_t XLDA = 0x01;  // Accelerometer data available
    static constexpr uint8_t GDA  = 0x02;  // Gyroscope data available
    static constexpr uint8_t TDA  = 0x04;  // Temperature data available
};

// Control register bit masks
struct ControlBits {
    // CTRL3_C register bits
    static constexpr uint8_t BOOT          = 0x80;
    static constexpr uint8_t BDU           = 0x40;
    static constexpr uint8_t H_LACTIVE     = 0x20;
    static constexpr uint8_t PP_OD         = 0x10;
    static constexpr uint8_t SIM           = 0x08;
    static constexpr uint8_t IF_INC        = 0x04;
    static constexpr uint8_t BLE           = 0x02;
    static constexpr uint8_t SW_RESET      = 0x01;
};

// Conversion factors
struct ConversionFactors {
    static constexpr float ACCEL_G4_SCALE   = 0.122f;    // mg/LSB for ±4g
    static constexpr float ACCEL_G8_SCALE   = 0.244f;    // mg/LSB for ±8g
    static constexpr float ACCEL_G16_SCALE  = 0.488f;    // mg/LSB for ±16g
    static constexpr float ACCEL_G32_SCALE  = 0.976f;    // mg/LSB for ±32g
    
    static constexpr float GYRO_250_SCALE   = 8.75f;     // mdps/LSB for ±250dps
    static constexpr float GYRO_500_SCALE   = 17.5f;     // mdps/LSB for ±500dps
    static constexpr float GYRO_1000_SCALE  = 35.0f;     // mdps/LSB for ±1000dps
    static constexpr float GYRO_2000_SCALE  = 70.0f;     // mdps/LSB for ±2000dps
    
    static constexpr float TEMP_SCALE       = 256.0f;    // LSB/°C
    static constexpr float TEMP_OFFSET      = 25.0f;     // °C
};

} // namespace lsm6dso32

#endif // LSM6DSO32_REGISTERS_HPP
