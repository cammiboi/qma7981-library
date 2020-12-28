#ifndef QMA7981_H
#define QMA7981_H
#include <Arduino.h>

#define ADO 0 // I2C Address pin, 0 for low and 1 for high

enum qma7981_full_scale_range_t
{
    RANGE_2G = 0b0001,
    RANGE_4G = 0b0010,
    RANGE_8G = 0b0100,
    RANGE_16G = 0b1000,
    RANGE_32G = 0b1111
};

enum qma7981_bandwidth_t
{
    MCLK_DIV_BY_7695 = 0b000,
    MCLK_DIV_BY_3855 = 0b001,
    MCLK_DIV_BY_1935 = 0b010,
    MCLK_DIV_BY_975 = 0b011,
    MCLK_DIV_BY_15375 = 0b101,
    MCLK_DIV_BY_30735 = 0b110,
    MCLK_DIV_BY_61455 = 0b111
};

enum qma7981_clock_freq_t
{
    CLK_500_KHZ = 0b0001,
    CLK_333_KHZ = 0b0000,
    CLK_200_KHZ = 0b0010,
    CLK_100_KHZ = 0b0011,
    CLK_50_KHZ = 0b0100,
    CLK_25_KHZ = 0b0101,
    CLK_12_KHZ_5 = 0b0110,
    CLK_5_KHZ = 0b0111
};

enum qma7981_no_motion_duration_t
{
    NO_MOTION_1_SEC = 0b000000,
    NO_MOTION_2_SEC = 0b000001,
    NO_MOTION_3_SEC = 0b000010,
    NO_MOTION_5_SEC = 0b000100,
    NO_MOTION_10_SEC = 0b001001,
    NO_MOTION_15_SEC = 0b001110,
    NO_MOTION_30_SEC = 0b010010,
    NO_MOTION_1_MIN = 0b011000,
    NO_MOTION_2_MIN = 0b100010,
    NO_MOTION_3_MIN = 0b101000,
    NO_MOTION_4_MIN = 0b101110
};

enum qma7981_any_motion_samples_t
{
    NUM_SAMPLES_1 = 0b00,
    NUM_SAMPLES_2 = 0b01,
    NUM_SAMPLES_3 = 0b10,
    NUM_SAMPLES_4 = 0b11
};

enum qma7981_mode_t
{
    MODE_STANDBY = 0,
    MODE_ACTIVE = 1
};

enum qma7981_motion_detect_t
{
    MOTION_DETECT_NOTHING = 0,
    MOTION_DETECT_ANY_MOTION = 1,
    MOTION_DETECT_NO_MOTION = 2
};
class QMA7981
{
public:
    QMA7981();
    void initialize_default();
    int16_t get_accel_x();
    int16_t get_accel_y();
    int16_t get_accel_z();
    uint8_t get_chip_id();
    void set_full_scale_range(qma7981_full_scale_range_t range);
    void set_bandwidth(qma7981_bandwidth_t bandwidth);
    void set_clock_freq(qma7981_clock_freq_t freq);
    void set_mode(qma7981_mode_t mode);
    void set_interrupt_pin_1_source(bool significant_step, bool step_valid, bool hand_down,
                                    bool hand_raise, bool significant_motion,
                                    bool any_motion, bool data_ready, bool no_motion);
    void set_interrupt_pin_1_type(bool open_drain, bool active_high);
    void setup_any_motion_detector(bool x_enabled, bool y_enabled, bool z_enabled,
                                   qma7981_any_motion_samples_t samples,
                                   uint8_t threshold);
    void setup_no_motion_detector(bool x_enabled, bool y_enabled, bool z_enabled,
                                  qma7981_no_motion_duration_t duration,
                                  uint8_t threshold);
    qma7981_motion_detect_t get_motion_detected();
    void disable_any_motion_detector();
    void disable_no_motion_detector();
};

#endif