#include <Arduino.h>
#include <Wire.h>
#include "qma7981.h"

#if ADO == 1
#define I2C_ADDRESS 0b0010011 // Device address when ADO = 1
#else
#define I2C_ADDRESS 0b0010010 // Device address when ADO = 0
#endif

// private functions

void write_byte(uint8_t address, uint8_t data)
{
    Wire.begin();
    Wire.beginTransmission(I2C_ADDRESS); // Initialize the Tx buffer
    Wire.write(address);                 // Put slave register address in Tx buffer
    Wire.write(data);                    // Put data in Tx buffer
    Wire.endTransmission();              // Send the Tx buffer
}

uint8_t read_byte(uint8_t address)
{
    Wire.begin();
    Wire.beginTransmission(I2C_ADDRESS); // Initialize the Tx buffer
    Wire.write(address);                 // Put slave register address in Tx buffer
    Wire.endTransmission(false);         // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(I2C_ADDRESS, 1);    // Read one byte from slave register address
    uint8_t data = Wire.read();          // Fill Rx buffer with result
    return data;                         // Return data read from slave register
}

int16_t read_accel_axis(uint8_t address_msb)
{
    Wire.begin();
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(address_msb);
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_ADDRESS, 2);
    int16_t data = (Wire.read() & 0b11111100) | (Wire.read() << 8); // dump into a 16 bit signed int, so the sign is correct
    data = data / 4;                                                // divide the result by 4 to maintain the sign, since the data is 14 bits
    return data;
}

void set_bit(uint8_t *byte, uint8_t n, bool value)
{
    *byte = (*byte & ~(1UL << n)) | (value << n);
}

bool get_bit(uint8_t byte, uint8_t n)
{
    return (byte >> n) & 1U;
}

void soft_reset()
{
    write_byte(0x36, 0xB6);
    write_byte(0x36, 0x00);
}

void set_interrupt_all_latch(bool latch)
{
    uint8_t data = read_byte(0x21);
    set_bit(&data, 0, latch);
    set_bit(&data, 7, 1); // clear all interrupts after reading any interrupt status register
    write_byte(0x21, data);
}

void reset_motion_detector(bool any_motion, bool significant_motion, bool no_motion)
{
    // to reset, write 0 first
    uint8_t data = read_byte(0x30);
    set_bit(&data, 2, !no_motion);
    set_bit(&data, 1, !significant_motion);
    set_bit(&data, 0, !any_motion);
    write_byte(0x30, data);
    // then write 1
    set_bit(&data, 2, 1);
    set_bit(&data, 1, 1);
    set_bit(&data, 0, 1);
    write_byte(0x30, data);
}

void set_any_motion_axis(bool x_enabled, bool y_enabled, bool z_enabled)
{
    uint8_t data = read_byte(0x18);
    set_bit(&data, 0, x_enabled);
    set_bit(&data, 1, y_enabled);
    set_bit(&data, 2, z_enabled);
    write_byte(0x18, data);
}

void set_any_motion_number_of_samples(qma7981_any_motion_samples_t samples)
{
    uint8_t data = read_byte(0x2C);
    data &= 0b11111100; // clear bits 1-0
    data |= (samples & 0b11);
    write_byte(0x2C, data);
}

void set_any_motion_threshold(uint8_t threshold)
{
    write_byte(0x2E, threshold);
}

void set_no_motion_axis(bool x_enabled, bool y_enabled, bool z_enabled)
{
    uint8_t data = read_byte(0x18);
    set_bit(&data, 5, x_enabled);
    set_bit(&data, 6, y_enabled);
    set_bit(&data, 7, z_enabled);
    write_byte(0x18, data);
}

void set_no_motion_duration(qma7981_no_motion_duration_t duration)
{
    uint8_t data = read_byte(0x2C);
    data &= 0b00000011; // clear bits 7-2
    data |= ((duration & 0b111111) << 2);
    write_byte(0x2C, data);
}

void set_no_motion_threshold(uint8_t threshold)
{
    write_byte(0x2D, threshold);
}

void set_any_or_significant_motion(bool significant_motion)
{
    uint8_t data = read_byte(0x2F);
    set_bit(&data, 0, significant_motion);
    write_byte(0x2F, data);
}

// public functions

QMA7981::QMA7981()
{
}

void QMA7981::initialize_default()
{
    delay(10);
    soft_reset();
    delay(10);
    set_mode(MODE_ACTIVE);                  // bring out of sleep mode
    set_clock_freq(CLK_50_KHZ);             // set digital clock freq
    set_bandwidth(MCLK_DIV_BY_61455);       // set bandwitch (samples per sec)
    set_full_scale_range(RANGE_2G);         // set full scale acceleration range
    set_interrupt_all_latch(true);          // set interrupt pin to latch after interrupt until interrupt status read
    set_interrupt_pin_1_type(false, false); // set interrupt pin type and logic level
    // enable no motion and any motion interrupts to trigger int pin 1
    set_interrupt_pin_1_source(false, false, false, false, true, true, false, true);
}

int16_t QMA7981::get_accel_x()
{
    return read_accel_axis(0x01);
}

int16_t QMA7981::get_accel_y()
{
    return read_accel_axis(0x03);
}

int16_t QMA7981::get_accel_z()
{
    return read_accel_axis(0x05);
}

uint8_t QMA7981::get_chip_id()
{
    return read_byte(0x00);
}

void QMA7981::set_full_scale_range(qma7981_full_scale_range_t range)
{
    uint8_t data = 0b11110000;
    data |= (range & 0b1111);
    write_byte(0x0F, data);
}

void QMA7981::set_bandwidth(qma7981_bandwidth_t bandwidth)
{
    uint8_t data = 0b11100000;
    data |= (bandwidth & 0b111);
    write_byte(0x10, data);
}

void QMA7981::set_clock_freq(qma7981_clock_freq_t freq)
{
    uint8_t data = read_byte(0x11);
    // TODO T_RSTB_SINC_SEL<1:0, right now kept at default of 0
    data &= 0b11110000;      // clear bits 0-3
    data |= (freq & 0b1111); // set freq on bits 0-3
    write_byte(0x11, data);
}

void QMA7981::set_mode(qma7981_mode_t mode)
{
    uint8_t data = read_byte(0x11);
    set_bit(&data, 7, mode);
    write_byte(0x11, data);
}

void QMA7981::set_interrupt_pin_1_source(bool significant_step, bool step_valid, bool hand_down,
                                         bool hand_raise, bool significant_motion,
                                         bool any_motion, bool data_ready, bool no_motion)
{
    uint8_t data = read_byte(0x19);
    set_bit(&data, 0, significant_motion);
    set_bit(&data, 1, hand_raise);
    set_bit(&data, 2, hand_down);
    set_bit(&data, 3, step_valid);
    set_bit(&data, 6, significant_step);
    write_byte(0x19, data);

    data = read_byte(0x1A);
    set_bit(&data, 0, any_motion);
    set_bit(&data, 4, data_ready);
    set_bit(&data, 7, no_motion);
    write_byte(0x1A, data);
}

void QMA7981::set_interrupt_pin_1_type(bool open_drain, bool active_high)
{
    uint8_t data = read_byte(0x20);
    set_bit(&data, 0, active_high);
    set_bit(&data, 1, open_drain);
    write_byte(0x20, data);
}

void QMA7981::setup_any_motion_detector(bool x_enabled, bool y_enabled, bool z_enabled,
                                        qma7981_any_motion_samples_t samples,
                                        uint8_t threshold)
{
    set_any_motion_axis(x_enabled, y_enabled, z_enabled);
    set_any_motion_number_of_samples(samples);
    set_any_motion_threshold(threshold);
    set_any_or_significant_motion(false);
    reset_motion_detector(true, false, false);
}

void QMA7981::setup_no_motion_detector(bool x_enabled, bool y_enabled, bool z_enabled,
                                       qma7981_no_motion_duration_t duration,
                                       uint8_t threshold)
{
    set_no_motion_axis(x_enabled, y_enabled, z_enabled);
    set_no_motion_duration(duration);
    set_no_motion_threshold(threshold);
    reset_motion_detector(false, false, true);
}

qma7981_motion_detect_t QMA7981::get_motion_detected()
{
    uint8_t data = read_byte(0x09);

    bool no_motion = get_bit(data, 7);
    if (no_motion)
    {
        return MOTION_DETECT_NO_MOTION;
    }

    bool x_any_motion = get_bit(data, 2);
    bool y_any_motion = get_bit(data, 1);
    bool z_any_motion = get_bit(data, 0);
    if (x_any_motion || y_any_motion || z_any_motion)
    {
        return MOTION_DETECT_ANY_MOTION;
    }

    return MOTION_DETECT_NOTHING;
}

void QMA7981::disable_any_motion_detector()
{
    set_any_motion_axis(false, false, false); // disable all 3 any motion detect axis
}

void QMA7981::disable_no_motion_detector()
{
    set_no_motion_axis(false, false, false); // disable all 3 any motion detect axis
}

void qma7981_setup_default()
{
}

// TODO: add interrupt pin 2 set
// TODO: add function to get axis and direction triggering any motion interrupt
// TODO: add function to get step interrupt status
// TODO: add function to get hand raise interrupt status
// TODO: add function to get data ready interrupt status