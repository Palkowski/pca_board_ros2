#ifndef I2C_PCA_UTILS_H_SENTRY
#define I2C_PCA_UTILS_H_SENTRY 1

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#define SET_BITS(a, b)   ((a) | (b))
#define CLEAR_BITS(a, b) ((a) & ~(b))

/* Linear map from range (`from_in`, `to_in`) to (`from_out`, `to_out`). */
#define LIN_MAP(value, from_in, to_in, from_out, to_out) \
    (((value)-(from_in))*((to_out)-(from_out))/ \
     ((to_in)-(from_in))+(from_out))


enum pwm_board_regs {
    mode_1_reg        = 0x00,
    mode_2_reg        = 0x01,
    subaddr_1_reg     = 0x02,
    subaddr_2_reg     = 0x03,
    subaddr_3_reg     = 0x04,
    allcalladdr_reg   = 0x05,
    pwm_0_on_l_reg    = 0x06,  /* pwm regs */
    pwm_0_on_h_reg    = 0x07,
    pwm_0_off_l_reg   = 0x08,
    pwm_0_off_h_reg   = 0x09,  /* and so on until pwm_15 */
    all_pwm_on_l_reg  = 0xfa,
    all_pwm_on_h_reg  = 0xfb,
    all_pwm_off_l_reg = 0xfc,
    all_pwm_off_h_reg = 0xfd,
    prescale_reg      = 0xfe,
    test_mode_reg     = 0xff
};

enum bits {
    bit_0 = 0x01,
    bit_1 = 0x02,
    bit_2 = 0x04,
    bit_3 = 0x08,
    bit_4 = 0x10,
    bit_5 = 0x20,
    bit_6 = 0x40,
    bit_7 = 0x80
};

enum mode_1_bits {
    mode_1_restart = bit_7,
    mode_1_extclk  = bit_6,
    mode_1_ai      = bit_5,
    mode_1_sleep   = bit_4,
    mode_1_sub_1   = bit_3,
    mode_1_sub_2   = bit_2,
    mode_1_sub_3   = bit_1,
    mode_1_allcall = bit_0
};

enum mode_2_bits {
    /* bits 7, 6, 5 are reserved */
    mode_2_invrt   = bit_4,
    mode_2_och     = bit_3,
    mode_2_outdrv  = bit_2,
    mode_2_outne   = 0x03,  /* bits 0 and 1*/
};

enum pwm_on_off_bits {
    pwm_on_l     = 0xff,
    pwm_on_h     = 0x0f,
    pwm_on_full  = 0x10,
    pwm_off_l    = 0xff,
    pwm_off_h    = 0x0f,
    pwm_off_full = 0x10
};

struct servo_type {
    double pwm_freq;       /* Hz */
    double angle_range;    /* deg */
    double pulse_len_min;  /* ms */
    double pulse_len_max;  /* ms */
};

const int pwm_min = 0x0;
const int pwm_max = 0xfff;
const int prescale_min = 0x03;
const int prescale_max = 0xff;
const double in_osc_hz = 25000000.0;  /* PCA9685 internal oscillator */

/* Opens I2C communication and returns file descriptor assigned to I2C
 * adapter, or -1 in case of an error. */
int connect_i2c(int adapter_num);

/* Selects I2C slave address to interact with. Only least 7 bits of `addr`
 * are used. Slave address is an address of PWM board. */
int select_slave_addr(int i2c_fd, long addr);

/* Set PRE_SCALE value. Acceptable values are from 0x03 to 0xff. */
int set_prescale(int i2c_fd, int value);

/* Set PWM frequency. Acceptable values are from 24 to 1526 hz. */
int set_pwm_freq(int i2c_fd, double freq_hz, double osc_clock_hz);

/* Set start `time_on` and end `time_off` of high signal of PWM. Limit is
 * 4095 (0xFFF) */
int set_pwm(int i2c_fd, int channel, int time_on, int time_off);

/* Set sleep bit to zero. */
int wake_up(int i2c_fd);

/* Set sleep bit to one. */
int pca_sleep(int i2c_fd);

/* Restart PCA board after it was set to sleep. */
int restart(int i2c_fd);

/* Converts time in milliseconds to clock value. */
int ms_to_cnt(double time_ms, double freq_hz);

/* Converts clock value to time in milliseconds. */
double cnt_to_ms(int cnt, double freq_hz);

/* Reads TIME OFF counter value. */
int read_toff(int i2c_fd, int channel);

/* Reads TIME ON counter value. */
int read_ton(int i2c_fd, int channel);

/* Returns current servo angle in deg. */
double get_servo_angle(int i2c_fd, int channel,
                      const struct servo_type *cf);

/* Set PWM pulse length in milliseconds. */
int set_pwm_ms(int i2c_fd, int channel, double hvt_ms, double freq_hz);

/* Loops through all PWM counter values from max to min, sets servo to
 * minimum possible angle. */
void find_min_angle(int i2c_fd, int channel);

/* Loops through all PWM counter values from min to max, sets servo to
 * maximum possible angle. */
void find_max_angle(int i2c_fd, int channel);

/* Set servo angle. */
int set_servo_angle(int i2c_fd, int channel, double angle_deg,
                    const struct servo_type *cf);

/* Set duty PWM cycle from 0 to 100%. */
int set_duty_cycle(int i2c_fd, int channel, double dutyc);

/* Debug info */
void print_mode_1(int val);

void print_mode_2(int val);

void print_regs(int i2c_fd, int channel);

#endif
