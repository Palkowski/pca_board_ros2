#include "i2c_pca_utils.h"


/* Opens I2C communication and returns file descriptor assigned to I2C
 * adapter, or -1 in case of an error. */
int connect_i2c(int adapter_num)
{
    char pseudo_file_name[20];
    int i2c_fd;
    snprintf(pseudo_file_name, 19, "/dev/i2c-%d", adapter_num);
    i2c_fd = open(pseudo_file_name, O_RDWR);
    return i2c_fd;
}

/* Selects I2C slave address to interact with. Only least 7 bits of `addr`
 * are used. Slave address is an address of PWM board. */
int select_slave_addr(int i2c_fd, long addr)
{
    return ioctl(i2c_fd, I2C_SLAVE, addr);
}

/* Set PRE_SCALE value. Acceptable values are from 0x03 to 0xff. */
int set_prescale(int i2c_fd, int value)
{
    if (value < prescale_min || value > prescale_max)
        return -1;
    return i2c_smbus_write_byte_data(i2c_fd, prescale_reg, value);
}

/* Set prescaler according to desired PWM frequency. Acceptable values are
 * from 24 to 1526 hz. */
int set_pwm_freq(int i2c_fd, int freq, double osc_clock_hz)
{
    int prescale_val;
    prescale_val = (int)(osc_clock_hz / (4096. * freq) - 1.);
    return set_prescale(i2c_fd, prescale_val);
}

/* Set start `time_on` and end `time_off` of high signal of PWM. Limit is
 * 4095 (0xFFF) */
int set_pwm(int i2c_fd, int channel, int time_on, int time_off)
{
    int res;
    if (time_on < pwm_min || time_on > pwm_max)
        return -1;
    if (time_off < pwm_min || time_off > pwm_max)
        return -1;
    res = i2c_smbus_write_byte_data(i2c_fd, pwm_0_on_l_reg  + 4 * channel,
                                    time_on & pwm_on_l);
    if (res < 0){ return res; }
    res = i2c_smbus_write_byte_data(i2c_fd, pwm_0_on_h_reg  + 4 * channel,
                                    (time_on >> 8) & pwm_on_h);
    if (res < 0){ return res; }
    res = i2c_smbus_write_byte_data(i2c_fd, pwm_0_off_l_reg + 4 * channel,
                                    time_off & pwm_off_l);
    if (res < 0){ return res; }
    res = i2c_smbus_write_byte_data(i2c_fd, pwm_0_off_h_reg + 4 * channel,
                                    (time_off >> 8) & pwm_off_h);
    if (res < 0){ return res; }
    return 0;
}

/* Set sleep bit to zero. */
int wake_up(int i2c_fd)
{
    int m1_val;
    m1_val = i2c_smbus_read_byte_data(i2c_fd, mode_1_reg);
    if (m1_val < 0){ return m1_val; }
    m1_val = CLEAR_BITS(m1_val, mode_1_sleep);
    return i2c_smbus_write_byte_data(i2c_fd, mode_1_reg, m1_val);
}

/* Set sleep bit to one. */
int pca_sleep(int i2c_fd)
{
    int m1_val;
    m1_val = i2c_smbus_read_byte_data(i2c_fd, mode_1_reg);
    if (m1_val < 0){ return m1_val; }
    m1_val = SET_BITS(m1_val, mode_1_sleep);
    return i2c_smbus_write_byte_data(i2c_fd, mode_1_reg, m1_val);
}

/* Restart PCA board after it was set to sleep. */
int restart(int i2c_fd)
{
    int m1_val;
    m1_val = i2c_smbus_read_byte_data(i2c_fd, mode_1_reg);
    if (m1_val < 0){ return m1_val; }
    if (m1_val & mode_1_restart){
        m1_val = CLEAR_BITS(m1_val, mode_1_sleep);
        usleep(500);  /* wait 500 microseconds */
        m1_val = SET_BITS(m1_val, mode_1_restart);
        return i2c_smbus_write_byte_data(i2c_fd, mode_1_reg, m1_val);
    }
    return -1;
}

/* Converts time in milliseconds to clock value. */
int ms_to_cnt(float time_ms, int freq_hz)
{
    float period = 1000. / (float)freq_hz;
    if (time_ms >= period)
        return pwm_max;
    if (time_ms <= 0)
        return pwm_min;
    return (int)LIN_MAP(time_ms, 0., period, (float)pwm_min, (float)pwm_max);
}

/* Converts clock value to time in milliseconds. */
float cnt_to_ms(int cnt, int freq_hz)
{
    float period = 1000. / (float)freq_hz;
    return LIN_MAP((float)cnt, (float)pwm_min, (float)pwm_max, 0., period);
}

/* Reads TIME OFF counter value. */
int read_toff(int i2c_fd, int channel)
{
    int toff, tmp;
    toff = i2c_smbus_read_byte_data(i2c_fd, pwm_0_off_l_reg + 4 * channel);
    tmp  = i2c_smbus_read_byte_data(i2c_fd, pwm_0_off_h_reg + 4 * channel);
    toff |= (tmp << 8);
    return toff;
}

/* Reads TIME ON counter value. */
int read_ton(int i2c_fd, int channel)
{
    int ton, tmp;
    ton = i2c_smbus_read_byte_data(i2c_fd, pwm_0_on_l_reg + 4 * channel);
    tmp = i2c_smbus_read_byte_data(i2c_fd, pwm_0_on_h_reg + 4 * channel);
    ton |= (tmp << 8);
    return ton;
}

/* Returns current servo angle in deg. */
int get_servo_angle(int i2c_fd, int channel, int freq_hz)
{
    float ms_on_dur;
    int cnt_on_dur;
    cnt_on_dur = read_toff(i2c_fd, channel);
    cnt_on_dur -= read_ton(i2c_fd, channel);
    ms_on_dur = cnt_to_ms(cnt_on_dur, freq_hz);
    return (int)LIN_MAP(ms_on_dur, servo_high_ms_min, servo_high_ms_max,
            0., 180.);
}

/* Set PWM high voltage time in milliseconds. */
int set_pwm_ms(int i2c_fd, int channel, float hvt_ms, int freq_hz)
{
    return set_pwm(i2c_fd, channel, 0, ms_to_cnt(hvt_ms, freq_hz));
}

/* Loops through all PWM counter values from max to min, sets servo to
 * minimal angle possible. */
void find_min_angle(int i2c_fd, int channel)
{
    int i;
    for (i = pwm_max; i >= pwm_min; i--){
        set_pwm(i2c_fd, channel, 0, i);
    }
}

/* Loops through all PWM counter values from min to max, sets servo to
 * maximal angle possible. */
void find_max_angle(int i2c_fd, int channel)
{
    int i;
    for (i = pwm_min; i <= pwm_max; i++){
        set_pwm(i2c_fd, channel, 0, i);
    }
}

/* Set servo to angle in degrees. */
int set_servo_angle(int i2c_fd, int channel, int angle_deg, int freq_hz)
{
    float hvt = LIN_MAP((float)angle_deg, 0., 180., servo_high_ms_min,
                        servo_high_ms_max);
    return set_pwm_ms(i2c_fd, channel, hvt, (float)freq_hz);
}

/* Set duty PWM cycle from 0 to 100%. */
int set_duty_cycle(int i2c_fd, int channel, float dutyc)
{
    int toff = LIN_MAP(dutyc, 0., 100., (float)pwm_min, (float)pwm_max);
    return set_pwm(i2c_fd, channel, 0, toff);
}

/* Debug info */
void explain_mode_1(int val)
{
    printf("=== MODE 1 INFO ===\n");
    printf("RESTART = %d\n", (val & mode_1_restart) != 0);
    printf("EXTCLK  = %d\n", (val & mode_1_extclk)  != 0);
    printf("AI      = %d\n", (val & mode_1_ai)      != 0);
    printf("SLEEP   = %d\n", (val & mode_1_sleep)   != 0);
    printf("SUB 1   = %d\n", (val & mode_1_sub_1)   != 0);
    printf("SUB 2   = %d\n", (val & mode_1_sub_2)   != 0);
    printf("SUB 3   = %d\n", (val & mode_1_sub_3)   != 0);
    printf("ALLCALL = %d\n", (val & mode_1_allcall) != 0);
    printf("\n");
}

void explain_mode_2(int val)
{
    printf("=== MODE 2 INFO ===\n");
    printf("INVRT  = %d\n", (val & mode_2_invrt)  != 0);
    printf("OCH    = %d\n", (val & mode_2_och)    != 0);
    printf("OUTDRV = %d\n", (val & mode_2_outdrv) != 0);
    printf("OUTNE  = %d\n", (val & mode_2_outne)  != 0);
    printf("\n");
}

void print_regs(int i2c_fd, int channel)
{
    int res, tmp;

    res = i2c_smbus_read_byte_data(i2c_fd, mode_1_reg);
    printf("MODE 1 = %d\n", res);
    explain_mode_1(res);

    res = i2c_smbus_read_byte_data(i2c_fd, mode_2_reg);
    printf("MODE 2 = %d\n", res);
    explain_mode_2(res);

    res = i2c_smbus_read_byte_data(i2c_fd, prescale_reg);
    printf("PRESCALE = %d\n", res);

    res = i2c_smbus_read_byte_data(i2c_fd, pwm_0_on_l_reg + 4 * channel);
    tmp = i2c_smbus_read_byte_data(i2c_fd, pwm_0_on_h_reg + 4 * channel);
    res |= (tmp << 8);
    printf("PWM%d ON = %d\n", channel, res);

    res = i2c_smbus_read_byte_data(i2c_fd, pwm_0_off_l_reg + 4 * channel);
    tmp = i2c_smbus_read_byte_data(i2c_fd, pwm_0_off_h_reg + 4 * channel);
    res |= (tmp << 8);
    printf("PWM%d OFF = %d\n", channel, res);

    res = i2c_smbus_read_byte_data(i2c_fd, all_pwm_on_l_reg);
    tmp = i2c_smbus_read_byte_data(i2c_fd, all_pwm_on_h_reg);
    res |= (tmp << 8);
    printf("ALL PWM ON = %d\n", res);

    res = i2c_smbus_read_byte_data(i2c_fd, all_pwm_off_l_reg);
    tmp = i2c_smbus_read_byte_data(i2c_fd, all_pwm_off_h_reg);
    res |= (tmp << 8);
    printf("ALL PWM OFF = %d\n", res);
}

/* Program for servo testing */
/*
int main(int argc, char **argv)
{
    int fd, cn, angle;
    if (argc != 3){
        printf("USAGE: set-servo-angle <channel> <angle>\n");
    } else {
        sscanf(argv[1], "%d", &cn);
        sscanf(argv[2], "%d", &angle);
        printf("cn = %d, angle = %d\n", cn, angle);
    }
    fd = connect_i2c(1);
    if (fd < 0){
        printf("Can't connect to I2C device.");
        exit(1);
    }
    if (select_slave_addr(fd, 0x40) < 0){
        printf("Can't select slave addr \"0x40\".");
        exit(1);
    }
    if (set_pwm_freq(fd, 50, in_osc_hz) < 0){
        printf("Can't set pwm freq \"50\".\n");
        exit(1);
    }
    wake_up(fd);
    set_servo_angle(fd, cn, angle, 50);
    print_regs(fd, cn);
    return 0;
}
*/
