#include "i2c_pca_utils.h"


int main(int argc, char **argv)
{
    int fd, cn, i2c_bus, slave_addr;
    double angle;
    struct servo_type st;

    st.pwm_freq = 50;
    st.pulse_len_min = 1.0;
    st.pulse_len_max = 2.0;
    st.angle_range = 90;
    i2c_bus = 1;
    slave_addr = 0x40;

    if (argc != 3){
        puts("USAGE: set-servo-angle <channel> <angle>");
        return 0;
    } else {
        sscanf(argv[1], "%d", &cn);
        sscanf(argv[2], "%lf", &angle);
        printf("cn = %d, angle = %lf\n", cn, angle);
    }
    fd = connect_i2c(i2c_bus);
    if (fd < 0){
        puts("Can't connect to I2C device.");
        return 1;
    }
    if (select_slave_addr(fd, slave_addr) < 0){
        puts("Can't select slave addr \"0x40\".");
        return 1;
    }
    if (set_pwm_freq(fd, st.pwm_freq, in_osc_hz) < 0){
        puts("Can't set pwm freq \"50\".");
        return 1;
    }
    wake_up(fd);
    set_servo_angle(fd, cn, angle, &st);
    /* print_regs(fd, cn); */
    return 0;
}
