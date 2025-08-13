CC = gcc
CFLAGS = -Wall -Wfatal-errors -pedantic -static -O2

# all: set_servo_angle

set-servo-angle: set_servo_angle.c i2c_pca_utils.o
	$(CC) $(CFLAGS) $^ -o $@ -li2c

i2c_pca_utils.o: i2c_pca_utils.c i2c_pca_utils.h
	$(CC) $(CFLAGS) -c $< -o $@ -li2c

clean:
	rm -rf set-servo-angle i2c_pca_utils.o
