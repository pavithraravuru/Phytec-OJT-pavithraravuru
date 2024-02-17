#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

int set_interface_attribs(int fd, int speed) {
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetispeed(&tty, (speed_t)speed);
    cfsetospeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flow control */

    tty.c_iflag = IGNPAR;
    tty.c_lflag = 0;

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

int main() {
    char *portname = "/dev/ttyS3";
    int fd;
    int wlen, rdlen;

    char arr1[] = "AT";
    char arr2[] = "AT+MODE=TEST";
    char arr3[] = " AT+TEST=RFCFG,866,SF12,125,12,15,14,ON,OFF,OFF";
    char arr4[] = "AT+TEST=TXLRSTR,\"Hello @ phytec\"";

  
    unsigned char buf[100];

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }

    set_interface_attribs(fd, B9600);

    // Write CMD+RESET=0 command
    printf("%s", arr1);
    wlen = write(fd, arr1, sizeof(arr1) - 1);
    sleep(2);
printf("%s", arr2);                                             
    wlen = write(fd, arr2, sizeof(arr2) - 1);                       
    sleep(2);
printf("%s", arr3);                                             
    wlen = write(fd, arr3, sizeof(arr3) - 1);                       
    sleep(2);
printf("%s", arr4);                                             
    wlen = write(fd, arr4, sizeof(arr4) - 1);                       
    sleep(4);   

    while (1) {
        // Read data from the module
        rdlen = read(fd, buf, sizeof(buf) - 1);
        if (rdlen > 0) {
            buf[rdlen] = '\0';
            printf("Received data: %s\n", buf);
        }
    }

    close(fd);
    return 0;
}
