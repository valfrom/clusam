#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <string.h>

#define MAX_BUFFER 10000

unsigned char buf [MAX_BUFFER];
int position = 0;

void error_message(char *text, int errno) {
    printf("%s\n", text);
}

int set_interface_attribs (int fd, int speed, int parity) {
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
        error_message ("error %d from tcgetattr", -1);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        error_message ("error %d from tcsetattr", -2);
        return -1;
    }
    return 0;
}

void set_blocking (int fd, int should_block) {
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
        error_message ("error %d from tggetattr", -3);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        error_message ("error %d setting term attributes", -4);
    }
}

void messageReceived(unsigned char src_address, unsigned char dst_address, unsigned char command, unsigned char size, const unsigned char *data) {
    printf("src: %d dst: %d cmd: %d data: ", src_address, dst_address, command);
    for(int i=0;i<size;i++) {
        printf("0x%x ", data[i]);
    }
    printf("%s", "\n");
}

void processBuffer() {
    
    while(position > 0 && position >= buf[0] + 4) {
        unsigned char size = buf[0];
        unsigned char src_address = buf[1];
        unsigned char dst_address = buf[2];
        unsigned char command = buf[3];

        messageReceived(src_address, dst_address, command, size, buf + 4);

        int len = size + 4;
        for(int i=0;i<position - len;i++) {
            buf[i] = buf[i+len];
        }
        position = position - len;
    }
}

int main(int argc, char** arg) {
    printf("%s\n", "started..");
    char *portname = "/dev/cu.usbmodem1421";

    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        char buffer[100];
        sprintf(buffer, "error opening %s", portname);
        error_message (buffer, -6);
        return -1;
    }

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set to blocking

    // write (fd, "hello!\n", 7);           // send 7 character greeting

    // usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
                                         // receive 25:  approx 100 uS per char transmit
    
    while(1) {
        int n = read (fd, buf + position, MAX_BUFFER - position);  // read up to 100 characters if ready to read
        
        if(n > 0) {            
            position = position + n;
            processBuffer();
        }
        usleep (100);

    }

    return 0;
}