#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <string.h>
#include <stdlib.h>

#define MAX_BUFFER 10000

struct ClunetMsg {
    unsigned char src_address;
    unsigned char dst_address;
    unsigned char command;
    unsigned char size;
    unsigned char *data;
};

unsigned char buf [MAX_BUFFER];

unsigned char fileBuffer[100000];
long fsize;

int position = 0;

int received = 0;

int fd;

int verbose = 0;

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
    if(verbose) {
        printf("-> src: %d dst: %d cmd: %d data: ", src_address, dst_address, command);
        for(int i=0;i<size;i++) {
            printf("0x%x ", data[i]);
        }
        printf("%s", "\n");
    }
    if(received == 0) {
        received = 1;
    }    
}

void sendCommand(struct ClunetMsg *msg) {
    if(verbose) {
        printf("<- src: %d dst: %d cmd: %d data: ", msg->src_address, msg->dst_address, msg->command);
    }
    int len = msg->size + 4;
    unsigned char buffer[len];

    buffer[0] = msg->size;
    buffer[1] = msg->src_address;
    buffer[2] = msg->dst_address;
    buffer[3] = msg->command;

    for(int i=0;i<msg->size;i++) {
        buffer[i+4] = msg->data[i];
        if(verbose) {
            printf("0x%x ", msg->data[i]);
        }
    }
    if(verbose) {
        printf("%s", "\n");
    }
    int n = write(fd, buffer, len);
    if(verbose) {
        printf("write %d bytes\n", n);
    }
    usleep ((len + 25) * 100); //wait for time enough to transfer
}

struct ClunetMsg *processBuffer() {
    
    if(position <= 0 || position < buf[0] + 4) {
        return NULL;
    }

    unsigned char size = buf[0];
    unsigned char src_address = buf[1];
    unsigned char dst_address = buf[2];
    unsigned char command = buf[3];

    struct ClunetMsg *msg = malloc(sizeof(struct ClunetMsg));

    msg->src_address = src_address;
    msg->dst_address = dst_address;
    msg->command = command;
    msg->size = size;
    msg->data = malloc(size);

    for(int i=0;i<size;i++) {
        msg->data[i] = buf[i+4];
    }

    messageReceived(src_address, dst_address, command, size, buf + 4);

    int len = size + 4;
    for(int i=0;i<position - len;i++) {
        buf[i] = buf[i+len];
    }
    position = position - len;
    return msg;
}

struct ClunetMsg *readMessage() {
    int count  = 30;
    while(count > 0) {
        int n = read (fd, buf + position, MAX_BUFFER - position);  // read up to 100 characters if ready to read

        // printf("count %d\n", count);
        if(n > 0) {            
            position = position + n;
            struct ClunetMsg *msg = processBuffer();

            if(msg != NULL) {
                return msg;
            }
        } else {
            count --;
        }
        usleep (100);
    }
    return NULL;
}

struct ClunetMsg *waitForMessage2(unsigned char src_address, unsigned char command) {
    int count  = 10;
    while((count --) > 0) {
        struct ClunetMsg *msg = readMessage();
        if(msg == NULL) {
            return NULL;
        }
        if(msg->src_address == src_address && msg->command == command) {
            return msg;
        }
        free(msg->data);
        free(msg);
    }
    return NULL;
}

struct ClunetMsg *waitForMessage(unsigned char src_address, unsigned char command, unsigned char subcommand) {
    int count  = 10;
    while((count--) > 0) {
        struct ClunetMsg *msg = readMessage();
        if(msg == NULL) {
            return NULL;
        }
        if(msg->src_address == src_address && msg->command == command && msg->data[0] == subcommand) {
            return msg;
        }
        free(msg->data);
        free(msg);
    }
    return NULL;
}

void loadFile() {
    FILE *f = fopen("../switch/clunet-demo.bin", "r");
    fseek(f, 0, SEEK_END);
    fsize = ftell(f);
    printf("File of size %ld loaded..\n", fsize);
    fseek(f, 0, SEEK_SET);  //same as rewind(f);
    
    fread(fileBuffer, fsize, 1, f);
    fclose(f);    
}

int main(int argc, char** arg) {
    printf("%s\n", "Started..");
    loadFile();
    char *portname = "/dev/cu.wchusbserial1410";
    int deviceId = 99;

    fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        char buffer[100];
        sprintf(buffer, "error opening %s", portname);
        error_message (buffer, -6);
        return -1;
    }

    printf("Connect to serial %s..\n", portname);

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking

    printf("Serial connection established..\n");

    printf("Try to reboot device: %d\n", deviceId);

    while(1) {
        struct ClunetMsg m0 = {255, deviceId, 3, 0, NULL};
        sendCommand(&m0);

        struct ClunetMsg *msg = waitForMessage(deviceId, 2, 0);
        if(msg != NULL) {
            free(msg->data);
            free(msg);
            break;
        }
    }

    unsigned char data[1];
    data[0] = 1;

    struct ClunetMsg m = {255, deviceId, 2, 1, data};
    sendCommand(&m);

    struct ClunetMsg *msg = waitForMessage(deviceId, 2, 2);
    int pageSize = msg->data[1] + (msg->data[2] << 8);
    printf("Connected, page size is %d, uploading:\n", pageSize);
    if(msg != NULL) {
        free(msg->data);
        free(msg);
    }

    int n = fsize / pageSize + 1;

    int p = 0;
    int oldP = 0;

    for(int i=0;i<n;i++) {
        unsigned char data[pageSize + 1 + 4];
        data[0] = 3;
        int offset = i*pageSize;
        data[1] = (offset) & 0xFF;
        data[2] = (offset >> 8) & 0xFF;
        data[3] = (offset >> 16) & 0xFF;
        data[4] = (offset >> 24) & 0xFF;
        for(int j=0;j<pageSize;j++) {
            data[j + 1 + 4] = fileBuffer[i*pageSize+j];
        }

        struct ClunetMsg m = {255, deviceId, 2, pageSize+1+4, data};
        sendCommand(&m);

        msg = waitForMessage(deviceId, 2, 4);
        if(msg != NULL) {
            free(msg->data);
            free(msg);
        }

        p = i * 100 / n;

        if(p != 0 && p != oldP) {
            printf("%d%%\n", p);
            oldP = p;
        }
    }

    printf("100%%\n");

    data[0] = 5;

    struct ClunetMsg m2 = {255, deviceId, 2, 1, data};
    sendCommand(&m2);

    msg = waitForMessage2(deviceId, 4);
    if(msg != NULL) {
        free(msg->data);
        free(msg);
    }

    printf("COMPLETED..\n");

    return 0;
}