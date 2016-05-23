/**
Алгоритм работы с бутлоадером такой:
1) При необходимости посылаем CLUNET_COMMAND_REBOOT, чтобы устройство перезагрузилось в бутлоадер.
Далее всё взаимодействие идёт через команду CLUNET_COMMAND_BOOT_CONTROL, первый байт в поле данных - код операции.
2) Устройство посылает CLUNET_COMMAND_BOOT_CONTROL, в данных 0. Так мы узнали, что бутлоадер запустился.
3) Сразу же посылаем на адрес устройства CLUNET_COMMAND_BOOT_CONTROL, в данных 1. Это переводит устройство в режим прошивки.
4) Устройство посылает CLUNET_COMMAND_BOOT_CONTROL, первый байт данных - 2, подтверждает, что перешли в режим прошивки. Следующие два байта - это размер страницы.
5) Посылаем в устройство CLUNET_COMMAND_BOOT_CONTROL, первый байт - 3. Это команда на запись страницы прошивки. Байты с 1го по 4й - это смещение от начала прошивки, все остальные байтики - сам кусок прошивки. Этот кусок должен быть равен размеру страницы, который мы получили в пункте 4 (может и не соответствовать реальному, если реальный слишком большой).
6) Устройство посылает CLUNET_COMMAND_BOOT_CONTROL, в данных 4 - это подтверждает, что страница прошита. После этого повторяем пункты 5-6, пока не отправим всю прошивку.
7) Посылаем в устройство CLUNET_COMMAND_BOOT_CONTROL, в данных 5 - это завершает работу бутлоадера и запускает только что прошитый код.
*/

#include "defines.h"
#include "../defines.h"
#include "../clunet_config.h"
#include "../../clunet.h"
#include "../../bits.h"
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/interrupt.h>

#define COMMAND_FIRMWARE_UPDATE_START    0
#define COMMAND_FIRMWARE_UPDATE_INIT     1
#define COMMAND_FIRMWARE_UPDATE_READY    2
#define COMMAND_FIRMWARE_UPDATE_WRITE    3
#define COMMAND_FIRMWARE_UPDATE_WRITTEN  4
#define COMMAND_FIRMWARE_UPDATE_DONE     5

#define APP_END (FLASHEND - (BOOTSIZE * 2))
#define COMMAND buffer[CLUNET_OFFSET_COMMAND]
#define SUB_COMMAND buffer[CLUNET_OFFSET_DATA]

#define PAUSE(t) {CLUNET_TIMER_REG = 0; while(CLUNET_TIMER_REG < (t*CLUNET_T));}
#define SEND_BIT(t) {CLUNET_SEND_1; PAUSE(t); CLUNET_SEND_0; PAUSE(1);}

#if SPM_PAGESIZE > 64
#define MY_SPM_PAGESIZE 64
#else 
#define MY_SPM_PAGESIZE SPM_PAGESIZE
#endif

char update_start_command[7];
char update_start_command_size;

volatile char update = 0;
char buffer[MY_SPM_PAGESIZE+0x10];
static void (*jump_to_app)(void) = 0x0000;

char check_crc(char* data, unsigned char size) {
      uint8_t crc=0;
      uint8_t i,j;
      for (i=0; i<size;i++) {
            uint8_t inbyte = data[i];
            for (j=0;j<8;j++) {
                  uint8_t mix = (crc ^ inbyte) & 0x01;
                  crc >>= 1;
                  if (mix) 
                        crc ^= 0x8C;
                  
                  inbyte >>= 1;
            }
      }
      return crc;
}

void send() {
    CLUNET_SEND_0;
    char crc = check_crc(update_start_command, update_start_command_size);
    PAUSE(3);
    SEND_BIT(10); // Init
    SEND_BIT(3); SEND_BIT(3);// Prio
    short int i, m;
    for(i = 0; i <= update_start_command_size; i++) {
        char b = (i < update_start_command_size) ? update_start_command[i] : crc;
        for (m = 0; m < 8; m++) {
            CLUNET_SEND_1; 
            short int p = (b & (1<<m)) ? 3 : 1;
            PAUSE(p); CLUNET_SEND_0; PAUSE(1);
        }
    }   
}

char wait_for_signal() {
    int time = 0; 
    CLUNET_TIMER_REG = 0;
    while (time < ((BOOTLOADER_TIMEOUT*CLUNET_T)>>8)) {
        if (CLUNET_READING) return 1;
        if (CLUNET_TIMER_REG >= 254) {
            CLUNET_TIMER_REG = 0;
            time++;
        }
    }   
    return 0;
}

int read() {
    int current_byte = 0, current_bit = 0;
    do {
        if (!wait_for_signal()) {
            return 0;
        }
        CLUNET_TIMER_REG = 0;
        while (CLUNET_READING) {
            ;
        }
    } while (CLUNET_TIMER_REG <  (CLUNET_1_T+ CLUNET_INIT_T)/2);
    
    if (!wait_for_signal()) {
        return 0; // Init
    }

    while (CLUNET_READING) {
        ;
    }

    if (!wait_for_signal()) {
        return 0;
    }

    while (CLUNET_READING) {
        ;
    }

    current_byte = 0;
    current_bit = 0;
    
    do {
        buffer[current_byte] = 0;
        for (current_bit = 0; current_bit < 8; current_bit++) {
            if (!wait_for_signal()) {
                return 0;
            }
            CLUNET_TIMER_REG = 0;
            while (CLUNET_READING) {
                ;
            }
            if (CLUNET_TIMER_REG > (CLUNET_0_T+CLUNET_1_T)/2) {
                buffer[current_byte] |= (1<<current_bit);
            }
        }
        current_byte++;         
    } while (((current_byte < 4) || (current_byte < buffer[CLUNET_OFFSET_SIZE]+CLUNET_OFFSET_DATA+1)) && (current_byte < 512));

    if ((buffer[CLUNET_OFFSET_DST_ADDRESS] == CLUNET_DEVICE_ID) && (check_crc(buffer, current_byte) == 0) && (buffer[CLUNET_OFFSET_COMMAND] == CLUNET_COMMAND_BOOT_CONTROL)) {
        return buffer[CLUNET_OFFSET_SIZE];
    }
    return -1; // Пришёл пакет, но левый
}

void write_flash_page(uint32_t address, char* pagebuffer) {
    eeprom_busy_wait ();

#if MY_SPM_PAGESIZE != SPM_PAGESIZE
    if (address % SPM_PAGESIZE == 0)
#endif
    {
        boot_page_erase (address);
        boot_spm_busy_wait ();      // Wait until the memory is erased.
    }

    int i;
    for (i=0; i<MY_SPM_PAGESIZE; i+=2) {
        // Set up little-endian word.
        uint16_t w = *((uint16_t*)(pagebuffer + i));
        boot_page_fill (address + i, w);
    }

    boot_page_write(address);     // Store buffer in flash page.
    boot_spm_busy_wait();            // Wait until the memory is written.

    boot_rww_enable ();
}

void send_firmware_command(char b) {
    update_start_command[0] = CLUNET_DEVICE_ID;
    update_start_command[1] = CLUNET_BROADCAST_ADDRESS;
    update_start_command[2] = CLUNET_COMMAND_BOOT_CONTROL;
    update_start_command[3] = 1;
    update_start_command[4] = b;
    update_start_command_size = 5;
    send();
}

void firmware_update() {
    update_start_command[0] = CLUNET_DEVICE_ID;
    update_start_command[1] = CLUNET_BROADCAST_ADDRESS;
    update_start_command[2] = CLUNET_COMMAND_BOOT_CONTROL;
    update_start_command[3] = 3;
    update_start_command[4] = COMMAND_FIRMWARE_UPDATE_READY;
    update_start_command[5] = (MY_SPM_PAGESIZE&0xFF);
    update_start_command[6] = (MY_SPM_PAGESIZE>>8);
    update_start_command_size = 7;
    send();

    while(1) {
        int r = read();
        if (r > 0) {
            switch(SUB_COMMAND) {
                case COMMAND_FIRMWARE_UPDATE_INIT:
                    firmware_update();
                    break;
                case COMMAND_FIRMWARE_UPDATE_WRITE: {
                        uint16_t address = *((uint32_t*)(buffer+CLUNET_OFFSET_DATA+1));
                        char* pagebuffer = buffer+CLUNET_OFFSET_DATA+5;
                        write_flash_page(address, pagebuffer);
                        send_firmware_command(COMMAND_FIRMWARE_UPDATE_WRITTEN);
                    }
                    break;
                case COMMAND_FIRMWARE_UPDATE_DONE:
                    jump_to_app();
                    break;
            }
        }
    }
}

int main (void) {
    cli();
    CLUNET_TIMER_INIT;
    CLUNET_READ_INIT;
    CLUNET_SEND_INIT;
    
    send_firmware_command(COMMAND_FIRMWARE_UPDATE_START);

    int r = read();
    
    if ((r > 0) && (SUB_COMMAND == COMMAND_FIRMWARE_UPDATE_INIT)) {
        firmware_update();
    }

    jump_to_app();
    //asm("rjmp 0000");
    return 0;
}
