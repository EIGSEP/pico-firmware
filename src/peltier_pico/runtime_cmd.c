#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/bootrom.h"          
#include "pico/stdio.h"
#include "hardware/regs/usb.h"
#include "runtime_cmd.h"

static void snap_and_print(const HBridge *hb) {
    printf("%lu,%.2f,%.2f,%.2f\r\n",
           (unsigned long)hb->t_now,
           hb->T_now, hb->T_target, hb->drive);
}

int host_cmd_execute(char *line, HBridge *hb) {
    
    if (strcmp(line, "REQ") == 0) {
    snap_and_print(hb);
    return 0;

    } else if (strncmp(line, "SET,", 4) == 0) {          // SET,<temp>
        float t = atof(&line[4]);
        hb->T_target = t;
        printf("ACK: set %.2f\r\n", t);
        return 0;   

    } else if (strcmp(line, "STOP") == 0) {              // pause control loop, hb -> enabled = false/true allows us to enable/disable H-bridge at runtime
        hb->enabled = false;
        hb->drive   = 0.0f;
        printf("ACK: stopped\r\n");
        return 0;
    
    } else if (strcmp(line, "RESUME") == 0) {            // resume control loop
        hb->enabled = true;
        printf("ACK: resumed\r\n");
        return 0;
    
    } else if (strcmp(line, "BOOTSEL") == 0) {            // reboots device into BOOTSEL mode
        printf("ACK: reboot to BOOTSEL\r\n"); 
        sleep_ms(20);
        rom_reset_usb_boot(0, 0);                        // never returns, funtion defined on pg. 477 pico C/C++ SDK guide. passing (0, 0) keeps USB active, host will see new RP2040 device.
        
    } else if (strncmp(line, "HYST,", 5) == 0) {          // Hysteresis, <∆T>
        float h = atof(&line[5]);
        hb->hysteresis = h;
        printf("ACK: hysteresis %.2f\r\n", h);
        return 0;
    }
    
      // else if (strncmp(line, "GAIN,", 5) == 0) {         // GAIN, kp, ki, kd
      //   if (sscanf(&line[5], "%f,%f,%f",
      //              &hb->kp, &hb->ki, &hb->kd) == 3) {
      //       printf("ACK: gain %.3f,%.3f,%.3f\r\n",
      //              hb->kp, hb->ki, hb->kd);
      //       return 0;
      //   }
      //   return -1;
    return -1;
    
}
