#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

// Initialize USB serial number based on DIP switches
// Call this BEFORE stdio_init_all() to ensure proper enumeration
void usb_serial_init(void);

#endif // USB_DESCRIPTORS_H