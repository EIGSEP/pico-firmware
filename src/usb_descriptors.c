#include "tusb.h"
#include "pico/unique_id.h"
#include "hardware/gpio.h"

// DIP switch GPIO pins (same as main.c)
#define DIP0_PIN 2
#define DIP1_PIN 3
#define DIP2_PIN 4

// Global storage for dynamic serial number
static char dynamic_serial_number[16] = "PICO_000";

// Read 3-bit DIP switch code
static uint8_t read_dip_code_early(void) {
    // Initialize pins if not already done
    gpio_init(DIP0_PIN);
    gpio_init(DIP1_PIN); 
    gpio_init(DIP2_PIN);
    gpio_set_dir(DIP0_PIN, GPIO_IN);
    gpio_set_dir(DIP1_PIN, GPIO_IN);
    gpio_set_dir(DIP2_PIN, GPIO_IN);
    gpio_pull_down(DIP0_PIN);
    gpio_pull_down(DIP1_PIN);
    gpio_pull_down(DIP2_PIN);
    
    // Small delay for settling
    sleep_us(1000);
    
    return (gpio_get(DIP2_PIN) << 2) |
           (gpio_get(DIP1_PIN) << 1) |
           gpio_get(DIP0_PIN);
}

// Initialize the dynamic serial number based on DIP switches
void usb_serial_init(void) {
    uint8_t dip_code = read_dip_code_early();
    snprintf(dynamic_serial_number, sizeof(dynamic_serial_number), "PICO_%03d", dip_code);
}

// Device Descriptor
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x2E8A, // Raspberry Pi
    .idProduct          = 0x000A, // Raspberry Pi Pico
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

// Return device descriptor
uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*) &desc_device;
}

// Configuration Descriptor
enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, 0x81, 8, 0x02, 0x82, 64),
};

// Return configuration descriptor
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void) index; // for multiple configurations
    return desc_configuration;
}

// String Descriptors
char const* string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 }, // 0: supported language is English (0x0409)
    "Raspberry Pi",                // 1: Manufacturer
    "Pico Multi-App",              // 2: Product
    dynamic_serial_number,         // 3: Serial number (dynamic based on DIP)
    "Pico CDC",                    // 4: CDC Interface
};

static uint16_t _desc_str[32];

// Return string descriptor
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;

    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
        // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) return NULL;

        const char* str = string_desc_arr[index];

        // Cap at max char
        chr_count = strlen(str);
        if (chr_count > 31) chr_count = 31;

        // Convert ASCII string into UTF-16
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

    return _desc_str;
}