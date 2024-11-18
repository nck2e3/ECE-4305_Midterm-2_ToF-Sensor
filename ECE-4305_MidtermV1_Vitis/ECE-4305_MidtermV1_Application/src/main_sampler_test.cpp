/*****************************************************************//**
 * @file main_sampler_test.cpp
 *
 * @brief Basic test of nexys4 ddr mmio cores
 *
 * @author p chu
 * @version v1.0: initial release
 *********************************************************************/

/*
 * Hardware specific references...
 *
 * TOF Sensor: 
 *  https://digilent.com/reference/pmod/pmodtof/reference-manual?redirect=1 
 *
 * DSP: 
 *  https://www.renesas.com/en/document/dst/isl29501-datasheet
 * 
 * DSP CALIBRATION: 
 *  https://www.renesas.com/en/document/apn/an1724-isl29501-firmware-routines?_ga=2.14571511.1370529177.1731833754-1292776711.1730131426
 *
 * EEPROM:
 *  http://ww1.microchip.com/downloads/en/devicedoc/atmel-8896e-seeprom-at24c04d-datasheet.pdf?_ga=2.14524279.1370529177.1731833754-1292776711.*1730131426
 *
 */

#include "chu_init.h"
#include "chu_io_map.h"
#include "gpio_cores.h"
#include "xadc_core.h"
#include "sseg_core.h"
#include "spi_core.h"
#include "i2c_core.h"
#include "ps2_core.h"
#include "ddfs_core.h"
#include "adsr_core.h"
#include <cstdint>

// Addresses to i2c devices...
#define dev_PMOD_RENESAS_DSP 0x57   // Renesas DSP onboard the ToF Sensor
#define dev_PMOD_EEPROM 0x50        // ATMEL EEPROM onboard the ToF Sensor

// Terminal color escape sequences...
#define RESET "\033[0m"
#define GREEN "\033[1;32m"
#define BLUE "\033[1;34m"
#define YELLOW "\033[1;33m"
#define RED "\033[1;31m"

/**
 * Simplified read interface for I2C (essentially a random read).
 * Performs a write to set the register address, followed by a read.
 *
 * @param i2c Pointer to the I2C core instance.
 * @param dev_addr I2C device address.
 * @param reg_addr Register address to read from.
 * @param bytes Pointer to the buffer where read data will be stored.
 * @param num Number of bytes to read.
 * @return The number of bytes read from the device.
 */
int easy_read_transaction(I2cCore *i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t *bytes, int num) {
    uint8_t wbytes[1] = {reg_addr};
    i2c->write_transaction(dev_addr, wbytes, 1, 1);
    return i2c->read_transaction(dev_addr, bytes, num, 0);
}

/**
 * Writes recommended initialization values to the ISL29501 DSP registers.
 *
 * @param ISL29501_p Pointer to the I2C core instance for the ISL29501 device.
 * @param dev_addr I2C device address of the ISL29501 DSP.
 */
void write_digilent_values(I2cCore *ISL29501_p, uint8_t dev_addr) {
    uint8_t wbytes[2], bytes[1];

   
    uart.disp("--------------[INITIALIZATION]--------------\r\n");
    uint8_t init_mappings[][2] = {
        {0x10, 0x04}, // Integration Period Register
        {0x11, 0x6E}, // Sample Period Register
        {0x13, 0x71}, // Sample Control Register
        {0x18, 0x22}, // Optimize AGC
        {0x19, 0x22}, // Automatic Gain Control
        {0x60, 0x01}, // Interrupt Control
        {0x90, 0x0F}, // Driver Range
        {0x91, 0xFF}, // Emitter DAC
    };

    int num_entries = sizeof(init_mappings) / sizeof(init_mappings[0]);
    for (int i = 0; i < num_entries; ++i) {
        wbytes[0] = init_mappings[i][0];
        wbytes[1] = init_mappings[i][1];
        ISL29501_p->write_transaction(dev_addr, wbytes, 2, 0);

        easy_read_transaction(ISL29501_p, dev_addr, wbytes[0], bytes, 1);
        uart.disp("Value @ 0x");
        uart.disp(wbytes[0], 16);
        uart.disp(" : 0x");
        uart.disp(bytes[0], 16);
        uart.disp("\n\r");
    }
    uart.disp("----------------[END INITIALIZATION]----------------\n\r");
}


/**
 * Reads calibration data from EEPROM and writes it to the DSP.
 *
 * @param ISL29501_p Pointer to the I2C core instance.
 * @param eeprom_addr I2C device address of the EEPROM.
 * @param dsp_addr I2C device address of the DSP.
 */
void read_eeprom_calibration(I2cCore *ISL29501_p, uint8_t eeprom_addr, uint8_t dsp_addr) {
    uint8_t read_start_address = 0x20 + 1; //Magic number at 0x20 (only used for alignment shift up by 1), see datasheet...
    uint8_t write_start_address = 0x24;
    uint8_t num_addresses = 13;
    uint8_t values[13];
    uint8_t wbytes[2], bytes[1];

    uart.disp("\r\n-----[COPYING CALIBRATION FROM EEPROM]-----\r\n");
    //Read from EEPROM starting at address 0x21 to (0x21 + 13 - 1)
    for (uint8_t i = 0; i < num_addresses; ++i) {
        easy_read_transaction(ISL29501_p, eeprom_addr, read_start_address + i, bytes, 1);
        values[i] = bytes[0];
    }
    
    //Write values read from EEPROM to DSP starting at address 0x24 to 0x30. (13 values...)
    for (uint8_t i = 0; i < num_addresses; ++i) {
        wbytes[0] = write_start_address + i;
        wbytes[1] = values[i];
        ISL29501_p->write_transaction(dsp_addr, wbytes, 2, 0);
    }
}

/**
 * Initializes the ISL29501 DSP by performing a factory reset,
 * loading EEPROM calibration data, and applying recommended values.
 *
 * @param ISL29501_p Pointer to the I2C core instance.
 * @param dsp_addr I2C device address of the DSP.
 * @param eeprom_addr I2C device address of the EEPROM.
 */
void ISL29501_initialize(I2cCore *ISL29501_p, uint8_t dsp_addr, uint8_t eeprom_addr) {
    uint8_t wbytes[2], bytes[1];

    // Factory reset (Write 0xD7 to 0xB0 according to the data sheet...)
    wbytes[0] = 0xB0;
    wbytes[1] = 0xD7;
    ISL29501_p->write_transaction(dsp_addr, wbytes, 2, 0);

    // Reading contents of omboard EEPROM to the DSP...
    read_eeprom_calibration(ISL29501_p, eeprom_addr, dsp_addr);
    
    // Writing digilent recommended DSP configs...
    write_digilent_values(ISL29501_p, dsp_addr);

    // Display Device ID
    easy_read_transaction(ISL29501_p, dsp_addr, 0x00, bytes, 1);
    uart.disp("Device ID: 0x");
    uart.disp(bytes[0], 16);
    uart.disp("\n\r");

    
}

/**
 * Reads the distance from the ISL29501 DSP in meters, centimeters, and inches.
 *
 * @param ISL29501_p Pointer to the I2C core instance.
 * @param dsp_addr I2C device address of the DSP.
 */
double ISL29501_read_distance(I2cCore *ISL29501_p, uint8_t dsp_addr) {
    uint8_t wbytes[2], bytes[1];
    uint16_t distanceMSB, distanceLSB;
    double distance;

    //Simulate a "SAMPLE START" as per the datasheet...
    wbytes[0] = 0xB0;
    wbytes[1] = 0x49; 
    ISL29501_p->write_transaction(dsp_addr, wbytes, 2, 0);

    //Read 16 bit distance registers at 0xD1 and 0xD2...
    easy_read_transaction(ISL29501_p, dsp_addr, 0xD1, bytes, 1);
    distanceMSB = bytes[0];
    easy_read_transaction(ISL29501_p, dsp_addr, 0xD2, bytes, 1);
    distanceLSB = bytes[0];

    uart.disp("[");
    uart.disp(distanceMSB);
    uart.disp(",");
    uart.disp(distanceLSB);
    uart.disp("] ");
    //Calculate distance according to datasheet...
    return distance = ((double)(distanceMSB * 256 + distanceLSB) / 65536) * 33.31;

}

void double_to_sseg(SsegCore *sseg, double adc_voltage)
{
    // Turn off unneeded SSeg displays (positions 0–3)
    for (int i = 3; i >= 0; --i)
        sseg->write_1ptn(0xff, i); // Active LOW

    // Set the decimal point: Place it between the 6th and 7th digits
    sseg->set_dp(0b01000000);  // Binary representation for decimal point at position 6

    // Extract integer and fractional parts of adc_voltage
    int integer_part = (int)adc_voltage; // Whole number part
    int fractional_part = (int)((adc_voltage - integer_part) * 1000); // First three decimal places

    // Display integer part (7th and 6th positions)
    uint8_t upper_disp_val = sseg->h2s(integer_part / 10); // Tens place of integer part
    sseg->write_1ptn(upper_disp_val, 7);

    uint8_t lower_disp_val = sseg->h2s(integer_part % 10); // Ones place of integer part
    sseg->write_1ptn(lower_disp_val, 6);

    // Display fractional part (5th, 4th, and 3rd positions)
    uint8_t frac_first = sseg->h2s((fractional_part / 100) % 10); // Tenths place
    sseg->write_1ptn(frac_first, 5);

    uint8_t frac_second = sseg->h2s((fractional_part / 10) % 10); // Hundredths place
    sseg->write_1ptn(frac_second, 4);
}

void print_distance(double distance) {
    double distance_cm = distance * 100;     //Calculate cm...
    double distance_in = distance * 39.3701; //Calculate in...

    // Use colors for output
    uart.disp(GREEN);
    uart.disp("Distance:");
    uart.disp(RESET);
    uart.disp(" ");
    uart.disp(distance, 10);
    uart.disp(" ");
    uart.disp(BLUE);
    uart.disp("m");
    uart.disp(RESET);
    uart.disp(", ");
    uart.disp(distance_cm, 10);
    uart.disp(" ");
    uart.disp(YELLOW);
    uart.disp("cm");
    uart.disp(RESET);
    uart.disp(", ");
    uart.disp(distance_in, 10);
    uart.disp(" ");
    uart.disp(RED);
    uart.disp("in");
    uart.disp(RESET);
    uart.disp("\n\r");
}

I2cCore ISL29501(get_slot_addr(BRIDGE_BASE, S4_USER));
SsegCore sseg(get_slot_addr(BRIDGE_BASE, S8_SSEG));

int main() {

    ISL29501_initialize(&ISL29501, dev_PMOD_RENESAS_DSP, dev_PMOD_EEPROM);
    SsegCore sseg(get_slot_addr(BRIDGE_BASE, S8_SSEG));

    /*We are running in single-shot mode, acquisition is handled by microcontroller, 
      DSP is not continuously unless CPU tells it to...*/
    while (1) {
        double distance = ISL29501_read_distance(&ISL29501, dev_PMOD_RENESAS_DSP);
        print_distance(distance);
        double_to_sseg(&sseg, distance);
    }


    
}
