#ifndef ads1248_h
#define ads1248_h

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define DRDY_VALUE "/sys/class/gpio/gpio53/value"
#define DRDY_DIRECTION "/sys/class/gpio/gpio53/direction"
// Register list of ADS1248
enum reg
{
  MUX0    = 0x00, //Multiplexer Control Register 0
  VBIAS   = 0x01, //Bias Voltage Register
  MUX1    = 0x02, //Multiplexer Control REgister 1
  SYS0    = 0x03, //System Control Register 0
  OFC0    = 0x04, //Offset Calibration Coefficient Register 0
  OFC1    = 0x05, //Offset Calibration Coefficient Register 1
  OFC2    = 0x06, //Offset Calibration Coefficient Register 2
  FSC0    = 0x07, //Full scale Callibration Coefficient Register 0
  FSC1    = 0x08, //Full scale Callibration Coefficient Register 1
  FSC2    = 0x09, //Full scale Callibration Coefficient REgister 2
  IDAC0   = 0x0A, //IDAC Control Register 0
  IDAC1   = 0x0B, //IDAC Control Register 1
  GPIOCFG = 0x0C, //GPIO Configuration Register
  GPIODIR = 0x0D, //GPIODirection REgister
  GPIODAT = 0x0E  //GPIO Data Register
};
// commande list of ADS1248
enum cmd
{
  WAKEUP  = 0x00, //Exit Sleep Mode
  SLEEP   = 0x02, //Enter Sleep Mode
  SYNC    = 0x04, //Synchornize the A/D Conversion
  RESET   = 0x06, //Reset To Power UP values
  NOP     = 0xff, //NO Operation
  RDATA   = 0x12, //Read data once
  RDATAC  = 0x14, //Read data continously
  SDATAC  = 0x16, //Stop reading data continously
  RREG    = 0x20, //Read From Register
  WREG    = 0x40, //Write To Register
  SYSOCAL = 0x60, //System Offset Calibration
  SYSGCAL = 0x61, //System Gain Calibration
  SELFOCAL= 0x62  //Self Offset Calibration
};
// MUX control Register 0
enum MUX0
{
  MUXSN_AIN0 = (0 << 0), // Negative Input Channel AIN0
  MUXSN_AIN1 = (1 << 0), // Negative Input Channel AIN1 (default)
  MUXSN_AIN2 = (2 << 0), // Negative Input Channel AIN2
  MUXSN_AIN3 = (3 << 0), // Negative Input Channel AIN3
  MUXSN_AIN4 = (4 << 0), // Negative Input Channel AIN4
  MUXSN_AIN5 = (5 << 0), // Negative Input Channel AIN5
  MUXSN_AIN6 = (6 << 0), // Negative Input Channel AIN6
  MUXSN_AIN7 = (7 << 0), // Negative Input Channel AIN7
  MUXSP_AIN0 = (0 << 3), // Positive Input Channel AIN0 (default)
  MUXSP_AIN1 = (1 << 3), // Positive Input Channel AIN1
  MUXSP_AIN2 = (2 << 3), // Positive Input Channel AIN2
  MUXSP_AIN3 = (3 << 3), // Positive Input Channel AIN3
  MUXSP_AIN4 = (4 << 3), // Positive Input Channel AIN4
  MUXSP_AIN5 = (5 << 3), // Positive Input Channel AIN5
  MUXSP_AIN6 = (6 << 3), // Positive Input Channel AIN6
  MUXSP_AIN7 = (7 << 3), // Positive Input Channel AIN7
  BCS_OFF    = (0 << 6), // Burnout current source Off (default)
  BCS_500NA  = (1 << 6), // Burnout current source 0.5uA (500nA)
  BCS_2UA    = (2 << 6), // Burnout current source 2uA
  BCS_10UA   = (3 << 6)  // Burnout current source 10uA
};
// Bias Voltage Register
enum VBIAS
{
  OFF  = (0x00),   // Bias Voltage not enabled (default)
  AIN0 = (1 << 0), // Bias Voltage applied to AIN0
  AIN1 = (1 << 1), // Bias Voltage applied to AIN1
  AIN2 = (1 << 2), // Bias Voltage applied to AIN2
  AIN3 = (1 << 3), // Bias Voltage applied to AIN3
  AIN4 = (1 << 4), // Bias Voltage applied to AIN4
  AIN5 = (1 << 5), // Bias Voltage applied to AIN5
  AIN6 = (1 << 6), // Bias Voltage applied to AIN6
  AIN7 = (1 << 7)  // Bias Voltage applied to AIN7
};
// Multiplexer Control Register 1
enum MUX1
{
  MUXCAL_NORMAL = (0 << 0), // Normal operation, PGA gain set by SYS0 (default)
  MUXCAL_OFFSET = (1 << 0), // Offset measurement, PGA gain set by SYS0
  MUXCAL_GAIN   = (2 << 0), // Gain measurement, PGA gain 1
  MUXCAL_TEMP   = (3 << 0), // Temperature measurement, PGA gain 1
  MUXCAL_REF1   = (4 << 0), // Ext REF1 measurement, PGA gain 1
  MUXCAL_REF0   = (5 << 0), // Ext REF0 measurement, PGA gain 1
  MUXCAL_AVDD   = (6 << 0), // AVDD measurement, PGA gain 1
  MUXCAL_DVDD   = (7 << 0), // DVDD measurement, PGA gain 1
  REFSELT_REF0  = (0 << 3), // REF0 input pair selected (default)
  REFSELT_REF1  = (1 << 3), // REF1 input pair selected
  REFSELT_REFOB = (2 << 3), // Onboard reference selected
  REFSELT_REF0OB    = (3 << 3), // Onboard reference selected and internally connected to REF0 input pair
  VREFCON_INTREFOFF = (0 << 5), // Internal reference is always Off (default)
  VREFCON_INTREFON  = (1 << 5), // Internal reference is always On
  VREFCON_INTREFONC = (2 << 5), // Internal reference is On when a conversion is in progress
  CLKSTAT_INTOSC = (0 << 7), // Internal oscillator (read-only)
  CLKSTAT_EXTOSC = (0 << 7)  // External oscillator (read-only)
};
// System Control Register 0
enum SYS0
{
  DOR_5SPS   = (0 << 0), // Data Output Rate 5 SPS (default)
  DOR_10SPS  = (1 << 0), // Data Output Rate 10 SPS
  DOR_20SPS  = (2 << 0), // Data Output Rate 20 SPS
  DOR_40SPS  = (3 << 0), // Data Output Rate 40 SPS
  DOR_80SPS  = (4 << 0), // Data Output Rate 80 SPS
  DOR_160SPS = (5 << 0), // Data Output Rate 160 SPS
  DOR_320SPS = (6 << 0), // Data Output Rate 320 SPS
  DOR_640SPS = (7 << 0), // Data Output Rate 640 SPS
  DOR_1000SPS = (8 << 0),// Data Output Rate 1000 SPS
  DOR_2000SPS = (9 << 0),// Data Output Rate 2000 SPS
  PGA_1 = (0 << 4),      // Gain 1 (default)
  PGA_2 = (1 << 4),      // Gain 2
  PGA_4 = (2 << 4),      // Gain 4
  PGA_8 = (3 << 4),      // Gain 8
  PGA_16 = (4 << 4),     // Gain 16
  PGA_32 = (5 << 4),     // Gain 32
  PGA_64 = (6 << 4),     // Gain 64
  PGA_128 = (7 << 4)     // Gain 128
};
// IDAC Control Register 0
enum IDAC0
{
  IMAG_OFF    = (0 << 0), // Excitation Current Magnitude Off (default)
  IMAG_50UA   = (1 << 0), // Excitation Current Magnitude 50uA
  IMAG_100UA  = (2 << 0), // Excitation Current Magnitude 100uA
  IMAG_250UA  = (3 << 0), // Excitation Current Magnitude 250uA
  IMAG_500UA  = (4 << 0), // Excitation Current Magnitude 500uA
  IMAG_750UA  = (5 << 0), // Excitation Current Magnitude 750uA
  IMAG_1000UA = (6 << 0), // Excitation Current Magnitude 1000uA
  IMAG_1500UA = (7 << 0), // Excitation Current Magnitude 1000uA
  DRDYMODE_DOUT = (0 << 3), // DOUT/DRDY pin function Data Out (default)
  DRDYMODE_DOUTDRDY = (1 << 3) // DOUT/DRDY pin function Data Out and Data Ready (active low)
};
// IDAC Control Register 1
enum IDAC1
{
  I2DIR_AIN0  = (0 << 0), // Second Current Source to AIN0
  I2DIR_AIN1  = (1 << 0), // Second Current Source to AIN1
  I2DIR_AIN2  = (2 << 0), // Second Current Source to AIN2
  I2DIR_AIN3  =  (3 << 0),// Second Current Source to AIN3
  I2DIR_AIN4  = (4 << 0), // Second Current Source to AIN4
  I2DIR_AIN5  = (5 << 0), // Second Current Source to AIN5
  I2DIR_AIN6  = (6 << 0), // Second Current Source to AIN6
  I2DIR_AIN7  = (7 << 0), // Second Current Source to AIN7
  I2DIR_IEXT1 = (8 << 0), // Second Current Source to IEXT1 Pin
  I2DIR_IEXT2 = (9 << 0), // Second Current Source to IEXT2 Pin
  I2DIR_OFF   = (15 << 0),// Second Current Source Disconnected (default)
  I1DIR_AIN0  = (0 << 4), // First Current Source to AIN0
  I1DIR_AIN1  = (1 << 4), // First Current Source to AIN1
  I1DIR_AIN2  = (2 << 4), // First Current Source to AIN2
  I1DIR_AIN3  = (3 << 4), // First Current Source to AIN3
  I1DIR_AIN4  = (4 << 4), // First Current Source to AIN4
  I1DIR_AIN5  = (5 << 4), // First Current Source to AIN5
  I1DIR_AIN6  = (6 << 4), // First Current Source to AIN6
  I1DIR_AIN7  = (7 << 4), // First Current Source to AIN7
  I1DIRIEXT1  = (8 << 4), // First Current Source to IEXT1 pin
  I1DIRIEXT2  = (9 << 4), // First Current Source to IEXT1 pin
  I1DIR_OFF   = (15 << 4) // First Current Source Disconnected (default)
};
// GPIO Configuration Register.
enum GPIOCFG
{
  IOCFG_GPIO0 = (1 << 0), // GPIO0 shared with REFP0
  IOCFG_GPIO1 = (1 << 1), // GPIO1 shared with REFN0
  IOCFG_GPIO2 = (1 << 2), // GPIO2 shared with AIN2
  IOCFG_GPIO3 = (1 << 3), // GPIO3 shared with AIN3
  IOCFG_GPIO4 = (1 << 4), // GPIO4 shared with AIN4
  IOCFG_GPIO5 = (1 << 5), // GPIO5 shared with AIN5
  IOCFG_GPIO6 = (1 << 6), // GPIO6 shared with AIN6
  IOCFG_GPIO7 = (1 << 7)  // GPIO7 shared with AIN7
};
// GPIO Direction Register.
// By Default GPIO direction is output
enum GPIODIR
{
  IODIR_IN0 = (1 << 0), // GPIO0 shared with REFP0
  IODIR_IN1 = (1 << 1), // GPIO1 shared with REFN0
  IODIR_IN2 = (1 << 2), // GPIO2 shared with AIN2
  IODIR_IN3 = (1 << 3), // GPIO3 shared with AIN3
  IODIR_IN4 = (1 << 4), // GPIO4 shared with AIN4
  IODIR_IN5 = (1 << 5), // GPIO5 shared with AIN5
  IODIR_IN6 = (1 << 6), // GPIO6 shared with AIN6
  IODIR_IN7 = (1 << 7)  // GPIO7 shared with AIN7
};
// GPIO Data Register.
enum GPIODAT
{
  IODAT0 = (1 << 0), // GPIO0 shared with REFP0
  IODAT1 = (1 << 1), // GPIO1 shared with REFN0
  IODAT2 = (1 << 2), // GPIO2 shared with AIN2
  IODAT3 = (1 << 3), // GPIO3 shared with AIN3
  IODAT4 = (1 << 4), // GPIO4 shared with AIN4
  IODAT5 = (1 << 5), // GPIO5 shared with AIN5
  IODAT6 = (1 << 6), // GPIO6 shared with AIN6
  IODAT7 = (1 << 7), // GPIO7 shared with AIN7
};
// Init Serial communication and reset ads1248 chip
// @param DREDY gpio pin pass -1 if not used
// @return 0 on success -1 on failed
int ads1248Init(int drdy_pin);
// Synchronisation cmd of ADS1248
void sync(void);
// Reset cmd of ADS1248
void reset(void);
// Read ADS1248 register
// @param reg is register address
// @return register value
uint8_t readReg(uint8_t regAddress);
// Write ADS1248 register
// @param reg is register address
// @param val value to write
// @return 0 on success -1 on failed
int writeReg(uint8_t regAddress, uint8_t regValue);
// Read 24 bit data
// @return 24 bit value
int readAdc();
// System Offset Calibration cmd of ADS1248
void systemOffsetCal(void);
// System Gain Calibration cmd of ADS1248
void systemGainCal(void);
// Self Offset Calibration cmd of ADS1248 (ask waitReady)
void selfOffsetCal(void);
#endif
