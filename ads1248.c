/* Program to test ADS1248 with Colibri VFxx SPI 1*/

#include "ads1248.h"
#define ADS1248_HOLD_TIME_US        2
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
static void pabort(const char *s)
{
  perror(s);
  abort();
}
// #define DEBUG 0
static const char *device = "/dev/spidev1.0";
static uint32_t mode = 1;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay;
int fd = 0;

static void openSPI()
{
  int ret = 0;
  fd = open(device, O_RDWR);
  if (fd < 0)
  pabort("can't open device");
  /*
  * spi mode
  */
  ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
  if (ret == -1)
  pabort("can't set spi mode");

  ret = ioctl(fd, SPI_IOC_RD_MODE32, &mode);
  if (ret == -1)
  pabort("can't get spi mode");
  /*
  * bits per word
  */
  ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
  if (ret == -1)
  pabort("can't set bits per word");

  ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
  if (ret == -1)
  pabort("can't get bits per word");
  /*
  * max speed hz
  */
  ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
  if (ret == -1)
  pabort("can't set max speed hz");
  ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
  if (ret == -1)
  pabort("can't get max speed hz");
  printf("spi mode: 0x%x\n", mode);
  printf("bits per word: %d\n", bits);
  printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
}

static int transfer(uint8_t  *tx, uint8_t  *rx, size_t len)
{
  int ret;
  struct spi_ioc_transfer spi;
  memset (&spi, 0, sizeof (spi)) ;
  spi.tx_buf = (unsigned long)tx;
  spi.rx_buf = (unsigned long)rx;
  spi.len = len;
  spi.delay_usecs = delay;
  spi.speed_hz = speed;
  spi.bits_per_word = bits;
  spi.cs_change = 0;
  ret = ioctl(fd, SPI_IOC_MESSAGE(1), &spi);
  return ret;
}

void sync()
{
  uint8_t tx[1] = {0};
  uint8_t rx[1] = {0};
  tx[0] = SYNC;
  if(transfer(tx, rx, 1) < 0)
  {
    printf("Error in sync");
  }
}

void reset()
{
  uint8_t tx[1] = {0};
  uint8_t rx[1] = {0};
  tx[0] = RESET;
  if(transfer(tx, rx, 1) < 0)
  {
    printf("Error in reset");
  }
  usleep(1000); // 1ms delay
}

uint8_t readReg(uint8_t regAddress)
{
  uint8_t tx[3] = {0};
  uint8_t rx[3] = {0};
  tx[0] = (RREG|(regAddress&0x0F));
  tx[1] = 0x01;
  tx[2] = NOP;
  if(transfer(tx, rx, 3) < 0)
  {
    printf("Error in readReg");
  }
  #ifdef DEBUG
  printf("Register Read %x = %x\n", regAddress, rx[2]);
  #endif
  return rx[2];
}

void writeReg(uint8_t regAddress, uint8_t regValue)
{
  uint8_t tx[4] = {0};
  uint8_t rx[4] = {0};
  tx[0] = (WREG|(regAddress&0x0F));
  tx[1] = 0x01;
  tx[2] = regValue;
  tx[3] = NOP;
  if(transfer(tx, rx, 4) < 0)
  {
    printf("Error in writeReg");
  }
  #ifdef DEBUG
  printf("Register Write %x = %x\n", regAddress, regValue);
  #endif
  if(regValue != readReg(regAddress))
  {
    printf("Write Register %x value failed, Value read is %x\n", regAddress, readReg(regAddress));
  }
}

void systemOffsetCal(void)
{
  uint8_t tx[1] = {0};
  uint8_t rx[1] = {0};
  tx[0] = SYSOCAL;
  if(transfer(tx, rx, 1) < 0)
  {
    printf("Error in systemOffsetCal");
  }
}

void systemGainCal(void)
{
  uint8_t tx[1] = {0};
  uint8_t rx[1] = {0};
  tx[0] = SYSGCAL;
  if(transfer(tx, rx, 1) < 0)
  {
    printf("Error in systemGainCal");
  }
}

void selfOffsetCal(void)
{
  uint8_t tx[1] = {0};
  uint8_t rx[1] = {0};
  tx[0] = SELFOCAL;
  if(transfer(tx, rx, 1) < 0)
  {
    printf("Error in selfOffsetCal");
  }
}

void adcInit()
{
	  writeReg(MUX0, 0b00000001); 	// MUX0:  Pos. input: AIN0, Neg. input: AIN1 (Burnout current source off)
    writeReg(MUX1, 0b00100000); 	// MUX1:  REF0, normal operation
    writeReg(SYS0, 0b00000000); 	// SYS0:  PGA Gain = 1, 5 SPS
    writeReg(IDAC0,0b00000000); 	// IDAC0: off
    writeReg(IDAC1,0b11001100); 	// IDAC1: n.c.
    writeReg(VBIAS,0b00000000); 	// VBIAS: BIAS voltage disabled
    writeReg(OFC0, 0b00000000); 	// OFC0:  0 => reset offset calibration
    writeReg(OFC1, 0b00000000); 	// OFC1:  0 => reset offset calibration
    writeReg(OFC2, 0b00000000); 	// OFC2:  0 => reset offset calibration
    writeReg(GPIOCFG, 0b00000000);  // GPIOCFG: all used as analog inputs
    writeReg(GPIODIR, 0b00000000);  // GPIODIR: -
    writeReg(GPIODAT, 0b00000000);  // GPIODAT: -
}

int readAdc()
{
  int adcValue = 0;
	uint8_t tx[5] = {0};
	uint8_t rx[5] = {0};
	tx[0] = RDATA;
	tx[1] = NOP;
	tx[2] = NOP;
	tx[3] = NOP;
	if(transfer(tx, rx, 4) < 0)
	{
		printf("Error in selfOffsetCal");
	}
  adcValue = ((rx[1]<<16)+(rx[2]<<8)+rx[3]);
  int value = (adcValue & 0x7FFFFF);
  int sign = (adcValue & 0x800000) == 0x800000;
  if (sign){
    value *= -1;
  }
  #ifdef DEBUG
	printf("Value %d\n", adcValue);
  #endif
  return value;
}

void readInternalTemp()
{
  writeReg(MUX1, (uint8_t)(VREFCON_INTREFON | REFSELT_REFOB | MUXCAL_TEMP));
  printf("Ambient Temperature %d\n", readAdc());
}

void setGpioConfiguration()
{
  writeReg(GPIOCFG, (uint8_t)(IOCFG_GPIO4 | IOCFG_GPIO5 | IOCFG_GPIO6));
  writeReg(GPIODIR, 0x00);
}
void testSPI()
{
	uint8_t reg_tx[] = {0x01, 0x02,
	};
	uint8_t rx[16] = {0};
	if(transfer(reg_tx, rx, 2) < 0)
	{
		printf("Error in selfOffsetCal");
	}

}

void debug_dump()
{
  printf("MUX0 = %x\n", readReg(MUX0));
  printf("VBIAS = %x\n", readReg(VBIAS));
  printf("MUX1 = %x\n", readReg(MUX1));
  printf("SYS0 = %x\n", readReg(SYS0));
  printf("OFC0 = %x\n", readReg(OFC0));
  printf("OFC1 = %x\n", readReg(OFC1));
  printf("OFC2 = %x\n", readReg(OFC2));
  printf("FSC0 = %x\n", readReg(FSC0));
  printf("FSC1 = %x\n", readReg(FSC1));
  printf("FSC2 = %x\n", readReg(FSC2));
  printf("IDAC0 = %x\n", readReg(IDAC0));
  printf("IDAC1 = %x\n", readReg(IDAC1));
  printf("GPIOCFG = %x\n", readReg(GPIOCFG));
  printf("GPIODIR = %x\n", readReg(GPIODIR));
  printf("GPIODAT = %x\n", readReg(GPIODAT));
}

int main(int argc, char *argv[])
{
  int ret = 0;
  openSPI();
  usleep(100);
  reset();
  usleep(100000);
  readInternalTemp();
  while(1)
  {
    readInternalTemp();
    sleep(1);
  }
  printf("Program End Here\n");
  close(fd);
  return ret;
}
