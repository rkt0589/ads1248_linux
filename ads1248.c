/* Program to test ADS1248 with Colibri VFxx SPI 1*/

#include "ads1248.h"
//#define DEBUG 1

static const char *device = "/dev/spidev1.0";
static uint32_t mode = 1;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay;
uint32_t drdy_pin = -1;
int fd = 0;

static int openSPI()
{
  int ret = 0;
  fd = open(device, O_RDWR);
  if (fd < 0){
    printf("can't open device");
    return -1;
  }
  //spi mode
  ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
  if (ret == -1){
  printf("can't set spi mode");
  return ret;
  }
  ret = ioctl(fd, SPI_IOC_RD_MODE32, &mode);
  if (ret == -1){
    printf("can't get spi mode");
    return ret;
  }
  // bits per word
  ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
  if (ret == -1){
    printf("can't set bits per word");
    return ret;
  }
  ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
  if (ret == -1){
    printf("can't get bits per word");
    return ret;
  }
  // max speed hz
  ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
  if (ret == -1){
    printf("can't set max speed hz");
    return ret;
  }
  ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
  if (ret == -1){
    printf("can't get max speed hz");
    return ret;
  }
  #ifdef DEBUG
  printf("spi mode: 0x%x\n", mode);
  printf("bits per word: %d\n", bits);
  printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
  #endif
  return ret;
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

static void closeSPI()
{
  close(fd);
}

static void gpioExport(int gpio)
{
    int fd;
    char buf[255];
    fd = open("/sys/class/gpio/export", O_WRONLY);
    sprintf(buf, "%d", gpio);
    write(fd, buf, strlen(buf));
    close(fd);
}

static void gpioDirection(int gpio, int direction) // 1 for output, 0 for input
{
    int fd;
    char buf[255];
    sprintf(buf, "/sys/class/gpio/gpio%d/direction", gpio);
    fd = open(buf, O_WRONLY);
    if (direction)
    {
        write(fd, "out", 3);
    }
    else
    {
        write(fd, "in", 2);
    }
    close(fd);
}

static int readGpio(int gpio)
{
  char value;
  int fd;
  char buf[255];
  sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
  fd = open(buf, O_RDONLY);
  read(fd, &value, 1);
  close(fd);
  return (int)value;
}

static int GetADS1148Drdy()
{
  if(drdy_pin == -1)
  return -1;
  gpioExport(drdy_pin);
  gpioDirection(drdy_pin, 0);// 1 for output 0 for input
  return readGpio(drdy_pin);
}

int ads1248Init(int drdy)
{
  if(openSPI() == -1)
  {
    closeSPI();
    printf("Error in adsInit");
    return -1;
  }
  if(drdy != -1)
  {
    drdy_pin = drdy;
  }
  usleep(100);
  reset();
  usleep(100000);
  return 0;
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

int writeReg(uint8_t regAddress, uint8_t regValue)
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
    return -1;
  }
  #ifdef DEBUG
  printf("Register Write %x = %x\n", regAddress, regValue);
  #endif
  if(regValue != readReg(regAddress))
  {
    printf("Write Register %x value failed, Value read is %x\n", regAddress, readReg(regAddress));
  }
  return 0;
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

int readAdc()
{
  int adcValue = 0;
	uint8_t tx[5] = {0};
	uint8_t rx[5] = {0};
	tx[0] = RDATA;
	tx[1] = NOP;
	tx[2] = NOP;
	tx[3] = NOP;
  while((GetADS1148Drdy() == 1) | (GetADS1148Drdy() == -1)){}
	if(transfer(tx, rx, 4) < 0)
	{
		printf("Error in selfOffsetCal");
	}
  adcValue = ((rx[1]<<16)+(rx[2]<<8)+rx[3]);
  int value = (adcValue & 0x7FFFFF);
  int sign = (adcValue & 0x800000);
  if (sign)
  {
    value *= -1;
  }
  #ifdef DEBUG
	printf("Value %d\n", adcValue);
  #endif
  return value;
}
