/* Program to test ADS1248 with Colibri VFxx SPI 1*/
#include "ads1248.h"

int readInternalTemp()
{
  writeReg(MUX1, (uint8_t)(VREFCON_INTREFON | REFSELT_REFOB | MUXCAL_TEMP));
  int uV = ((readAdc() * 204800)/0x7fffff);
  int uVOffset = uV - 118000;
  int offsetDegrees = (uVOffset*10)/405;
  int temp = 250 + offsetDegrees;
  printf("Ambient Temperature %d\n", temp);
  return temp;
}

void setGpioConfiguration()
{
  writeReg(GPIOCFG, (uint8_t)(IOCFG_GPIO4 | IOCFG_GPIO5 | IOCFG_GPIO6));
  writeReg(GPIODIR, 0x00);
}

void selectChannel(uint8_t channel)
{
  setGpioConfiguration();
  writeReg(GPIODAT, (channel << 4));
  debug_dump();
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
  ads1248Init(53);
  while(1)
  {
    readInternalTemp();
    sleep(1);
  }
  printf("Channel One Muxed %d\n", readAdc());
  printf("Program End Here\n");
  return ret;
}
