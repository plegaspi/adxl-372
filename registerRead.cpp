// Compiler command:  g++ registerRead.cpp ADXL372.cpp -lspidev-lib++ -lwiringPi -o test && ./test

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <chrono>
#include "ADXL372.hpp"

spi_config_t spi_config;
uint8_t tx_buffer[32];
uint8_t rx_buffer[32];

int cs_pin = 27;


int  main( void)
{

  SPI *arduinoSPI = NULL;
  ADXL372class *ADXL372 = NULL;

  spi_config.mode=0;
  spi_config.speed=100000; // Do not set higher than 10 MHz
  spi_config.delay=0;
  spi_config.bits_per_word=8;
  
  wiringPiSetupGpio();
  pinMode(cs_pin, OUTPUT);

  arduinoSPI=new SPI("/dev/spidev0.1",&spi_config);

  ADXL372 = new ADXL372class(arduinoSPI, cs_pin);

  if (arduinoSPI->begin())
  {
    // ADXL372 -> reset();
    ADXL372 -> printDevice();
    ADXL372 -> setOperatingMode(STANDBY);
    ADXL372 -> setOdr(ODR_6400Hz);
    ADXL372 -> setBandwidth(BW_3200Hz);
    ADXL372 -> setOperatingMode(FULL_BANDWIDTH);
    
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    while (true) {
      ADXL372 -> readAcceleration(x, y, z);
      auto finish = std::chrono::high_resolution_clock::now();
      auto time_in_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(finish.time_since_epoch()).count();
      std::cout << std::dec << time_in_ns << " " << x << " " << y << " " << z << std::endl;
    }
    arduinoSPI -> end();
  } else {
    std::cout << "Failed to initiate communication via SPI" << std::endl;
  }
 return 1;
}