// Rapsberry Pi: SPI in C - Versione 0.5 - Luglio 2013
// Copyright (c) 2013, Vincenzo Villa (https://www.vincenzov.net)
// Creative Commons | Attribuzione-Condividi allo stesso modo 3.0 Unported.
// Creative Commons | Attribution-Share Alike 3.0 Unported
// https://www.vincenzov.net/tutorial/elettronica-di-base/RaspberryPi/spi-c.htm

// Compile:  gcc dac.c -std=c99 -o dac
// Run as user with R&W right on /dev/spidev0.* (NOT ROOT!)
// vv@vvrpi ~ $ ./dac 1.23 0.12 
// Using MCP4822 - Output dual voltage 

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define SPI_SPEED 2100000                       // SPI frequency clock
#define VREF 2.048				// MCP4822 voltage reference [V]

static const char *device = "/dev/spidev0.1";	// MCP4822 CD to RPi CS1
static uint8_t mode = SPI_MODE_0;		// SPI_MODE_0 

static void exit_on_error (const char *s)	// Exit and print error code
{ 	perror(s);
  	abort();
} 

int main(int argc, char *argv[])
{
	int fd;
	int dataA, dataB;
	double voltA, voltB;
	
      	uint16_t tx;    	// RX buffer (16 bit unsigned integer)
                                
      	struct spi_ioc_transfer tr = 
      	{	.tx_buf = (unsigned long)&tx,         
          	.rx_buf = (unsigned long)NULL,        
          	.len = 2,             
          	.delay_usecs = 0,
          	.speed_hz = SPI_SPEED,   
          	.bits_per_word = 8,
          	.cs_change = 0,
       	};

        printf("Rapsberry Pi: SPI in C - Versione 0.5 - Luglio 2013\n");
        printf("Copyright (c) 2013, Vincenzo Villa (https://www.vincenzov.net)\n");
        printf("Creative Commons | Attribuzione-Condividi allo stesso modo 3.0 Unported.\n");
        printf("Creative Commons | Attribution-Share Alike 3.0 Unported\n");
        printf("https://www.vincenzov.net/tutorial/elettronica-di-base/RaspberryPi/spi-c.htm\n\n");                                       

        if (argc != 3) 
        { 
          printf ( "Usage: \n rpi ~ $ ./dac VA VB\n Vx is a float number from 0 to 2.048\n\n");
          exit(-1);        
         }
         
        voltA = atof(argv[1]);
        voltB = atof(argv[2]);
        
        if ((voltA > VREF) || (voltA < 0))
        { 
          printf ( "VA out of range (0 -> %f)\n\n", VREF);
          exit(-2);        
         }
                                                      
        if ((voltB > VREF) || (voltB < 0))
        {
          printf ( "VB out of range (0 -> %f)\n\n", VREF);
          exit(-3);
        }
        
       	// Open SPI device
       	if ((fd = open(device, O_RDWR)) < 0) exit_on_error ("Can't open SPI device");

	// Set SPI mode
	if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) exit_on_error ("Can't set SPI mode");

	dataB = voltB / VREF * 4096; 	// Create data to send (see textand data sheet)
        dataB = dataB | 0xF000;	
        
	tx = (dataB << 8) | (dataB >> 8);	// Adjust bits
        
       	// Write data
       	if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) exit_on_error ("Can't send SPI message");
           
        dataA = voltA / VREF * 4096;
        dataA = (dataA & 0x0FFF) | 0x3000;
        
        tx = (dataA << 8) | (dataA >> 8);                     
                            
        // Write data
        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) exit_on_error ("Can't send SPI message");
                  
        printf("Sending data to MCP4822 on %s\n\n", device);
        
        printf("Voltage A: %f V - Hex value: 0x%X \n\n", voltA, dataA );
        printf("Voltage B: %f V - Hex value: 0x%X \n\n", voltB, dataB );
                                  
	close(fd);

	return (0);
}