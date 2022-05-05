#ifndef SPI__H__
#define SPI__H__

#include <xc.h>
#include <sys/attribs.h>

void initSPI();
unsigned char spi_io(unsigned char o);

#endif // SPI__H__