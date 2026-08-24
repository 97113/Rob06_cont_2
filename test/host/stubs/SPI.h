#pragma once
#include <stdint.h>
struct SPIClass { void begin(int, int, int, int) {} };
extern SPIClass SPI;
