
#pragma once

#include <stdint.h>

#include "crsf.h"
#include "dsm.h"
#include "ghst.hpp"
#include "sbus.h"
#include "st24.h"
#include "sumd.h"

uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a);
uint8_t crc8_dvb_s2_buf(uint8_t *buf, int len);
