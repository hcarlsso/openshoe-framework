/**
 * \file
 *
 * \brief User board definition template
 *
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

// Board clock oscillator
#define BOARD_OSC0_HZ           16000000
#define BOARD_OSC0_STARTUP_US   2000
#define BOARD_OSC0_IS_XTAL      true

#if defined(MIMU3333)
#  include "MIMU3333.h"
#elif defined(MIMU4444)
#  include "MIMU4444.h"
#elif defined(MIMU22BT)
#  include "MIMU22BT.h"
#elif defined(MIMU4444BT)
#  include "MIMU4444BT.h"
#elif defined(MIMU22BTv3)
#  include "MIMU22BTv3.h"
#else
#  error No known board specified
#endif

#endif // USER_BOARD_H
