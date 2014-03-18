/**
 * \file
 *
 * \brief User board initialization template
 *
 */

#include <board.h>
#include <conf_board.h>
#include <gpio.h>

void board_init(void)
{

  #ifdef LED0
  gpio_configure_pin(LED0,GPIO_DIR_OUTPUT);
  #endif

}
