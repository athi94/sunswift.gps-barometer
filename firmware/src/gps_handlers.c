#include <arch/gpio.h>

#include <scandal/utils.h>
#include <scandal/message.h>
#include <scandal/leds.h>

#include <project/target_config.h>
#include <project/leds_annexure.h>
#include <project/conversion.h>


void gps_timer_handler() {
    toggle_yellow_led();
    GPIO_IntClear(2, 10);
}
