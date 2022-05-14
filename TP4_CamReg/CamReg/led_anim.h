#ifndef LEDANIM_H
#define LEDANIM_H

typedef enum {
	NO_LEDS,
	TURN_CW,
	TURN_CCW,
	ALL_ON,
	ALL_ON_COLOR,
	BLINK,
}LED_STATES_t;

void led_anim_start(void);
void set_led_state (LED_STATES_t);
void set_led_color(color_rgb_n_t);
#endif /* PROCESS_IMAGE_H */
