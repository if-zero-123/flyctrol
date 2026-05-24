#ifndef APP_LED_H
#define APP_LED_H

#include <stdbool.h>
#include "flight_types.h"

typedef enum {
  LED_GREEN = 0,
  LED_BLUE = 1
} led_id_t;

void Led_Init(void);
void Led_Set(led_id_t led, bool on);
void Led_Toggle(led_id_t led);
void Led_UpdateStatus(const flight_status_t *status);

#endif /* APP_LED_H */
