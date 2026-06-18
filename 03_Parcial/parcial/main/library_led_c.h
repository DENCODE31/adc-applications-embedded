#ifndef LIBRARY_LED_C_H
#define LIBRARY_LED_C_H

#include <stdint.h>         /* Permite usar tipos enteros como uint32_t. */

#include "driver/gpio.h"   /* Permite usar gpio_num_t para guardar pines GPIO. */
#include "driver/ledc.h"   /* Permite usar tipos LEDC para PWM: timer, canal y modo. */

/* ========================================================================== */
/*                         Estructura de un color LED                         */
/* ========================================================================== */

typedef struct {
    uint32_t duty;              /* Valor PWM aplicado al color del LED. */
    gpio_num_t gpio_num;        /* Pin GPIO donde esta conectado ese color. */
    ledc_channel_t channel;     /* Canal LEDC usado para generar PWM. */
} led_t;

/* ========================================================================== */
/*                         Estructura completa LED RGB                        */
/* ========================================================================== */

typedef struct {
    led_t led_red;                      /* Configuracion del color rojo. */
    led_t led_green;                    /* Configuracion del color verde. */
    led_t led_blue;                     /* Configuracion del color azul. */
    ledc_timer_t timer;                 /* Timer LEDC compartido por los canales. */
    ledc_timer_bit_t duty_resolution;   /* Resolucion del PWM en bits. */
    uint32_t frequency;                 /* Frecuencia PWM en Hz. */
    ledc_mode_t speed_mode;             /* Modo de velocidad del periferico LEDC. */
} led_rgb_t;

/* ========================================================================== */
/*                         Prototipos de la libreria                          */
/* ========================================================================== */

void config_led_rgb(led_rgb_t *led_rgb);
void set_led_rgb_given_struct(led_rgb_t *led_rgb);
void set_led_rgb_given_values(led_rgb_t *led_rgb, uint32_t duty_red, uint32_t duty_green, uint32_t duty_blue);
void set_led_rgb_percentage_given_values(led_rgb_t *led_rgb, int percentage_red, int percentage_green, int percentage_blue);

#endif /* LIBRARY_LED_C_H */
