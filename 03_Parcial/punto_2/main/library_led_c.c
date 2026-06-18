#include "library_led_c.h"    /* Incluye las estructuras y prototipos de esta libreria. */
#include "esp_err.h"          /* Incluye ESP_ERROR_CHECK para validar llamadas de ESP-IDF. */

/* ============================================================================
 *  1. Calculo auxiliar del duty maximo
 * ========================================================================== */

static uint32_t max_duty_for_resolution(ledc_timer_bit_t duty_resolution)
{
    return (1U << duty_resolution) - 1U;    /* Para N bits, el maximo es 2^N - 1. */
}

/* ============================================================================
 *  2. Configuracion inicial del LED RGB
 * ========================================================================== */

void config_led_rgb(led_rgb_t *led_rgb)
{
    ledc_timer_config_t ledc_timer = {              /* Estructura de configuracion del timer PWM. */
        .speed_mode = led_rgb->speed_mode,          /* Usa el modo LEDC guardado en la estructura. */
        .duty_resolution = led_rgb->duty_resolution,/* Define cuantos bits tendra el duty PWM. */
        .timer_num = led_rgb->timer,                /* Selecciona el timer LEDC que se va a usar. */
        .freq_hz = led_rgb->frequency,              /* Define la frecuencia de la senal PWM. */
        .clk_cfg = LEDC_AUTO_CLK                    /* Deja que ESP-IDF seleccione el reloj adecuado. */
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer)); /* Aplica la configuracion del timer. */

    ledc_channel_config_t ledc_channel_red = {       /* Estructura de configuracion del canal rojo. */
        .speed_mode = led_rgb->speed_mode,           /* Usa el mismo modo LEDC del timer. */
        .channel = led_rgb->led_red.channel,         /* Canal PWM asignado al LED rojo. */
        .timer_sel = led_rgb->timer,                 /* Timer PWM asociado a este canal. */
        .intr_type = LEDC_INTR_DISABLE,              /* No se usan interrupciones de LEDC. */
        .gpio_num = led_rgb->led_red.gpio_num,       /* GPIO fisico del LED rojo. */
        .duty = led_rgb->led_red.duty,               /* Duty inicial del LED rojo. */
        .hpoint = 0                                  /* Punto inicial del pulso PWM. */
    };

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_red)); /* Aplica el canal rojo. */

    ledc_channel_config_t ledc_channel_green = {     /* Estructura de configuracion del canal verde. */
        .speed_mode = led_rgb->speed_mode,           /* Usa el mismo modo LEDC del timer. */
        .channel = led_rgb->led_green.channel,       /* Canal PWM asignado al LED verde. */
        .timer_sel = led_rgb->timer,                 /* Timer PWM asociado a este canal. */
        .intr_type = LEDC_INTR_DISABLE,              /* No se usan interrupciones de LEDC. */
        .gpio_num = led_rgb->led_green.gpio_num,     /* GPIO fisico del LED verde. */
        .duty = led_rgb->led_green.duty,             /* Duty inicial del LED verde. */
        .hpoint = 0                                  /* Punto inicial del pulso PWM. */
    };

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_green)); /* Aplica el canal verde. */

    ledc_channel_config_t ledc_channel_blue = {      /* Estructura de configuracion del canal azul. */
        .speed_mode = led_rgb->speed_mode,           /* Usa el mismo modo LEDC del timer. */
        .channel = led_rgb->led_blue.channel,        /* Canal PWM asignado al LED azul. */
        .timer_sel = led_rgb->timer,                 /* Timer PWM asociado a este canal. */
        .intr_type = LEDC_INTR_DISABLE,              /* No se usan interrupciones de LEDC. */
        .gpio_num = led_rgb->led_blue.gpio_num,      /* GPIO fisico del LED azul. */
        .duty = led_rgb->led_blue.duty,              /* Duty inicial del LED azul. */
        .hpoint = 0                                  /* Punto inicial del pulso PWM. */
    };

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_blue)); /* Aplica el canal azul. */
}

/* ============================================================================
 *  3. Actualizacion del LED usando los duty guardados en la estructura
 * ========================================================================== */

void set_led_rgb_given_struct(led_rgb_t *led_rgb)
{
    ESP_ERROR_CHECK(ledc_set_duty(led_rgb->speed_mode, led_rgb->led_red.channel, led_rgb->led_red.duty));
    ESP_ERROR_CHECK(ledc_set_duty(led_rgb->speed_mode, led_rgb->led_green.channel, led_rgb->led_green.duty));
    ESP_ERROR_CHECK(ledc_set_duty(led_rgb->speed_mode, led_rgb->led_blue.channel, led_rgb->led_blue.duty));

    ESP_ERROR_CHECK(ledc_update_duty(led_rgb->speed_mode, led_rgb->led_red.channel));
    ESP_ERROR_CHECK(ledc_update_duty(led_rgb->speed_mode, led_rgb->led_green.channel));
    ESP_ERROR_CHECK(ledc_update_duty(led_rgb->speed_mode, led_rgb->led_blue.channel));
}

/* ============================================================================
 *  4. Actualizacion del LED usando valores duty directos
 * ========================================================================== */

void set_led_rgb_given_values(led_rgb_t *led_rgb, uint32_t duty_red, uint32_t duty_green, uint32_t duty_blue)
{
    led_rgb->led_red.duty = duty_red;       /* Guarda el nuevo duty rojo dentro de la estructura. */
    led_rgb->led_green.duty = duty_green;   /* Guarda el nuevo duty verde dentro de la estructura. */
    led_rgb->led_blue.duty = duty_blue;     /* Guarda el nuevo duty azul dentro de la estructura. */

    ESP_ERROR_CHECK(ledc_set_duty(led_rgb->speed_mode, led_rgb->led_red.channel, duty_red));
    ESP_ERROR_CHECK(ledc_set_duty(led_rgb->speed_mode, led_rgb->led_green.channel, duty_green));
    ESP_ERROR_CHECK(ledc_set_duty(led_rgb->speed_mode, led_rgb->led_blue.channel, duty_blue));

    ESP_ERROR_CHECK(ledc_update_duty(led_rgb->speed_mode, led_rgb->led_red.channel));
    ESP_ERROR_CHECK(ledc_update_duty(led_rgb->speed_mode, led_rgb->led_green.channel));
    ESP_ERROR_CHECK(ledc_update_duty(led_rgb->speed_mode, led_rgb->led_blue.channel));
}

/* ============================================================================
 *  5. Actualizacion del LED usando porcentajes de brillo
 * ========================================================================== */

void set_led_rgb_percentage_given_values(led_rgb_t *led_rgb, int percentage_red, int percentage_green, int percentage_blue)
{
    uint32_t max_duty = max_duty_for_resolution(led_rgb->duty_resolution); /* Duty maximo segun la resolucion. */
    uint32_t duty_red;                                                     /* Duty calculado para el rojo. */
    uint32_t duty_green;                                                   /* Duty calculado para el verde. */
    uint32_t duty_blue;                                                    /* Duty calculado para el azul. */

    if (percentage_red < 0) {                 /* Evita porcentajes rojos menores que 0%. */
        percentage_red = 0;                   /* Ajusta el rojo al minimo permitido. */
    } else if (percentage_red > 100) {        /* Evita porcentajes rojos mayores que 100%. */
        percentage_red = 100;                 /* Ajusta el rojo al maximo permitido. */
    }

    if (percentage_green < 0) {               /* Evita porcentajes verdes menores que 0%. */
        percentage_green = 0;                 /* Ajusta el verde al minimo permitido. */
    } else if (percentage_green > 100) {      /* Evita porcentajes verdes mayores que 100%. */
        percentage_green = 100;               /* Ajusta el verde al maximo permitido. */
    }

    if (percentage_blue < 0) {                /* Evita porcentajes azules menores que 0%. */
        percentage_blue = 0;                  /* Ajusta el azul al minimo permitido. */
    } else if (percentage_blue > 100) {       /* Evita porcentajes azules mayores que 100%. */
        percentage_blue = 100;                /* Ajusta el azul al maximo permitido. */
    }

    duty_red = (max_duty * (uint32_t)percentage_red) / 100U;       /* Convierte rojo de porcentaje a duty. */
    duty_green = (max_duty * (uint32_t)percentage_green) / 100U;   /* Convierte verde de porcentaje a duty. */
    duty_blue = (max_duty * (uint32_t)percentage_blue) / 100U;     /* Convierte azul de porcentaje a duty. */

    set_led_rgb_given_values(led_rgb, duty_red, duty_green, duty_blue);    /* Aplica los tres duty al LED RGB. */
}
