#include <stdio.h>                 /* Permite usar printf para mostrar mensajes por consola. */
#include <stdbool.h>               /* Permite usar el tipo bool con true y false. */

#include "freertos/FreeRTOS.h"     /* Incluye funciones base de FreeRTOS, como conversion de tiempos. */
#include "freertos/task.h"         /* Permite usar vTaskDelay para pausar la tarea principal. */

#include "driver/gpio.h"           /* Permite configurar y leer pines GPIO. */
#include "driver/ledc.h"           /* Permite usar PWM con el periferico LEDC del ESP32. */

#include "esp_err.h"               /* Permite usar ESP_ERROR_CHECK para validar errores de ESP-IDF. */
#include "library_led_c.h"         /* Incluye las estructuras y funciones propias para manejar el LED RGB. */

/* ========================================================================== */
/*                              Pines del LED RGB                             */
/* ========================================================================== */

/* Cada color del LED RGB se conecta a un pin diferente del ESP32. */
#define LED_RGB_1_RED_GPIO        (13)  /* Pin GPIO conectado al color rojo. */
#define LED_RGB_1_GREEN_GPIO      (4)   /* Pin GPIO conectado al color verde. */
#define LED_RGB_1_BLUE_GPIO       (5)   /* Pin GPIO conectado al color azul. */

/* ========================================================================== */
/*                            Pines de los pulsadores                         */
/* ========================================================================== */

/* Cada pulsador controla el brillo de un color del LED RGB. */
#define BUTTON_RED_GPIO           (18)  /* Pin GPIO conectado al pulsador rojo. */
#define BUTTON_GREEN_GPIO         (19)  /* Pin GPIO conectado al pulsador verde. */
#define BUTTON_BLUE_GPIO          (20)  /* Pin GPIO conectado al pulsador azul. */

/* Los botones se conectan a GND y usan pull-up interno.
 * Por eso, cuando el boton se presiona, el pin lee nivel bajo: 0.
 */
#define BUTTON_PRESSED_LEVEL      (0)   /* Nivel logico que indica boton presionado. */

/* ========================================================================== */
/*                         Tiempos y pasos de control                         */
/* ========================================================================== */

#define POLL_DELAY_MS             (20)  /* Espera entre lecturas de botones. */
#define DEBOUNCE_DELAY_MS         (30)  /* Espera para eliminar rebote mecanico. */
#define BRIGHTNESS_STEP_PERCENT   (10)  /* Incremento de brillo por cada pulsacion. */

/* ========================================================================== */
/*                       Configuracion de entradas GPIO                       */
/* ========================================================================== */

static void config_button(gpio_num_t gpio_num)
{
    /* gpio_config_t guarda todos los parametros para configurar un pin GPIO. */
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << gpio_num,       /* Selecciona el pin que se va a configurar. */
        .mode = GPIO_MODE_INPUT,                /* Configura el pin como entrada digital. */
        .pull_up_en = GPIO_PULLUP_ENABLE,       /* Activa la resistencia pull-up interna. */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  /* Desactiva la resistencia pull-down interna. */
        .intr_type = GPIO_INTR_DISABLE          /* No se usan interrupciones; se lee por consulta. */
    };

    /* Aplica la configuracion y detiene el programa si ocurre un error. */
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

/* ========================================================================== */
/*                       Lectura de botones con antirrebote                   */
/* ========================================================================== */

static bool button_was_pressed(gpio_num_t gpio_num, bool *previous_state)
{
    /* Lee el pin y lo convierte a true si el boton esta presionado. */
    bool current_state = gpio_get_level(gpio_num) == BUTTON_PRESSED_LEVEL;

    /* Entra aqui solo cuando el boton acaba de pasar de suelto a presionado. */
    if (current_state && !(*previous_state)) {
        /* Espera un poco para ignorar rebotes mecanicos del pulsador. */
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));

        /* Vuelve a leer el boton despues del antirrebote. */
        current_state = gpio_get_level(gpio_num) == BUTTON_PRESSED_LEVEL;

        /* Si sigue presionado, se confirma una pulsacion valida. */
        if (current_state) {
            *previous_state = true;  /* Guarda que el boton ya fue contado. */
            return true;             /* Indica al programa principal que hubo pulsacion. */
        }
    }

    /* Si el boton esta suelto, se permite detectar una nueva pulsacion despues. */
    if (!current_state) {
        *previous_state = false;     /* Reinicia el estado anterior del boton. */
    }

    return false;                    /* No se detecto una pulsacion nueva. */
}

/* ========================================================================== */
/*                       Calculo del siguiente brillo                         */
/* ========================================================================== */

static int next_brightness_step(int current_percentage)
{
    /* Aumenta el brillo actual en el paso definido, por ejemplo de 20% a 30%. */
    current_percentage += BRIGHTNESS_STEP_PERCENT;

    /* Si el brillo supera 100%, vuelve a 0% para reiniciar el ciclo. */
    if (current_percentage > 100) {
        current_percentage = 0;
    }

    return current_percentage;       /* Devuelve el nuevo porcentaje de brillo. */
}

/* ========================================================================== */
/*                              Programa principal                            */
/* ========================================================================== */

void app_main(void)
{
    /* Estructura que contiene toda la configuracion del LED RGB. */
    led_rgb_t led_rgb1 = {
        .led_red = {
            .duty = 0,                           /* Duty inicial del rojo: apagado. */
            .gpio_num = LED_RGB_1_RED_GPIO,      /* Pin fisico del color rojo. */
            .channel = LEDC_CHANNEL_0            /* Canal PWM asignado al rojo. */
        },
        .led_green = {
            .duty = 0,                           /* Duty inicial del verde: apagado. */
            .gpio_num = LED_RGB_1_GREEN_GPIO,    /* Pin fisico del color verde. */
            .channel = LEDC_CHANNEL_1            /* Canal PWM asignado al verde. */
        },
        .led_blue = {
            .duty = 0,                           /* Duty inicial del azul: apagado. */
            .gpio_num = LED_RGB_1_BLUE_GPIO,     /* Pin fisico del color azul. */
            .channel = LEDC_CHANNEL_2            /* Canal PWM asignado al azul. */
        },
        .timer = LEDC_TIMER_0,                   /* Timer LEDC usado por los tres canales. */
        .duty_resolution = LEDC_TIMER_13_BIT,    /* Resolucion PWM de 13 bits: 0 a 8191. */
        .frequency = 4000,                       /* Frecuencia PWM de 4000 Hz. */
        .speed_mode = LEDC_LOW_SPEED_MODE        /* Modo de velocidad del periferico LEDC. */
    };

    /* Porcentajes actuales de brillo para cada color. */
    int red_percentage = 0;       /* Brillo rojo inicial en 0%. */
    int green_percentage = 0;     /* Brillo verde inicial en 0%. */
    int blue_percentage = 0;      /* Brillo azul inicial en 0%. */

    /* Estados anteriores de los botones para contar solo una vez cada pulsacion. */
    bool red_button_previous_state = false;    /* Estado anterior del boton rojo. */
    bool green_button_previous_state = false;  /* Estado anterior del boton verde. */
    bool blue_button_previous_state = false;   /* Estado anterior del boton azul. */

    /* Configura timer y canales PWM del LED RGB. */
    config_led_rgb(&led_rgb1);

    /* Configura cada pulsador como entrada con pull-up. */
    config_button(BUTTON_RED_GPIO);
    config_button(BUTTON_GREEN_GPIO);
    config_button(BUTTON_BLUE_GPIO);

    /* Apaga el LED al inicio usando los porcentajes iniciales. */
    set_led_rgb_percentage_given_values(
        &led_rgb1,
        red_percentage,
        green_percentage,
        blue_percentage
    );

    /* Muestra mensajes iniciales para confirmar que el programa arranco. */
    printf("Control RGB de catodo comun listo.\n");
    printf(
        "Pulsador rojo GPIO %d, verde GPIO %d, azul GPIO %d.\n",
        BUTTON_RED_GPIO,
        BUTTON_GREEN_GPIO,
        BUTTON_BLUE_GPIO
    );

    /* Bucle infinito: el programa siempre queda leyendo los tres botones. */
    while (1) {
        /* Indica si algun color cambio durante esta vuelta del ciclo. */
        bool updated = false;

        /* Si se presiona el boton rojo, se aumenta el brillo del color rojo. */
        if (button_was_pressed(BUTTON_RED_GPIO, &red_button_previous_state)) {
            red_percentage = next_brightness_step(red_percentage);
            updated = true;
        }

        /* Si se presiona el boton verde, se aumenta el brillo del color verde. */
        if (button_was_pressed(BUTTON_GREEN_GPIO, &green_button_previous_state)) {
            green_percentage = next_brightness_step(green_percentage);
            updated = true;
        }

        /* Si se presiona el boton azul, se aumenta el brillo del color azul. */
        if (button_was_pressed(BUTTON_BLUE_GPIO, &blue_button_previous_state)) {
            blue_percentage = next_brightness_step(blue_percentage);
            updated = true;
        }

        /* Si hubo cambios, se actualiza el LED RGB con los nuevos porcentajes. */
        if (updated) {
            set_led_rgb_percentage_given_values(
                &led_rgb1,
                red_percentage,
                green_percentage,
                blue_percentage
            );

            printf(
                "RGB = R:%d%% G:%d%% B:%d%%\n",
                red_percentage,
                green_percentage,
                blue_percentage
            );
        }

        /* Pausa corta para no saturar el procesador leyendo botones todo el tiempo. */
        vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));
    }
}
