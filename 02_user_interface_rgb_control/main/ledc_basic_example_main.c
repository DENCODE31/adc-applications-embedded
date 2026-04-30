#include <stdio.h>                     /* Permite usar printf para enviar mensajes al monitor serial. */
#include <stdlib.h>                    /* Se anadio para usar la funcion abs() (valor absoluto) y evitar brincos de brillo. */
#include <stdbool.h>                   /* Permite usar variables booleanas con los valores true y false. */
#include <stdint.h>                    /* Permite usar tipos enteros de tamano fijo como uint32_t. */

#include "freertos/FreeRTOS.h"         /* Incluye funciones base de FreeRTOS, como la conversion de milisegundos a ticks. */
#include "freertos/task.h"             /* Permite usar vTaskDelay para pausar la tarea principal. */

#include "driver/gpio.h"               /* Permite configurar y leer pines digitales GPIO. */
#include "driver/ledc.h"               /* Permite usar PWM mediante el periferico LEDC del ESP32. */

#include "esp_adc/adc_oneshot.h"       /* Permite hacer lecturas ADC simples usando el modo oneshot. */
#include "esp_err.h"                   /* Permite usar ESP_ERROR_CHECK para validar llamadas de ESP-IDF. */

#include "library_led_c.h"             /* Incluye las estructuras y funciones propias para controlar el LED RGB. */

/* ========================================================================== */
/*                              Pines del LED RGB                             */
/* ========================================================================== */

#define LED_RGB_1_RED_GPIO        (13)              /* Define el GPIO conectado al color rojo del LED RGB. */
#define LED_RGB_1_GREEN_GPIO      (19)              /* Define el GPIO conectado al color verde del LED RGB, dejando GPIO4 libre para el ADC. */
#define LED_RGB_1_BLUE_GPIO       (5)               /* Define el GPIO conectado al color azul del LED RGB. */

/* ========================================================================== */
/*                            Entrada del usuario                             */
/* ========================================================================== */

#define BUTTON_COLOR_GPIO         (18)              /* Define el GPIO del boton que cambia el color activo. */
#define BUTTON_PRESSED_LEVEL      (0)               /* Define que el boton se considera presionado cuando el pin lee 0. */

/* ========================================================================== */
/*                         Entrada analogica del ADC                          */
/* ========================================================================== */

#define POTENTIOMETER_ADC_UNIT    ADC_UNIT_1        /* Define que el potenciometro se lee usando la unidad ADC1. */
#define POTENTIOMETER_ADC_CHANNEL ADC_CHANNEL_4     /* Define el canal ADC conectado al cursor del potenciometro; en ESP32-C6 es GPIO4. */
#define POTENTIOMETER_ADC_ATTEN   ADC_ATTEN_DB_12   /* Permite medir un rango de voltaje mas amplio en el ADC. */
#define POTENTIOMETER_ADC_WIDTH   ADC_BITWIDTH_12   /* Define una lectura ADC de 12 bits: valores de 0 a 4095. */
#define ADC_MAX_RAW_VALUE         (4095)            /* Guarda el valor maximo que entrega una lectura ADC de 12 bits. */

/* ========================================================================== */
/*                         Tiempos de ejecucion                               */
/* ========================================================================== */

#define POLL_DELAY_MS             (20)              /* Define cada cuantos milisegundos se leen boton y potenciometro. */
#define DEBOUNCE_DELAY_MS         (30)              /* Define el tiempo usado para filtrar rebotes mecanicos del boton. */
#define PRINT_DELAY_CYCLES        (25)              /* Define cada cuantas vueltas se imprime el estado por consola. */

/* ========================================================================== */
/*                         Estados posibles del LED                           */
/* ========================================================================== */

typedef enum {                                      /* Crea un tipo enumerado para representar el color seleccionado. */
    LED_COLOR_BLUE = 0,                             /* Representa el estado en el que el color activo es azul. */
    LED_COLOR_GREEN,                                /* Representa el estado en el que el color activo es verde. */
    LED_COLOR_RED                                   /* Representa el estado en el que el color activo es rojo. */
} led_color_state_t;                                /* Nombra el tipo como led_color_state_t para usarlo en el programa. */

/* ========================================================================== */
/*                       Configuracion de entradas GPIO                       */
/* ========================================================================== */

static void config_button(gpio_num_t gpio_num)      /* Declara una funcion local para configurar un pin como boton. */
{                                                   /* Inicia el bloque de codigo de la funcion config_button. */
    gpio_config_t io_conf = {                       /* Crea una estructura con la configuracion electrica del GPIO. */
        .pin_bit_mask = 1ULL << gpio_num,           /* Selecciona el pin recibido desplazando un bit hasta su posicion. */
        .mode = GPIO_MODE_INPUT,                    /* Configura el GPIO como entrada digital. */
        .pull_up_en = GPIO_PULLUP_ENABLE,           /* Activa la resistencia pull-up interna para dejar el pin en 1 al soltar el boton. */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,      /* Desactiva la resistencia pull-down interna porque no se necesita. */
        .intr_type = GPIO_INTR_DISABLE              /* Desactiva interrupciones porque se usara lectura por consulta. */
    };                                              /* Termina la inicializacion de la estructura io_conf. */

    ESP_ERROR_CHECK(gpio_config(&io_conf));         /* Aplica la configuracion del GPIO y detiene el programa si falla. */
}                                                   /* Termina la funcion config_button. */

/* ========================================================================== */
/*                       Lectura de boton con antirrebote                     */
/* ========================================================================== */

static bool button_was_pressed(gpio_num_t gpio_num, bool *previous_state) /* Declara una funcion que detecta una pulsacion nueva. */
{                                                                         /* Inicia el bloque de codigo de button_was_pressed. */
    bool current_state = gpio_get_level(gpio_num) == BUTTON_PRESSED_LEVEL; /* Lee el GPIO y lo convierte a true si el boton esta presionado. */

    if (current_state && !(*previous_state)) {        /* Entra solo cuando el boton acaba de pasar de suelto a presionado. */
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS)); /* Espera unos milisegundos para dejar pasar el rebote mecanico. */
        current_state = gpio_get_level(gpio_num) == BUTTON_PRESSED_LEVEL; /* Vuelve a leer el boton despues del antirrebote. */

        if (current_state) {                          /* Confirma la pulsacion si el boton sigue presionado. */
            *previous_state = true;                   /* Guarda que esta pulsacion ya fue contada. */
            return true;                              /* Informa al programa principal que hubo una nueva pulsacion. */
        }                                             /* Termina la confirmacion de pulsacion estable. */
    }                                                 /* Termina la deteccion del flanco de presion. */

    if (!current_state) {                             /* Revisa si el boton esta suelto. */
        *previous_state = false;                      /* Libera la memoria del boton para poder detectar la siguiente pulsacion. */
    }                                                 /* Termina la actualizacion del estado al soltar. */

    return false;                                     /* Devuelve false cuando no hubo una pulsacion nueva. */
}                                                     /* Termina la funcion button_was_pressed. */

/* ========================================================================== */
/*                         Configuracion del ADC                              */
/* ========================================================================== */

static adc_oneshot_unit_handle_t config_potentiometer_adc(void) /* Declara una funcion que configura el ADC del potenciometro. */
{                                                               /* Inicia el bloque de codigo de config_potentiometer_adc. */
    adc_oneshot_unit_handle_t adc_handle = NULL;                 /* Crea el manejador del ADC e inicia su valor en NULL. */

    adc_oneshot_unit_init_cfg_t init_config = {                  /* Crea la configuracion inicial de la unidad ADC. */
        .unit_id = POTENTIOMETER_ADC_UNIT                        /* Selecciona la unidad ADC definida para el potenciometro. */
    };                                                          /* Termina la configuracion inicial de la unidad ADC. */

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle)); /* Crea la unidad ADC y guarda su manejador. */

    adc_oneshot_chan_cfg_t channel_config = {                    /* Crea la configuracion del canal ADC. */
        .atten = POTENTIOMETER_ADC_ATTEN,                        /* Configura la atenuacion para aceptar el rango del potenciometro. */
        .bitwidth = POTENTIOMETER_ADC_WIDTH                      /* Configura la resolucion de la lectura ADC. */
    };                                                          /* Termina la configuracion del canal ADC. */

    ESP_ERROR_CHECK(adc_oneshot_config_channel(                  /* Configura el canal fisico donde esta conectado el potenciometro. */
        adc_handle,                                             /* Entrega el manejador de la unidad ADC. */
        POTENTIOMETER_ADC_CHANNEL,                              /* Entrega el canal ADC que se va a leer. */
        &channel_config                                         /* Entrega la direccion de la configuracion del canal. */
    ));                                                         /* Cierra la llamada de configuracion y valida errores. */

    return adc_handle;                                          /* Devuelve el manejador del ADC ya configurado. */
}                                                               /* Termina la funcion config_potentiometer_adc. */

/* ========================================================================== */
/*                        Conversion ADC a porcentaje                         */
/* ========================================================================== */

static int adc_raw_to_percentage(int raw_value)      /* Declara una funcion para convertir una lectura ADC a porcentaje. */
{                                                    /* Inicia el bloque de codigo de adc_raw_to_percentage. */
    if (raw_value < 0) {                             /* Revisa si por alguna razon la lectura queda por debajo de cero. */
        raw_value = 0;                               /* Limita el valor minimo a cero para evitar porcentajes negativos. */
    }                                                /* Termina la validacion del limite inferior. */

    if (raw_value > ADC_MAX_RAW_VALUE) {             /* Revisa si la lectura supera el maximo esperado de 12 bits. */
        raw_value = ADC_MAX_RAW_VALUE;               /* Limita el valor maximo para evitar porcentajes mayores a 100. */
    }                                                /* Termina la validacion del limite superior. */

    return (raw_value * 100) / ADC_MAX_RAW_VALUE;    /* Escala el valor ADC de 0-4095 a un porcentaje de 0-100. */
}                                                    /* Termina la funcion adc_raw_to_percentage. */

/* ========================================================================== */
/*                          Maquina de estados RGB                            */
/* ========================================================================== */

static led_color_state_t next_color_state(led_color_state_t current_color) /* Declara una funcion que calcula el siguiente color. */
{                                                                          /* Inicia el bloque de codigo de next_color_state. */
    switch (current_color) {                                                /* Evalua el color actual para decidir el siguiente. */
    case LED_COLOR_BLUE:                                                    /* Si el color actual es azul, el siguiente sera verde. */
        return LED_COLOR_GREEN;                                             /* Devuelve el estado verde. */
    case LED_COLOR_GREEN:                                                   /* Si el color actual es verde, el siguiente sera rojo. */
        return LED_COLOR_RED;                                               /* Devuelve el estado rojo. */
    case LED_COLOR_RED:                                                     /* Si el color actual es rojo, el siguiente sera azul. */
    default:                                                                /* Tambien usa azul si el estado llega a ser inesperado. */
        return LED_COLOR_BLUE;                                              /* Devuelve el estado azul para cerrar el ciclo. */
    }                                                                       /* Termina la evaluacion del switch. */
}                                                                           /* Termina la funcion next_color_state. */

static const char *color_state_to_text(led_color_state_t current_color) /* Declara una funcion que convierte el color a texto. */
{                                                                       /* Inicia el bloque de codigo de color_state_to_text. */
    switch (current_color) {                                             /* Evalua el color actual. */
    case LED_COLOR_BLUE:                                                 /* Revisa si el estado corresponde al azul. */
        return "azul";                                                   /* Devuelve el texto azul. */
    case LED_COLOR_GREEN:                                                /* Revisa si el estado corresponde al verde. */
        return "verde";                                                  /* Devuelve el texto verde. */
    case LED_COLOR_RED:                                                  /* Revisa si el estado corresponde al rojo. */
        return "rojo";                                                   /* Devuelve el texto rojo. */
    default:                                                             /* Atiende cualquier valor inesperado. */
        return "desconocido";                                            /* Devuelve un texto de seguridad para depuracion. */
    }                                                                    /* Termina la evaluacion del switch. */
}                                                                        /* Termina la funcion color_state_to_text. */

static void apply_rgb_state(led_rgb_t *led_rgb, led_color_state_t color, int brightness_percentage) /* Declara una funcion que aplica color y brillo al LED. */
{                                                                                                  /* Inicia el bloque de codigo de apply_rgb_state. */
    int red_percentage = 0;                                                                         /* Crea el brillo rojo en 0 para apagarlo por defecto. */
    int green_percentage = 0;                                                                       /* Crea el brillo verde en 0 para apagarlo por defecto. */
    int blue_percentage = 0;                                                                        /* Crea el brillo azul en 0 para apagarlo por defecto. */

    switch (color) {                                                                                /* Evalua cual color debe quedar encendido. */
    case LED_COLOR_BLUE:                                                                            /* Revisa si el color seleccionado es azul. */
        blue_percentage = brightness_percentage;                                                    /* Asigna el ultimo brillo memorizado al canal azul. */
        break;                                                                                      /* Sale del switch despues de configurar azul. */
    case LED_COLOR_GREEN:                                                                           /* Revisa si el color seleccionado es verde. */
        green_percentage = brightness_percentage;                                                   /* Asigna el ultimo brillo memorizado al canal verde. */
        break;                                                                                      /* Sale del switch despues de configurar verde. */
    case LED_COLOR_RED:                                                                             /* Revisa si el color seleccionado es rojo. */
        red_percentage = brightness_percentage;                                                     /* Asigna el ultimo brillo memorizado al canal rojo. */
        break;                                                                                      /* Sale del switch despues de configurar rojo. */
    default:                                                                                        /* Atiende cualquier estado inesperado. */
        blue_percentage = brightness_percentage;                                                    /* Usa azul como estado seguro por defecto. */
        break;                                                                                      /* Sale del switch despues del caso por defecto. */
    }                                                                                               /* Termina la seleccion de color. */

    set_led_rgb_percentage_given_values(                                                            /* Llama a la libreria para actualizar el PWM del LED RGB. */
        led_rgb,                                                                                    /* Entrega la estructura que contiene los canales LEDC. */
        red_percentage,                                                                             /* Entrega el porcentaje que se aplicara al canal rojo. */
        green_percentage,                                                                           /* Entrega el porcentaje que se aplicara al canal verde. */
        blue_percentage                                                                             /* Entrega el porcentaje que se aplicara al canal azul. */
    );                                                                                              /* Cierra la llamada que aplica los porcentajes RGB. */
}                                                                                                   /* Termina la funcion apply_rgb_state. */

/* ========================================================================== */
/*                              Programa principal                            */
/* ========================================================================== */

void app_main(void)                                      /* Define la funcion principal que ESP-IDF ejecuta al iniciar. */
{                                                        /* Inicia el bloque de codigo de app_main. */
    led_rgb_t led_rgb1 = {                               /* Crea la estructura que describe todo el LED RGB. */
        .led_red = {                                     /* Inicia la configuracion del canal rojo. */
            .duty = 0,                                   /* Inicia el rojo apagado. */
            .gpio_num = LED_RGB_1_RED_GPIO,              /* Asocia el rojo con su GPIO fisico. */
            .channel = LEDC_CHANNEL_0                    /* Asocia el rojo con el canal PWM 0. */
        },                                               /* Termina la configuracion del canal rojo. */
        .led_green = {                                   /* Inicia la configuracion del canal verde. */
            .duty = 0,                                   /* Inicia el verde apagado. */
            .gpio_num = LED_RGB_1_GREEN_GPIO,            /* Asocia el verde con su GPIO fisico. */
            .channel = LEDC_CHANNEL_1                    /* Asocia el verde con el canal PWM 1. */
        },                                               /* Termina la configuracion del canal verde. */
        .led_blue = {                                    /* Inicia la configuracion del canal azul. */
            .duty = 0,                                   /* Inicia el azul apagado. */
            .gpio_num = LED_RGB_1_BLUE_GPIO,             /* Asocia el azul con su GPIO fisico. */
            .channel = LEDC_CHANNEL_2                    /* Asocia el azul con el canal PWM 2. */
        },                                               /* Termina la configuracion del canal azul. */
        .timer = LEDC_TIMER_0,                           /* Usa el timer LEDC 0 para los tres canales PWM. */
        .duty_resolution = LEDC_TIMER_13_BIT,            /* Usa 13 bits de resolucion PWM para variar suavemente el brillo. */
        .frequency = 4000,                               /* Define la frecuencia PWM en 4000 Hz para evitar parpadeo visible. */
        .speed_mode = LEDC_LOW_SPEED_MODE                /* Usa el modo de baja velocidad del periferico LEDC. */
    };                                                   /* Termina la estructura de configuracion del LED RGB. */

    adc_oneshot_unit_handle_t adc_handle = config_potentiometer_adc(); /* Configura el ADC y guarda su manejador. */
    led_color_state_t current_color = LED_COLOR_BLUE;                  /* Inicia el sistema con el color azul seleccionado. */
    bool button_previous_state = false;                                /* Guarda el estado anterior del boton para detectar una pulsacion por vez. */
    int adc_raw_value = 0;                                              /* Guarda la lectura cruda del ADC entre 0 y 4095. */
    
    /* === CAMBIO: MEMORIA INDEPENDIENTE DE BRILLO ===
     * Se crea un arreglo de 3 posiciones para recordar el brillo de cada color por separado.
     * Indices: 0 = Azul, 1 = Verde, 2 = Rojo. */
    int color_brightness[3] = {0, 0, 0};                                /* Arreglo para guardar el brillo independiente de cada color. */
    int current_pot_percentage = 0;                                     /* Guarda el porcentaje actual leido del potenciometro. */
    
    /* === CAMBIO: EVITAR BRINCOS (TAKEOVER) === 
     * Se usa para saber donde estaba fisicamente el potenciometro la ultima vez que se movio intencionalmente. */
    int last_pot_percentage = -1;                                       /* Guarda el ultimo porcentaje aplicado para detectar cambios intencionales. */
    uint32_t print_counter = 0;                                         /* Cuenta vueltas del ciclo para no imprimir en cada lectura. */

    config_led_rgb(&led_rgb1);                                          /* Configura el timer y los canales PWM del LED RGB. */
    config_button(BUTTON_COLOR_GPIO);                                   /* Configura el boton de cambio de color como entrada con pull-up. */
    apply_rgb_state(&led_rgb1, current_color, color_brightness[current_color]); /* Aplica el estado inicial: azul seleccionado con brillo 0. */

    printf("Sistema RGB con ADC listo.\n");                             /* Informa por consola que el programa arranco correctamente. */
    printf("Orden del boton: azul -> verde -> rojo -> azul.\n");        /* Informa el orden de cambio de color solicitado. */
    printf("ADC canal %d controla el brillo; boton GPIO %d cambia color.\n", POTENTIOMETER_ADC_CHANNEL, BUTTON_COLOR_GPIO); /* Muestra canal ADC y pin del boton. */

    while (1) {                                                         /* Inicia el bucle infinito del sistema embebido. */
        ESP_ERROR_CHECK(adc_oneshot_read(                               /* Lee el valor instantaneo del potenciometro usando ADC oneshot. */
            adc_handle,                                                 /* Entrega el manejador de la unidad ADC configurada. */
            POTENTIOMETER_ADC_CHANNEL,                                  /* Entrega el canal ADC conectado al potenciometro. */
            &adc_raw_value                                              /* Entrega la direccion donde se guardara la lectura cruda. */
        ));                                                             /* Cierra la lectura ADC y valida errores. */

        current_pot_percentage = adc_raw_to_percentage(adc_raw_value);       /* Convierte la lectura ADC al porcentaje actual del potenciometro. */

        if (button_was_pressed(BUTTON_COLOR_GPIO, &button_previous_state)) { /* Revisa si el usuario presiono el boton de cambio de color. */
            current_color = next_color_state(current_color);                 /* Cambia al siguiente color en el orden azul, verde, rojo. */
            
            /* === CAMBIO: RECUPERAR BRILLO AL CAMBIAR COLOR ===
             * Al cambiar de color, no se lee el potenciometro de inmediato, sino que se restaura el brillo guardado en la memoria.
             * Tambien actualizamos last_pot_percentage para que el potencimetro no asuma el control hasta que el usuario lo vuelva a mover fisicamente. */
            last_pot_percentage = current_pot_percentage;                    /* Actualiza la referencia del pot para evitar brincos por ruido al cambiar. */
            apply_rgb_state(&led_rgb1, current_color, color_brightness[current_color]); /* Aplica el nuevo color restaurando su brillo guardado. */
            printf("Color activo: %s | Brillo recuperado: %d%%\n", color_state_to_text(current_color), color_brightness[current_color]); /* Reporta el nuevo estado. */
        }                                                                    /* Termina la atencion de la pulsacion del boton. */

        /* === CAMBIO: CONTROL DE UMBRAL ABSOLUTO ===
         * abs() asegura que no importa si giras la perilla a la izquierda o derecha.
         * Solo si el cambio es mayor o igual a 2%, consideramos que fue un movimiento intencional del usuario 
         * y no simple ruido electrico del potenciometro. En ese momento, se sobrescribe la memoria. */
        if (abs(current_pot_percentage - last_pot_percentage) >= 2) {        /* Revisa si el usuario movio el potenciometro intencionalmente (mas del 2%). */
            last_pot_percentage = current_pot_percentage;                    /* Actualiza la memoria del potenciometro. */
            color_brightness[current_color] = current_pot_percentage;        /* Guarda el nuevo brillo para el color actual de forma independiente. */
            apply_rgb_state(&led_rgb1, current_color, color_brightness[current_color]); /* Aplica el brillo nuevo al color activo. */
        }                                                                    /* Termina la actualizacion por cambio de potenciometro. */

        if (print_counter >= PRINT_DELAY_CYCLES) {                           /* Revisa si ya pasaron suficientes ciclos para imprimir estado. */
            print_counter = 0;                                               /* Reinicia el contador de impresion. */
            printf("ADC crudo: %d | Color: %s | Brillo: %d%%\n", adc_raw_value, color_state_to_text(current_color), color_brightness[current_color]); /* Imprime diagnostico del sistema. */
        } else {                                                             /* Ejecuta esta rama cuando aun no toca imprimir. */
            print_counter++;                                                 /* Aumenta el contador de ciclos sin imprimir. */
        }                                                                    /* Termina la logica de impresion periodica. */

        vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));                            /* Pausa el ciclo para reducir carga de CPU y estabilizar lecturas. */
    }                                                                        /* Termina una vuelta del bucle infinito y vuelve al inicio. */
}                                                                            /* Termina la funcion app_main. */
