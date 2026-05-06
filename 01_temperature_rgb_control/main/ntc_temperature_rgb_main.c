#include <stdio.h>                     /* Permite usar printf para enviar mensajes al monitor serial. */
#include <math.h>                      /* Permite usar logf para el calculo de temperatura con la NTC. */
#include <stdint.h>                    /* Permite usar tipos enteros de tamano fijo como uint32_t. */

#include "freertos/FreeRTOS.h"         /* Incluye funciones base de FreeRTOS necesarias para vTaskDelay. */
#include "freertos/task.h"             /* Permite usar vTaskDelay para pausar la tarea principal. */

#include "driver/ledc.h"               /* Permite usar PWM mediante el periferico LEDC del ESP32. */

#include "esp_adc/adc_oneshot.h"       /* Permite hacer lecturas ADC simples usando el modo oneshot. */
#include "esp_err.h"                   /* Permite usar ESP_ERROR_CHECK para validar llamadas de ESP-IDF. */

#include "library_led_c.h"             /* Incluye las estructuras y funciones propias para controlar el LED RGB. */

/* ========================================================================== */
/*                              Pines del LED RGB                             */
/* ========================================================================== */

#define LED_RGB_RED_GPIO     (13)              /* GPIO conectado al anodo rojo del LED RGB. */
#define LED_RGB_GREEN_GPIO   (19)              /* GPIO conectado al anodo verde del LED RGB. */
#define LED_RGB_BLUE_GPIO    (5)               /* GPIO conectado al anodo azul del LED RGB. */

/* ========================================================================== */
/*                         Entrada analogica de la NTC                        */
/* ========================================================================== */

#define NTC_ADC_UNIT         ADC_UNIT_1        /* Unidad ADC1 del ESP32-C6 utilizada para leer la NTC. */
#define NTC_ADC_CHANNEL      ADC_CHANNEL_0     /* Canal ADC0, correspondiente a GPIO0 en ESP32-C6. */
#define NTC_ADC_ATTEN        ADC_ATTEN_DB_12   /* Atenuacion 12 dB para medir hasta ~3.1 V en el ADC. */
#define NTC_ADC_WIDTH        ADC_BITWIDTH_12   /* Resolucion de 12 bits: valores de 0 a 4095. */
#define ADC_MAX_RAW          (4095)            /* Valor maximo que entrega una lectura ADC de 12 bits. */

/* ========================================================================== */
/*                         Parametros de la NTC (divisor de voltaje)          */
/* ========================================================================== */

/*
 * Circuito: VCC --- R_SERIE --- [GPIO0/ADC] --- NTC --- GND
 *
 * La NTC disminuye su resistencia al subir la temperatura.
 * A mayor temperatura → mayor tension en el nodo ADC → mayor lectura cruda.
 */

#define NTC_BETA             (3950.0f)         /* Coeficiente Beta tipico de una NTC 10 K. */
#define NTC_R0               (10000.0f)        /* Resistencia nominal de la NTC a 25 °C en ohms. */
#define NTC_T0_KELVIN        (298.15f)         /* Temperatura nominal convertida a Kelvin (25 + 273.15). */
#define NTC_R_SERIE          (10000.0f)        /* Resistencia en serie del divisor de voltaje en ohms. */

/* ========================================================================== */
/*                         Umbrales de temperatura                            */
/* ========================================================================== */

#define TEMP_UMBRAL_FRIO     (25.0f)           /* Por debajo de este valor se enciende el LED azul. */
#define TEMP_UMBRAL_CALIENTE (35.0f)           /* Por encima de este valor se enciende el LED rojo. */

/* ========================================================================== */
/*                         Tiempos de ejecucion                               */
/* ========================================================================== */

#define POLL_DELAY_MS        (500)             /* Pausa entre lecturas en milisegundos. */
#define PRINT_DELAY_CYCLES   (4)               /* Imprime por consola cada N ciclos para no saturar el puerto. */

/* ========================================================================== */
/*                         Configuracion del ADC                              */
/* ========================================================================== */

static adc_oneshot_unit_handle_t config_ntc_adc(void)   /* Declara funcion que configura el ADC de la NTC. */
{                                                        /* Inicia el bloque de codigo de config_ntc_adc. */
    adc_oneshot_unit_handle_t adc_handle = NULL;          /* Crea el manejador del ADC e inicia en NULL. */

    adc_oneshot_unit_init_cfg_t init_config = {           /* Crea la configuracion inicial de la unidad ADC. */
        .unit_id = NTC_ADC_UNIT                           /* Selecciona la unidad ADC definida para la NTC. */
    };                                                   /* Termina la configuracion inicial. */

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle)); /* Crea la unidad ADC y guarda el manejador. */

    adc_oneshot_chan_cfg_t channel_config = {              /* Crea la configuracion del canal ADC. */
        .atten = NTC_ADC_ATTEN,                           /* Configura la atenuacion para el rango de voltaje esperado. */
        .bitwidth = NTC_ADC_WIDTH                         /* Configura la resolucion de la lectura ADC. */
    };                                                   /* Termina la configuracion del canal. */

    ESP_ERROR_CHECK(adc_oneshot_config_channel(           /* Configura el canal fisico donde esta conectada la NTC. */
        adc_handle,                                      /* Entrega el manejador de la unidad ADC. */
        NTC_ADC_CHANNEL,                                 /* Entrega el canal ADC que se va a leer. */
        &channel_config                                  /* Entrega la direccion de la configuracion del canal. */
    ));                                                  /* Cierra la llamada de configuracion. */

    return adc_handle;                                   /* Devuelve el manejador del ADC ya configurado. */
}                                                        /* Termina la funcion config_ntc_adc. */

/* ========================================================================== */
/*                   Conversion de lectura ADC a temperatura                  */
/* ========================================================================== */

static float adc_raw_to_celsius(int raw_value)           /* Declara funcion que convierte ADC crudo a grados Celsius. */
{                                                        /* Inicia el bloque de codigo de adc_raw_to_celsius. */
    if (raw_value <= 0) {                                /* Evita division por cero cuando el ADC lee 0. */
        raw_value = 1;                                   /* Sustituye por 1 para que el calculo sea valido. */
    }                                                    /* Termina la proteccion del limite inferior. */

    if (raw_value >= ADC_MAX_RAW) {                      /* Evita division por cero cuando el ADC esta saturado. */
        raw_value = ADC_MAX_RAW - 1;                     /* Sustituye por el valor maximo menos 1. */
    }                                                    /* Termina la proteccion del limite superior. */

    /*
     * Calculo de la resistencia de la NTC usando el divisor de voltaje:
     *   R_ntc = R_serie * adc_raw / (ADC_MAX - adc_raw)
     */
    float r_ntc = NTC_R_SERIE * (float)raw_value / (float)(ADC_MAX_RAW - raw_value); /* Calcula la resistencia de la NTC. */

    /*
     * Ecuacion de Steinhart-Hart simplificada (ecuacion Beta):
     *   1/T = 1/T0 + (1/Beta) * ln(R / R0)
     */
    float inv_T = (1.0f / NTC_T0_KELVIN) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0); /* Calcula el inverso de la temperatura en Kelvin. */

    float temp_celsius = (1.0f / inv_T) - 273.15f;      /* Convierte el resultado de Kelvin a grados Celsius. */

    return temp_celsius;                                 /* Devuelve la temperatura calculada en grados Celsius. */
}                                                        /* Termina la funcion adc_raw_to_celsius. */

/* ========================================================================== */
/*                  Aplicacion del color segun la temperatura                 */
/* ========================================================================== */

static void apply_temperature_color(led_rgb_t *led_rgb, float temperature) /* Declara funcion que elige el color del LED segun la temperatura. */
{                                                                          /* Inicia el bloque de codigo de apply_temperature_color. */
    if (temperature < TEMP_UMBRAL_FRIO) {                                  /* Revisa si la temperatura esta por debajo de 25 C. */
        set_led_rgb_percentage_given_values(led_rgb, 0, 0, 100);           /* Enciende el LED azul al 100% y apaga rojo y verde. */
    } else if (temperature <= TEMP_UMBRAL_CALIENTE) {                      /* Revisa si la temperatura esta entre 25 C y 35 C inclusive. */
        set_led_rgb_percentage_given_values(led_rgb, 0, 100, 0);           /* Enciende el LED verde al 100% y apaga rojo y azul. */
    } else {                                                               /* Ejecuta esta rama si la temperatura supera los 35 C. */
        set_led_rgb_percentage_given_values(led_rgb, 100, 0, 0);           /* Enciende el LED rojo al 100% y apaga verde y azul. */
    }                                                                      /* Termina la seleccion de color por temperatura. */
}                                                                          /* Termina la funcion apply_temperature_color. */

/* ========================================================================== */
/*                       Texto descriptivo del color activo                   */
/* ========================================================================== */

static const char *temperature_to_color_text(float temperature) /* Declara funcion que devuelve el nombre del color activo como texto. */
{                                                               /* Inicia el bloque de codigo de temperature_to_color_text. */
    if (temperature < TEMP_UMBRAL_FRIO) {                       /* Revisa si corresponde al rango azul. */
        return "Azul  (T < 25 C)";                              /* Devuelve texto descriptivo del estado azul. */
    } else if (temperature <= TEMP_UMBRAL_CALIENTE) {           /* Revisa si corresponde al rango verde. */
        return "Verde (25 C <= T <= 35 C)";                     /* Devuelve texto descriptivo del estado verde. */
    } else {                                                    /* Asigna texto al rango rojo. */
        return "Rojo  (T > 35 C)";                              /* Devuelve texto descriptivo del estado rojo. */
    }                                                           /* Termina la seleccion del texto. */
}                                                               /* Termina la funcion temperature_to_color_text. */

/* ========================================================================== */
/*                              Programa principal                            */
/* ========================================================================== */

void app_main(void)                                                   /* Define la funcion principal que ESP-IDF ejecuta al iniciar. */
{                                                                     /* Inicia el bloque de codigo de app_main. */
    led_rgb_t led_rgb = {                                             /* Crea la estructura que describe todo el LED RGB. */
        .led_red = {                                                  /* Inicia la configuracion del canal rojo. */
            .duty     = 0,                                            /* Inicia el rojo apagado. */
            .gpio_num = LED_RGB_RED_GPIO,                             /* Asocia el rojo con su GPIO fisico. */
            .channel  = LEDC_CHANNEL_0                                /* Asocia el rojo con el canal PWM 0. */
        },                                                            /* Termina la configuracion del canal rojo. */
        .led_green = {                                                /* Inicia la configuracion del canal verde. */
            .duty     = 0,                                            /* Inicia el verde apagado. */
            .gpio_num = LED_RGB_GREEN_GPIO,                           /* Asocia el verde con su GPIO fisico. */
            .channel  = LEDC_CHANNEL_1                                /* Asocia el verde con el canal PWM 1. */
        },                                                            /* Termina la configuracion del canal verde. */
        .led_blue = {                                                 /* Inicia la configuracion del canal azul. */
            .duty     = 0,                                            /* Inicia el azul apagado. */
            .gpio_num = LED_RGB_BLUE_GPIO,                            /* Asocia el azul con su GPIO fisico. */
            .channel  = LEDC_CHANNEL_2                                /* Asocia el azul con el canal PWM 2. */
        },                                                            /* Termina la configuracion del canal azul. */
        .timer          = LEDC_TIMER_0,                               /* Usa el timer LEDC 0 para los tres canales PWM. */
        .duty_resolution = LEDC_TIMER_13_BIT,                         /* Usa 13 bits de resolucion PWM (0 a 8191). */
        .frequency      = 4000,                                       /* Frecuencia PWM en 4000 Hz para evitar parpadeo visible. */
        .speed_mode     = LEDC_LOW_SPEED_MODE                         /* Modo de baja velocidad del periferico LEDC. */
    };                                                                /* Termina la estructura de configuracion del LED RGB. */

    adc_oneshot_unit_handle_t adc_handle = config_ntc_adc();          /* Configura el ADC y obtiene su manejador. */
    int   adc_raw      = 0;                                           /* Almacena la lectura cruda del ADC entre 0 y 4095. */
    float temperature  = 0.0f;                                        /* Almacena la temperatura calculada en grados Celsius. */
    uint32_t print_counter = 0;                                       /* Cuenta ciclos para controlar la frecuencia de impresion. */

    config_led_rgb(&led_rgb);                                         /* Configura el timer y los canales PWM del LED RGB. */
    set_led_rgb_percentage_given_values(&led_rgb, 0, 0, 100);         /* Estado inicial: LED azul encendido al arrancar. */

    printf("=== Control RGB por temperatura NTC ===\n");              /* Titulo del sistema en el monitor serial. */
    printf("T < 25 C  -> LED Azul\n");                                /* Informa la condicion del rango frio. */
    printf("25 C <= T <= 35 C -> LED Verde\n");                       /* Informa la condicion del rango templado. */
    printf("T > 35 C  -> LED Rojo\n");                                /* Informa la condicion del rango caliente. */
    printf("NTC: GPIO%d | LED R:%d G:%d B:%d\n\n",                   /* Informa los pines utilizados. */
           NTC_ADC_CHANNEL, LED_RGB_RED_GPIO,
           LED_RGB_GREEN_GPIO, LED_RGB_BLUE_GPIO);

    while (1) {                                                       /* Inicia el bucle infinito del sistema embebido. */
        ESP_ERROR_CHECK(adc_oneshot_read(                             /* Lee el valor instantaneo de la NTC usando ADC oneshot. */
            adc_handle,                                              /* Entrega el manejador de la unidad ADC configurada. */
            NTC_ADC_CHANNEL,                                         /* Entrega el canal ADC conectado a la NTC. */
            &adc_raw                                                 /* Entrega la direccion donde se guardara la lectura cruda. */
        ));                                                          /* Cierra la lectura ADC y valida errores. */

        temperature = adc_raw_to_celsius(adc_raw);                   /* Convierte la lectura ADC a temperatura en grados Celsius. */

        apply_temperature_color(&led_rgb, temperature);              /* Actualiza el color del LED segun la temperatura medida. */

        if (print_counter >= PRINT_DELAY_CYCLES) {                   /* Revisa si es momento de imprimir el estado por consola. */
            print_counter = 0;                                       /* Reinicia el contador de impresion. */
            printf("ADC: %4d | Temp: %6.2f C | Color: %s\n",        /* Imprime el diagnostico del sistema. */
                   adc_raw,
                   temperature,
                   temperature_to_color_text(temperature));
        } else {                                                     /* Ejecuta esta rama cuando aun no toca imprimir. */
            print_counter++;                                         /* Aumenta el contador de ciclos sin imprimir. */
        }                                                            /* Termina la logica de impresion periodica. */

        vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));                    /* Pausa el ciclo 500 ms para estabilizar la lectura ADC. */
    }                                                                /* Termina una vuelta del bucle infinito. */
}                                                                    /* Termina la funcion app_main. */
