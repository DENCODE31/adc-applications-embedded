/* ============================================================
 *    SEGUNDO PUNTO LED RGB controlado por UART para rango e intensidad
 *   ============================================================

 *
 *   Comandos disponibles:
 *     ir N        -> intensidad del rojo  (0-100)
 *     ig N        -> intensidad del verde (0-100)
 *     ib N        -> intensidad del azul  (0-100)
 *     rr LO HI    -> rango del rojo  (0-100)
 *     rg LO HI    -> rango del verde (0-100)
 *     rb LO HI    -> rango del azul  (0-100)
 *     v N         -> fija el valor actual (tambien sirve escribir solo N)
 *     help        -> muestra esta ayuda
 *
 *   Ejemplo: "rr 0 10" y luego "5" enciende el rojo (5 esta en 0-10).
 *   Si los rangos se solapan, varios colores encienden a la vez,
 *   cada uno con su propia intensidad.
 *   ============================================================ */

#include <stdio.h>                     /* Permite usar printf para enviar mensajes al monitor serial. */
#include <string.h>                    /* Permite usar strncmp para comparar comandos de texto. */
#include <stdbool.h>                   /* Permite usar variables booleanas con los valores true y false. */
#include <stdint.h>                    /* Permite usar tipos enteros de tamano fijo como uint8_t. */

#include "freertos/FreeRTOS.h"         /* Incluye funciones base de FreeRTOS, como la conversion de milisegundos a ticks. */
#include "freertos/task.h"             /* Permite usar vTaskDelay para pausar la tarea principal. */

#include "driver/gpio.h"               /* Permite configurar y leer pines digitales GPIO. */
#include "driver/ledc.h"               /* Permite usar PWM mediante el periferico LEDC del ESP32. */
#include "driver/uart.h"               /* Permite leer comandos del usuario por la consola serial. */

#include "library_led_c.h"             /* Incluye las estructuras y funciones propias para controlar el LED RGB. */

/* ========================================================================== */
/*                              Pines del LED RGB                             */
/* ========================================================================== */

#define LED_RGB_1_RED_GPIO        (13)              /* Define el GPIO conectado al color rojo del LED RGB. */
#define LED_RGB_1_GREEN_GPIO      (19)              /* Define el GPIO conectado al color verde del LED RGB. */
#define LED_RGB_1_BLUE_GPIO       (5)               /* Define el GPIO conectado al color azul del LED RGB. */

/* ========================================================================== */
/*                         Consola serial (UART)                              */
/* ========================================================================== */

#define CONSOLE_UART_NUM          UART_NUM_0        /* Usa el UART0, el mismo puerto del monitor serial. */
#define CONSOLE_RX_BUFFER_SIZE    (256)             /* Tamano del buffer de recepcion del driver UART. */
#define COMMAND_LINE_MAX          (64)              /* Numero maximo de caracteres que se acumulan por comando. */

/* ========================================================================== */
/*                         Tiempos de ejecucion                               */
/* ========================================================================== */

#define POLL_DELAY_MS             (20)              /* Define cada cuantos milisegundos se revisa la consola. */

/* ========================================================================== */
/*                         Indices de cada color                              */
/* ========================================================================== */

#define COLOR_RED                 (0)               /* Posicion del rojo dentro de los arreglos de configuracion. */
#define COLOR_GREEN               (1)               /* Posicion del verde dentro de los arreglos de configuracion. */
#define COLOR_BLUE                (2)               /* Posicion del azul dentro de los arreglos de configuracion. */
#define COLOR_COUNT               (3)               /* Cantidad total de colores controlados. */

/* ========================================================================== */
/*                    Configuracion independiente por color                   */
/* ========================================================================== */

typedef struct {                                    /* Agrupa todo lo que define el comportamiento de un color. */
    int intensity;                                  /* Intensidad memorizada del color, de 0 a 100. */
    int range_low;                                  /* Limite inferior del rango que enciende el color. */
    int range_high;                                 /* Limite superior del rango que enciende el color. */
} color_config_t;                                   /* Nombra el tipo como color_config_t. */

/* ========================================================================== */
/*                    Limita un valor al rango 0 a 100                        */
/* ========================================================================== */

static int clamp_percentage(int value)               /* Declara una funcion que recorta cualquier valor al rango valido. */
{                                                    /* Inicia el bloque de codigo de clamp_percentage. */
    if (value < 0) {                                 /* Revisa si el valor recibido es negativo. */
        return 0;                                    /* Devuelve cero como minimo permitido. */
    }                                                /* Termina la validacion del minimo. */
    if (value > 100) {                               /* Revisa si el valor recibido supera el maximo. */
        return 100;                                  /* Devuelve cien como maximo permitido. */
    }                                                /* Termina la validacion del maximo. */
    return value;                                    /* Devuelve el valor sin cambios si ya estaba en rango. */
}                                                    /* Termina la funcion clamp_percentage. */

/* ========================================================================== */
/*                       Configuracion de la consola UART                     */
/* ========================================================================== */

static void config_console_uart(void)                /* Declara una funcion que prepara el UART para leer comandos. */
{                                                    /* Inicia el bloque de codigo de config_console_uart. */
    uart_driver_install(                             /* Instala el driver del UART para poder leer caracteres. */
        CONSOLE_UART_NUM,                            /* Selecciona el puerto UART de la consola. */
        CONSOLE_RX_BUFFER_SIZE,                      /* Define el tamano del buffer de recepcion. */
        0,                                           /* No usa buffer de transmision; printf escribe directo. */
        0,                                           /* No usa cola de eventos del UART. */
        NULL,                                        /* No entrega manejador de cola porque no se usa. */
        0                                            /* No reserva flags especiales de interrupcion. */
    );                                               /* Cierra la instalacion del driver UART. */
}                                                    /* Termina la funcion config_console_uart. */

/* ========================================================================== */
/*                          Ayuda de comandos                                 */
/* ========================================================================== */

static void print_help(void)                         /* Declara una funcion que muestra los comandos disponibles. */
{                                                    /* Inicia el bloque de codigo de print_help. */
    printf("\n--- Comandos disponibles ---\n");      /* Imprime el encabezado de la ayuda. */
    printf("ir N      intensidad rojo  (0-100)\n");  /* Explica el comando de intensidad del rojo. */
    printf("ig N      intensidad verde (0-100)\n");  /* Explica el comando de intensidad del verde. */
    printf("ib N      intensidad azul  (0-100)\n");  /* Explica el comando de intensidad del azul. */
    printf("rr LO HI  rango rojo  (0-100)\n");        /* Explica el comando de rango del rojo. */
    printf("rg LO HI  rango verde (0-100)\n");        /* Explica el comando de rango del verde. */
    printf("rb LO HI  rango azul  (0-100)\n");        /* Explica el comando de rango del azul. */
    printf("v N       fija el valor actual (o solo escribe N)\n"); /* Explica como mandar el valor. */
    printf("info      muestra el estado de cada color\n"); /* Explica el comando de informacion. */
    printf("help      muestra esta ayuda\n\n");      /* Explica el comando de ayuda. */
}                                                    /* Termina la funcion print_help. */

/* ========================================================================== */
/*                  Informacion del estado de cada color                      */
/* ========================================================================== */

static void print_info(color_config_t colors[COLOR_COUNT], int current_value) /* Muestra rango e intensidad de cada color. */
{                                                    /* Inicia el bloque de codigo de print_info. */
    printf("\n--- Estado actual ---\n");             /* Imprime el encabezado del reporte. */
    printf("Valor actual: %d\n", current_value);     /* Muestra el valor que el usuario fijo por UART. */
    printf("R[%d-%d]=%d%%\n", colors[COLOR_RED].range_low, colors[COLOR_RED].range_high, colors[COLOR_RED].intensity);       /* Estado del rojo. */
    printf("G[%d-%d]=%d%%\n", colors[COLOR_GREEN].range_low, colors[COLOR_GREEN].range_high, colors[COLOR_GREEN].intensity); /* Estado del verde. */
    printf("B[%d-%d]=%d%%\n\n", colors[COLOR_BLUE].range_low, colors[COLOR_BLUE].range_high, colors[COLOR_BLUE].intensity);  /* Estado del azul. */
}                                                    /* Termina la funcion print_info. */

/* ========================================================================== */
/*           Aplica el estado del LED segun el valor actual y los rangos      */
/* ========================================================================== */

static int brightness_for_color(const color_config_t *color, int value) /* Calcula el brillo final de un color. */
{                                                                       /* Inicia el bloque de brightness_for_color. */
    if (value >= color->range_low && value <= color->range_high) {      /* Revisa si el valor cae dentro del rango del color. */
        return color->intensity;                                        /* Enciende el color con su intensidad memorizada. */
    }                                                                   /* Termina el caso dentro del rango. */
    return 0;                                                           /* Apaga el color si el valor esta fuera de su rango. */
}                                                                       /* Termina la funcion brightness_for_color. */

static void apply_leds(led_rgb_t *led_rgb, color_config_t colors[COLOR_COUNT], int value) /* Actualiza los tres canales del LED. */
{                                                                                         /* Inicia el bloque de apply_leds. */
    set_led_rgb_percentage_given_values(                                                  /* Aplica el PWM segun el valor actual. */
        led_rgb,                                                                          /* Entrega la estructura con los canales LEDC. */
        brightness_for_color(&colors[COLOR_RED], value),                                  /* Rojo enciende si el valor esta en su rango. */
        brightness_for_color(&colors[COLOR_GREEN], value),                                /* Verde enciende si el valor esta en su rango. */
        brightness_for_color(&colors[COLOR_BLUE], value)                                  /* Azul enciende si el valor esta en su rango. */
    );                                                                                    /* Cierra la llamada que aplica los porcentajes RGB. */
}                                                                                         /* Termina la funcion apply_leds. */

/* ========================================================================== */
/*                       Procesa un comando del usuario                       */
/* ========================================================================== */

static void process_command(char *line, color_config_t colors[COLOR_COUNT], int *current_value) /* Interpreta una linea de texto. */
{                                                                           /* Inicia el bloque de codigo de process_command. */
    int low = 0;                                                            /* Guarda el limite inferior leido en comandos de rango. */
    int high = 0;                                                           /* Guarda el limite superior leido en comandos de rango. */
    int value = 0;                                                          /* Guarda el numero leido en comandos de intensidad o valor. */

    if (sscanf(line, "ir %d", &value) == 1) {                               /* Detecta el comando de intensidad del rojo. */
        colors[COLOR_RED].intensity = clamp_percentage(value);              /* Guarda la nueva intensidad del rojo de forma independiente. */
        printf("Intensidad rojo = %d%%\n", colors[COLOR_RED].intensity);    /* Confirma el cambio por consola. */
    } else if (sscanf(line, "ig %d", &value) == 1) {                        /* Detecta el comando de intensidad del verde. */
        colors[COLOR_GREEN].intensity = clamp_percentage(value);            /* Guarda la nueva intensidad del verde de forma independiente. */
        printf("Intensidad verde = %d%%\n", colors[COLOR_GREEN].intensity); /* Confirma el cambio por consola. */
    } else if (sscanf(line, "ib %d", &value) == 1) {                        /* Detecta el comando de intensidad del azul. */
        colors[COLOR_BLUE].intensity = clamp_percentage(value);             /* Guarda la nueva intensidad del azul de forma independiente. */
        printf("Intensidad azul = %d%%\n", colors[COLOR_BLUE].intensity);   /* Confirma el cambio por consola. */
    } else if (sscanf(line, "rr %d %d", &low, &high) == 2) {                /* Detecta el comando de rango del rojo. */
        colors[COLOR_RED].range_low = clamp_percentage(low);                /* Guarda el limite inferior del rango rojo. */
        colors[COLOR_RED].range_high = clamp_percentage(high);              /* Guarda el limite superior del rango rojo. */
        printf("Rango rojo = [%d, %d]\n", colors[COLOR_RED].range_low, colors[COLOR_RED].range_high); /* Confirma el cambio. */
    } else if (sscanf(line, "rg %d %d", &low, &high) == 2) {                /* Detecta el comando de rango del verde. */
        colors[COLOR_GREEN].range_low = clamp_percentage(low);              /* Guarda el limite inferior del rango verde. */
        colors[COLOR_GREEN].range_high = clamp_percentage(high);            /* Guarda el limite superior del rango verde. */
        printf("Rango verde = [%d, %d]\n", colors[COLOR_GREEN].range_low, colors[COLOR_GREEN].range_high); /* Confirma el cambio. */
    } else if (sscanf(line, "rb %d %d", &low, &high) == 2) {                /* Detecta el comando de rango del azul. */
        colors[COLOR_BLUE].range_low = clamp_percentage(low);               /* Guarda el limite inferior del rango azul. */
        colors[COLOR_BLUE].range_high = clamp_percentage(high);             /* Guarda el limite superior del rango azul. */
        printf("Rango azul = [%d, %d]\n", colors[COLOR_BLUE].range_low, colors[COLOR_BLUE].range_high); /* Confirma el cambio. */
    } else if (sscanf(line, "v %d", &value) == 1) {                         /* Detecta el comando que fija el valor con prefijo. */
        *current_value = clamp_percentage(value);                           /* Guarda el nuevo valor actual recortado al rango valido. */
        printf("Valor actual = %d\n", *current_value);                      /* Confirma el nuevo valor por consola. */
    } else if (sscanf(line, "%d", &value) == 1) {                           /* Detecta cuando el usuario escribe solo un numero. */
        *current_value = clamp_percentage(value);                           /* Usa ese numero como valor actual. */
        printf("Valor actual = %d\n", *current_value);                      /* Confirma el nuevo valor por consola. */
    } else if (strncmp(line, "info", 4) == 0) {                             /* Detecta el comando de informacion. */
        print_info(colors, *current_value);                                 /* Muestra el estado de cada color. */
    } else if (strncmp(line, "help", 4) == 0) {                             /* Detecta el comando de ayuda. */
        print_help();                                                       /* Muestra la lista de comandos. */
    } else {                                                                /* Atiende cualquier texto que no sea un comando valido. */
        printf("Comando no reconocido: '%s'. Escribe help.\n", line);       /* Avisa al usuario y sugiere ver la ayuda. */
    }                                                                       /* Termina la cadena de comparaciones de comandos. */
}                                                                           /* Termina la funcion process_command. */

/* ========================================================================== */
/*           Lee la consola sin bloquear y arma una linea de comando          */
/* ========================================================================== */

static bool poll_console(color_config_t colors[COLOR_COUNT], int *current_value) /* Revisa si llego texto y devuelve si hubo comando. */
{                                                            /* Inicia el bloque de codigo de poll_console. */
    static char command_buffer[COMMAND_LINE_MAX];            /* Acumula los caracteres del comando entre llamadas. */
    static int command_length = 0;                           /* Recuerda cuantos caracteres llevamos acumulados. */
    uint8_t incoming = 0;                                    /* Guarda el caracter recibido en cada lectura. */
    bool processed = false;                                  /* Indica si se proceso al menos un comando completo. */

    while (uart_read_bytes(CONSOLE_UART_NUM, &incoming, 1, 0) > 0) { /* Lee un caracter a la vez sin esperar si no hay datos. */
        if (incoming == '\n' || incoming == '\r') {                 /* Revisa si el caracter cierra la linea de comando. */
            if (command_length > 0) {                               /* Procesa solo si la linea tiene contenido. */
                command_buffer[command_length] = '\0';              /* Cierra la cadena para poder leerla como texto. */
                process_command(command_buffer, colors, current_value); /* Interpreta y aplica el comando recibido. */
                command_length = 0;                                 /* Reinicia el buffer para el siguiente comando. */
                processed = true;                                   /* Marca que hubo un comando para refrescar el LED. */
            }                                                       /* Termina el procesamiento de la linea completa. */
        } else if (command_length < (COMMAND_LINE_MAX - 1)) {       /* Revisa que aun quepa el caracter en el buffer. */
            command_buffer[command_length++] = (char)incoming;      /* Guarda el caracter y avanza la posicion. */
        }                                                           /* Termina el caso de acumulacion de caracteres. */
    }                                                               /* Termina la lectura de todos los caracteres pendientes. */

    return processed;                                               /* Devuelve si se debe refrescar el LED. */
}                                                                   /* Termina la funcion poll_console. */

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

    /* Configuracion inicial de cada color: intensidad y rango por defecto.
     * Los rangos parten divididos en tercios y se pueden cambiar por UART.
     * La intensidad arranca en 100% y cada color la guarda de forma independiente. */
    color_config_t colors[COLOR_COUNT] = {               /* Crea el arreglo con la configuracion de los tres colores. */
        [COLOR_RED]   = { .intensity = 100, .range_low = 0,  .range_high = 10 }, /* Rojo enciende de 0 a 10. */
        [COLOR_GREEN] = { .intensity = 100, .range_low = 10, .range_high = 20 }, /* Verde enciende de 10 a 20. */
        [COLOR_BLUE]  = { .intensity = 100, .range_low = 20, .range_high = 30 }  /* Azul enciende de 20 a 30. */
    };                                                   /* Termina la inicializacion del arreglo de colores. */

    int current_value = 0;                               /* Guarda el valor actual que el usuario manda por UART. */

    config_led_rgb(&led_rgb1);                           /* Configura el timer y los canales PWM del LED RGB. */
    config_console_uart();                               /* Prepara el UART para recibir comandos del usuario. */
    apply_leds(&led_rgb1, colors, current_value);        /* Aplica el estado inicial del LED segun el valor de arranque. */

    printf("Sistema RGB por UART listo (sin potenciometro).\n"); /* Informa que el programa arranco correctamente. */
    printf("Configura rangos e intensidades, luego manda un valor.\n"); /* Explica el flujo de uso. */
    print_help();                                        /* Muestra los comandos disponibles al arrancar. */

    while (1) {                                          /* Inicia el bucle infinito del sistema embebido. */
        if (poll_console(colors, &current_value)) {      /* Revisa la consola y aplica los comandos pendientes. */
            apply_leds(&led_rgb1, colors, current_value); /* Refresca el LED solo cuando hubo un comando nuevo. */
        }                                                /* Termina el refresco por comando. */

        vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));        /* Pausa el ciclo para reducir carga de CPU. */
    }                                                    /* Termina una vuelta del bucle infinito y vuelve al inicio. */
}                                                        /* Termina la funcion app_main. */
