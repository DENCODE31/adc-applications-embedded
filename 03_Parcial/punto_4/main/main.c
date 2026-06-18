/* ============================================================
 *   Umbral por potenciómetro — Encendido de LED rojo
 *   ============================================================
 *   El potenciómetro se lee por ADC y se mapea a un valor de
 *   0 a 100 %. Ese valor es el "nivel" actual del sistema.
 *   El usuario fija un umbral (0-100) por comando UART y el LED
 *   rojo se enciende cuando el nivel del potenciómetro alcanza
 *   o supera ese umbral. Por UART se puede leer el umbral en
 *   cualquier momento y cambiarlo en caliente.
 *
 *   Comandos UART (escribir y presionar Enter):
 *     umbral        -> imprime el umbral actual y el nivel del pot
 *     set <0-100>   -> fija un nuevo umbral de encendido
 *   ============================================================ */

#include <stdio.h>                        /* printf: reportar estado por el monitor serial. */
#include <string.h>                       /* strncmp: parsear los comandos recibidos por UART. */
#include <stdlib.h>                        /* atoi: convertir el argumento numérico del comando "set". */
#include <stdint.h>                       /* uint32_t y tipos de tamaño fijo usados por FreeRTOS y LEDC. */

#include "freertos/FreeRTOS.h"            /* Base de FreeRTOS: debe incluirse antes que cualquier otra cabecera del RTOS. */
#include "freertos/task.h"                /* vTaskDelay: cede el CPU entre ciclos para no bloquear el scheduler. */

#include "driver/ledc.h"                  /* Constantes LEDC_CHANNEL_x y LEDC_TIMER_x del periférico PWM. */
#include "driver/uart.h"                  /* Driver UART: instalar puerto y leer los comandos del usuario. */

#include "esp_adc/adc_oneshot.h"          /* API oneshot: lectura ADC bajo demanda para el potenciómetro. */
#include "esp_err.h"                      /* ESP_ERROR_CHECK: detiene el sistema si una llamada de ESP-IDF falla. */

#include "library_led_c.h"                /* led_rgb_t y funciones de control PWM del LED RGB (usamos solo el rojo). */

/* ========================================================================== */
/*                              Pines del LED RGB                             */
/* ========================================================================== */

#define LED_RGB_RED_GPIO     (13)         /* GPIO del ánodo rojo: único canal que controla este programa. */
#define LED_RGB_GREEN_GPIO   (19)         /* GPIO del ánodo verde: configurado pero siempre apagado aquí. */
#define LED_RGB_BLUE_GPIO    (5)          /* GPIO del ánodo azul: configurado pero siempre apagado aquí. */

/* ========================================================================== */
/*                      Entrada analógica del potenciómetro                   */
/* ========================================================================== */

#define POT_ADC_UNIT         ADC_UNIT_1   /* Unidad ADC1 del ESP32-C6 que agrupa los canales disponibles. */
#define POT_ADC_CHANNEL      ADC_CHANNEL_1 /* Canal 1 de ADC1, conectado físicamente al GPIO1 del ESP32-C6. */
#define POT_ADC_ATTEN        ADC_ATTEN_DB_12 /* Atenuación 12 dB: cubre todo el recorrido del potenciómetro (0-3.3 V). */
#define POT_ADC_WIDTH        ADC_BITWIDTH_12 /* Resolución de 12 bits: valores crudos entre 0 y 4095. */
#define POT_ADC_MAX_RAW      (4095)       /* Lectura cruda máxima del ADC a 12 bits, usada para mapear a porcentaje. */

/*
 * Por qué oversampling: el ADC del ESP32-C6 tiene ruido de ±10-30 LSB, lo que
 * hace temblar el porcentaje del potenciómetro sin que el usuario lo mueva.
 * Promediar N muestras reduce el ruido en un factor de sqrt(N) y estabiliza el nivel.
 */
#define N_MUESTRAS_ADC       (16)         /* Número de muestras crudas que se promedian en cada lectura. */

/* ========================================================================== */
/*                         Configuración del comando UART                     */
/* ========================================================================== */

#define UART_NUM             UART_NUM_0   /* Puerto UART0: el mismo del monitor serial USB. */
#define UART_RX_BUF_SIZE     (256)        /* Tamaño del buffer de recepción del driver UART. */
#define CMD_BUF_SIZE         (64)         /* Tamaño máximo de una línea de comando del usuario. */

/* ========================================================================== */
/*                         Tiempos de ejecución                               */
/* ========================================================================== */

#define POLL_DELAY_MS        (100)        /* Pausa entre ciclos: 100 ms da respuesta ágil al mover el potenciómetro. */
#define PRINT_DELAY_CYCLES   (10)         /* Imprime el estado cada 10 ciclos (~1 s) para no saturar el serial. */

#define UMBRAL_POR_DEFECTO   (50)         /* Umbral inicial de encendido al arrancar, en porcentaje (0-100). */

/* ========================================================================== */
/*                       Configuración del ADC oneshot                        */
/* ========================================================================== */

static void config_pot_adc(adc_oneshot_unit_handle_t *adc_handle) /* Configura la unidad ADC y el canal del potenciómetro. */
{                                                                  /* Inicia el bloque de config_pot_adc. */
    adc_oneshot_unit_init_cfg_t init_cfg = {                       /* Estructura de inicialización de la unidad ADC. */
        .unit_id = POT_ADC_UNIT                                    /* Selecciona la unidad ADC1 donde está el canal del potenciómetro. */
    };                                                             /* Termina la configuración de la unidad. */

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, adc_handle));  /* Crea la unidad ADC y guarda su manejador. */

    adc_oneshot_chan_cfg_t chan_cfg = {                            /* Estructura de configuración del canal ADC. */
        .atten    = POT_ADC_ATTEN,                                 /* Atenuación para cubrir todo el rango del potenciómetro. */
        .bitwidth = POT_ADC_WIDTH                                  /* Resolución de 12 bits. */
    };                                                             /* Termina la configuración del canal. */

    ESP_ERROR_CHECK(adc_oneshot_config_channel(                    /* Aplica la configuración al canal físico del potenciómetro. */
        *adc_handle,                                               /* Manejador de la unidad ADC recién creada. */
        POT_ADC_CHANNEL,                                           /* Canal ADC1 conectado a GPIO1. */
        &chan_cfg                                                  /* Configuración de atenuación y resolución. */
    ));                                                            /* Cierra la configuración del canal. */
}                                                                  /* Termina la función config_pot_adc. */

/* ========================================================================== */
/*                 Lectura del potenciómetro mapeada a 0-100 %                 */
/* ========================================================================== */

static int leer_pot_porcentaje(adc_oneshot_unit_handle_t adc_handle) /* Promedia N lecturas crudas y las mapea a un porcentaje 0-100. */
{                                                                  /* Inicia el bloque de leer_pot_porcentaje. */
    int suma    = 0;                                               /* Acumulador de las N lecturas crudas. */
    int muestra = 0;                                               /* Almacena cada lectura cruda individual del ADC. */

    for (int i = 0; i < N_MUESTRAS_ADC; i++) {                     /* Repite la lectura N veces para promediar y reducir el ruido. */
        ESP_ERROR_CHECK(adc_oneshot_read(                          /* Toma una muestra cruda del canal del potenciómetro. */
            adc_handle,                                            /* Manejador de la unidad ADC. */
            POT_ADC_CHANNEL,                                       /* Canal conectado al potenciómetro. */
            &muestra                                               /* Destino de la lectura cruda (0 a 4095). */
        ));                                                        /* Cierra la lectura y valida errores. */
        suma += muestra;                                           /* Acumula la muestra para el promedio final. */
    }                                                              /* Termina el ciclo de oversampling. */

    int raw = suma / N_MUESTRAS_ADC;                               /* Promedio crudo: posición del potenciómetro sin ruido. */

    return (raw * 100) / POT_ADC_MAX_RAW;                          /* Mapea linealmente la posición cruda a un porcentaje 0-100. */
}                                                                  /* Termina la función leer_pot_porcentaje. */

/* ========================================================================== */
/*                   Lectura no bloqueante de comandos UART                    */
/* ========================================================================== */

/*
 * Lee del UART sin bloquear el bucle principal: acumula caracteres en 'buf'
 * hasta recibir un fin de línea ('\n' o '\r'). Devuelve 1 cuando hay una línea
 * completa lista para procesar, o 0 si todavía no llegó el Enter del usuario.
 */
static int leer_linea_uart(char *buf, int *len)                    /* Acumula caracteres de UART hasta completar una línea. */
{                                                                  /* Inicia el bloque de leer_linea_uart. */
    uint8_t c = 0;                                                 /* Carácter recibido del UART en esta iteración. */

    while (uart_read_bytes(UART_NUM, &c, 1, 0) == 1) {             /* Lee byte a byte con timeout 0 (no bloquea si no hay datos). */
        if (c == '\n' || c == '\r') {                              /* Detecta el fin de línea que marca el final del comando. */
            if (*len > 0) {                                        /* Solo cierra la línea si ya hay caracteres acumulados. */
                buf[*len] = '\0';                                  /* Termina la cadena para poder procesarla como texto. */
                *len = 0;                                          /* Reinicia el contador para el siguiente comando. */
                return 1;                                          /* Señala que hay una línea completa lista. */
            }                                                      /* Ignora finales de línea sueltos sin contenido previo. */
        } else if (*len < CMD_BUF_SIZE - 1) {                      /* Acumula el carácter si todavía cabe en el buffer. */
            buf[(*len)++] = (char)c;                               /* Guarda el carácter y avanza el índice de escritura. */
        }                                                          /* Descarta caracteres si el buffer ya está lleno. */
    }                                                              /* Termina mientras haya bytes disponibles en el UART. */

    return 0;                                                      /* No llegó aún el fin de línea: comando incompleto. */
}                                                                  /* Termina la función leer_linea_uart. */

/* ========================================================================== */
/*                       Procesamiento de cada comando                        */
/* ========================================================================== */

/*
 * Interpreta la línea recibida y actúa sobre el umbral:
 *   "umbral"      -> reporta umbral actual y nivel del potenciómetro
 *   "set <0-100>" -> valida y fija el nuevo umbral de encendido
 */
static void procesar_comando(char *linea, int *umbral, int nivel_pot) /* Ejecuta el comando del usuario sobre el umbral. */
{                                                                  /* Inicia el bloque de procesar_comando. */
    if (strncmp(linea, "umbral", 6) == 0) {                        /* Comando de lectura: el usuario quiere conocer el umbral. */
        printf("Umbral actual: %d %% | Nivel pot: %d %% | LED rojo: %s\n", /* Reporta umbral, nivel y estado del LED. */
               *umbral, nivel_pot, (nivel_pot >= *umbral) ? "ON" : "OFF");
    } else if (strncmp(linea, "set", 3) == 0) {                    /* Comando de escritura: el usuario quiere cambiar el umbral. */
        int valor = atoi(linea + 3);                               /* Convierte el texto después de "set" en el nuevo umbral. */
        if (valor < 0 || valor > 100) {                            /* Valida que el umbral quede dentro del rango permitido. */
            printf("Valor invalido. Usa: set <0-100>\n");          /* Avisa al usuario si el valor está fuera de rango. */
        } else {                                                   /* El valor es válido: se acepta el nuevo umbral. */
            *umbral = valor;                                       /* Actualiza el umbral de encendido en caliente. */
            printf("Nuevo umbral: %d %%\n", *umbral);              /* Confirma el cambio por el monitor serial. */
        }                                                          /* Termina la validación del comando set. */
    } else {                                                       /* La línea no coincide con ningún comando conocido. */
        printf("Comando no reconocido. Usa: umbral | set <0-100>\n"); /* Muestra la ayuda de comandos disponibles. */
    }                                                              /* Termina la selección de comando. */
}                                                                  /* Termina la función procesar_comando. */

/* ========================================================================== */
/*                              Programa principal                            */
/* ========================================================================== */

void app_main(void)                                                /* Función principal que ESP-IDF ejecuta al iniciar el sistema. */
{                                                                  /* Inicia el bloque de app_main. */
    led_rgb_t led_rgb = {                                          /* Estructura que describe la configuración del LED RGB (solo usamos rojo). */
        .led_red = {                                               /* Configuración del canal rojo. */
            .duty     = 0,                                         /* Rojo apagado al inicio. */
            .gpio_num = LED_RGB_RED_GPIO,                          /* GPIO físico del ánodo rojo. */
            .channel  = LEDC_CHANNEL_0                             /* Canal PWM 0 asignado al rojo. */
        },                                                         /* Termina la configuración del canal rojo. */
        .led_green = {                                             /* Configuración del canal verde (se mantiene apagado). */
            .duty     = 0,                                         /* Verde apagado. */
            .gpio_num = LED_RGB_GREEN_GPIO,                        /* GPIO físico del ánodo verde. */
            .channel  = LEDC_CHANNEL_1                             /* Canal PWM 1 asignado al verde. */
        },                                                         /* Termina la configuración del canal verde. */
        .led_blue = {                                              /* Configuración del canal azul (se mantiene apagado). */
            .duty     = 0,                                         /* Azul apagado. */
            .gpio_num = LED_RGB_BLUE_GPIO,                         /* GPIO físico del ánodo azul. */
            .channel  = LEDC_CHANNEL_2                             /* Canal PWM 2 asignado al azul. */
        },                                                         /* Termina la configuración del canal azul. */
        .timer           = LEDC_TIMER_0,                           /* Timer LEDC 0 compartido por los tres canales PWM. */
        .duty_resolution = LEDC_TIMER_13_BIT,                      /* 13 bits de resolución: 8192 niveles de brillo. */
        .frequency       = 4000,                                   /* 4000 Hz: frecuencia PWM por encima del parpadeo visible. */
        .speed_mode      = LEDC_LOW_SPEED_MODE                     /* Modo de baja velocidad: único disponible en ESP32-C6. */
    };                                                             /* Termina la estructura del LED RGB. */

    adc_oneshot_unit_handle_t adc_handle = NULL;                   /* Manejador de la unidad ADC, inicializado en NULL antes de configurar. */
    int  umbral          = UMBRAL_POR_DEFECTO;                     /* Umbral de encendido actual, en porcentaje (0-100). */
    int  nivel_pot       = 0;                                      /* Nivel actual del potenciómetro leído en cada ciclo (0-100). */
    char cmd_buf[CMD_BUF_SIZE] = {0};                              /* Buffer donde se arma la línea de comando recibida por UART. */
    int  cmd_len         = 0;                                      /* Cantidad de caracteres acumulados en el buffer de comando. */
    uint32_t contador    = 0;                                      /* Contador de ciclos para controlar la frecuencia de impresión. */

    config_pot_adc(&adc_handle);                                   /* Inicializa el ADC y configura el canal del potenciómetro. */
    config_led_rgb(&led_rgb);                                      /* Configura el timer LEDC y los canales PWM del LED RGB. */
    set_led_rgb_percentage_given_values(&led_rgb, 0, 0, 0);        /* Arranca con el LED rojo apagado. */

    ESP_ERROR_CHECK(uart_driver_install(                           /* Instala el driver UART0 para leer comandos del usuario. */
        UART_NUM,                                                  /* Puerto UART0 (el del monitor serial). */
        UART_RX_BUF_SIZE,                                          /* Tamaño del buffer de recepción. */
        0,                                                         /* Sin buffer de transmisión: printf maneja la salida. */
        0,                                                         /* Sin cola de eventos. */
        NULL,                                                      /* No se usa handle de cola de eventos. */
        0                                                          /* Sin flags de asignación de interrupción. */
    ));                                                            /* Cierra la instalación del driver UART. */

    printf("=== Umbral por potenciometro -> LED rojo ===\n");      /* Encabezado del sistema en el monitor serial. */
    printf("Pot: ADC_CH%d (GPIO1) mapeado a 0-100 %%\n", POT_ADC_CHANNEL); /* Informa el pin y mapeo del potenciómetro. */
    printf("LED rojo ON cuando nivel_pot >= umbral\n");            /* Explica la condición de encendido del LED. */
    printf("Comandos: 'umbral' (leer) | 'set <0-100>' (cambiar)\n"); /* Lista los comandos disponibles. */
    printf("Umbral inicial: %d %%\n\n", umbral);                   /* Informa el umbral con el que arranca el sistema. */

    while (1) {                                                    /* Bucle infinito principal del sistema embebido. */
        nivel_pot = leer_pot_porcentaje(adc_handle);               /* Lee y promedia el potenciómetro, mapeado a 0-100 %. */

        if (nivel_pot >= umbral) {                                 /* Comprueba si el nivel del potenciómetro alcanzó el umbral. */
            set_led_rgb_percentage_given_values(&led_rgb, 100, 0, 0); /* Enciende el LED rojo al 100 %. */
        } else {                                                   /* El nivel está por debajo del umbral. */
            set_led_rgb_percentage_given_values(&led_rgb, 0, 0, 0); /* Apaga el LED rojo. */
        }                                                          /* Termina la lógica de encendido del LED. */

        if (leer_linea_uart(cmd_buf, &cmd_len)) {                  /* Revisa si el usuario completó un comando por UART. */
            procesar_comando(cmd_buf, &umbral, nivel_pot);         /* Ejecuta el comando recibido sobre el umbral. */
        }                                                          /* Termina el manejo del comando UART. */

        if (contador >= PRINT_DELAY_CYCLES) {                      /* Revisa si pasaron suficientes ciclos para imprimir. */
            contador = 0;                                          /* Reinicia el contador de impresión. */
            printf("Nivel pot: %3d %% | Umbral: %3d %% | LED rojo: %s\n", /* Imprime el diagnóstico periódico. */
                   nivel_pot, umbral, (nivel_pot >= umbral) ? "ON" : "OFF");
        } else {                                                   /* No es momento de imprimir: solo incrementa el contador. */
            contador++;                                            /* Avanza el contador de ciclos sin impresión. */
        }                                                          /* Termina la lógica de impresión periódica. */

        vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));                  /* Pausa 100 ms: cede el CPU y permite atender otras tareas. */
    }                                                              /* Termina una vuelta del bucle. */
}                                                                  /* Termina la función app_main. */
