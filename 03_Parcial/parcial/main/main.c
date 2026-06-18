/* ============================================================
 *   PARCIAL — Sistemas en Tiempo Real — ESP32-C6 - Dénnir
 *   ============================================================
 *   Integra los cuatro puntos del parcial en un solo programa:
 *
 *   PUNTO 1: NTC termistor → temperatura reportada por UART
 *            cada X segundos. Intervalo y unidad configurables
 *            por comandos UART.
 *
 *   PUNTO 2: LED RGB controlado por rangos de temperatura (en C)
 *            configurables via UART. La NTC dispara los colores:
 *            si temp cae en el rango de un color, ese color enciende
 *            con su intensidad configurada. Rangos pueden solaparse.
 *
 *   PUNTO 3: Pulsador GPIO18 alterna la unidad de impresion
 *            entre Celsius y Fahrenheit (ISR + debounce 200 ms).
 *
 *   PUNTO 4: Potenciometro (GPIO1/ADC_CH1) define el umbral de
 *            temperatura: si NTC >= pot% -> LED rojo con
 *            prioridad maxima sobre los rangos de P2.
 *
 *   ─────────────────────────────────────────────────────────
 *   Comandos UART
 *
 *     Temperatura (P1):
 *       c             -> grados Celsius
 *       f             -> grados Fahrenheit
 *       t <seg>       -> intervalo de impresion (1-3600 s)
 *
 *     LED por rangos de temperatura (P2):
 *       ir N          -> intensidad rojo   (0-100)
 *       ig N          -> intensidad verde  (0-100)
 *       ib N          -> intensidad azul   (0-100)
 *       rr LO HI      -> rango rojo   en grados C
 *       rg LO HI      -> rango verde  en grados C
 *       rb LO HI      -> rango azul   en grados C
 *       info 2        -> estado de rangos e intensidades
 *       info          -> intervalo de impresion y unidad activa (P1)
 *
 *     Potenciometro (P4):
 *       pot           -> leer nivel actual del pot y comparacion con NTC
 *
 *     General:
 *       ? / help      -> ayuda completa
 *
 *   ============================================================ */

#include <stdio.h>               /* printf: reportar estado por el monitor serial.                  */
#include <string.h>              /* strncmp, memset: comparar y limpiar cadenas de comandos.         */
#include <stdlib.h>              /* atoi: convertir texto numerico de los comandos.                  */
#include <ctype.h>               /* tolower: aceptar comandos en mayuscula o minuscula.              */
#include <math.h>                /* logf: requerida por la ecuacion Beta de la NTC.                  */
#include <stdint.h>              /* uint32_t y tipos de tamano fijo.                                 */
#include <stdbool.h>             /* bool, true, false.                                               */

#include "freertos/FreeRTOS.h"   /* Base de FreeRTOS: debe incluirse antes de las demas cabeceras.   */
#include "freertos/task.h"       /* xTaskGetTickCountFromISR: marca de tiempo para debounce del boton.*/

#include "driver/uart.h"         /* Driver UART: recibir comandos del usuario por el monitor serial. */
#include "driver/gpio.h"         /* gpio_config, gpio_isr_handler_add: pulsador P3.                 */
#include "driver/ledc.h"         /* Constantes LEDC_CHANNEL_x y LEDC_TIMER_x del periferico PWM.    */

#include "esp_adc/adc_oneshot.h"     /* API oneshot: lecturas ADC bajo demanda (NTC y pot).         */
#include "esp_adc/adc_cali.h"        /* adc_cali_raw_to_voltage: voltaje calibrado de la NTC.       */
#include "esp_adc/adc_cali_scheme.h" /* adc_cali_create_scheme_curve_fitting: calibracion ESP32-C6. */
#include "esp_timer.h"           /* Temporizador de software periodico para impresion de temp (P1). */
#include "esp_err.h"             /* ESP_ERROR_CHECK: aborta si una llamada de ESP-IDF falla.         */
#include "esp_attr.h"            /* IRAM_ATTR: coloca la ISR del boton en RAM interna.               */

#include "library_led_c.h"       /* led_rgb_t y funciones de control PWM del LED RGB.               */


/*                              Pines del LED RGB (P2/P3/P4)                  */


#define LED_RGB_RED_GPIO     (13)         /* GPIO del anodo rojo del LED RGB.   */
#define LED_RGB_GREEN_GPIO   (19)         /* GPIO del anodo verde del LED RGB.  */
#define LED_RGB_BLUE_GPIO    (5)          /* GPIO del anodo azul del LED RGB.   */


/*                     configuracion del ADC para la NTC (Punto 1/Punto 3)               */


// configuracion del ADC para la NTC: canal 4 de ADC1, atenuacion 12 dB, resolucion 12 bits.

#define NTC_ADC_UNIT         ADC_UNIT_1       /* Unidad ADC1 del ESP32-C6.                           */
#define NTC_ADC_CHANNEL      ADC_CHANNEL_4    /* Canal 4 de ADC1 -> GPIO4.                           */
#define NTC_ADC_ATTEN        ADC_ATTEN_DB_12  /* Atenuacion 12 dB: mide hasta ~3.1 V.                */
#define NTC_ADC_WIDTH        ADC_BITWIDTH_12  /* Resolucion 12 bits: valores 0-4095.                 */
#define N_MUESTRAS_NTC       (16)             /* Muestras promediadas para reducir ruido del ADC.    */


/*         Parametros de la NTC para convertir el voltaje medido a temperatura:       */

#define NTC_BETA             (3950.0f)        /* Coeficiente Beta de la NTC 5 kohm.                  */
#define NTC_R0               (5000.0f)        /* Resistencia nominal a 25 C en ohms.                 */
#define NTC_T0_KELVIN        (298.15f)        /* Temperatura nominal en Kelvin (25 C + 273.15).      */
#define NTC_R_SERIE          (10000.0f)       /* Resistencia en serie del divisor en ohms.           */
#define NTC_VCC_MV           (3300)           /* Tension de alimentacion del divisor en mV.          */


/*                     configuracion del ADC para el Potenciometro (Punto 4)                  */

#define POT_ADC_CHANNEL      ADC_CHANNEL_1    /* Canal 1 de ADC1 -> GPIO1.                           */
#define POT_ADC_ATTEN        ADC_ATTEN_DB_12  /* Atenuacion 12 dB: cubre todo el recorrido del pot.  */
#define POT_ADC_WIDTH        ADC_BITWIDTH_12  /* Resolucion 12 bits.                                 */
#define POT_ADC_MAX_RAW      (4095)           /* Lectura cruda maxima a 12 bits.                     */
#define N_MUESTRAS_POT       (16)             /* Muestras promediadas para estabilizar el nivel.     */


/*                         Configuracion del UART (Punto 1/Punto 2/Punto 4)                  */


#define UART_PORT            UART_NUM_0   /* UART0: el mismo que usa el monitor serial por USB.      */
#define UART_BAUD_RATE       (115200)     /* Velocidad estandar del monitor serial de ESP-IDF.       */
#define UART_RX_BUF_SIZE     (256)        /* Tamano del buffer de recepcion del driver UART.         */
#define CMD_BUF_SIZE         (64)         /* Longitud maxima de una linea de comando del usuario.    */


/*                         Pulsador para C/F (Punto 3)                             */


#define BUTTON_GPIO          (18)         /* GPIO del pulsador que alterna la unidad C <-> F.        */
#define BUTTON_DEBOUNCE_MS   (200)        /* evita mulitples activaciones falsas espera ese tiempo antes de aceptar el cambio         */


/*                    Umbrales de temperatura con histeresis (Punto 3)              */


/*
 * Histeresis de +/-0.5 C alrededor de cada umbral:
 *   AZUL  -> VERDE : T >= 25.5 C
 *   VERDE -> AZUL  : T <  24.5 C
 *   VERDE -> ROJO  : T >  35.5 C
 *   ROJO  -> VERDE : T <= 34.5 C
 */

#define TEMP_AZUL_A_VERDE    (25.5f)
#define TEMP_VERDE_A_AZUL    (24.5f)
#define TEMP_VERDE_A_ROJO    (35.5f)
#define TEMP_ROJO_A_VERDE    (34.5f)


/*                     Parametros de tiempo y ciclos de muestreo               */


#define POLL_DELAY_MS        (20)         // los Tick del bucle: timeout de lectura UART. 
#define ADC_PERIOD_CYCLES    (25)         // Leo NTC cada 25 ticks x 20 ms~500 ms.                
#define POT_PERIOD_CYCLES    (5)          // Leo pot cada  5 ticks x 20 ms~100 ms.                

#define INTERVALO_DEFECTO_S  (2)          // Intervalo inicial de impresion de temperatura (P1).   
#define INTERVALO_MIN_S      (1)          // Intervalo minimo permitido.                      
#define INTERVALO_MAX_S      (3600)       // Intervalo maximo permitido (1 hora).                    

/*                         Indices de color para Punto 2                           */

#define COLOR_RED_IDX   (0) //
#define COLOR_GREEN_IDX (1)
#define COLOR_BLUE_IDX  (2)
#define COLOR_COUNT     (3)

/*                              Tipos enumerados                               */


typedef enum {                            /* Unidad de temperatura seleccionada.                     */
    UNIDAD_CELSIUS = 0,
    UNIDAD_FAHRENHEIT
} unidad_temp_t;

typedef enum {                            /* Estado de color para reporte serial de temperatura (P3). */
    COLOR_AZUL  = 0,                      /* T < 25 C                                                */
    COLOR_VERDE,                          /* 25 C <= T <= 35 C                                       */
    COLOR_ROJO                            /* T > 35 C                                                */
} color_estado_t;

/*              Estructura de configuracion de color por rangos (Punto 2)          */

typedef struct {
    int intensity;    /* Intensidad del color en % (0-100) memorizada por separado.  */
    int range_low;    /* Limite inferior del rango que enciende el color.             */
    int range_high;   /* Limite superior del rango que enciende el color.             */
} color_config_t;

/*                         Estado global compartido                            */

static volatile unidad_temp_t g_unidad      = UNIDAD_CELSIUS; /* Unidad activa; arranca en Celsius. */
static volatile bool          g_imprimir_ya = false;          /* Timer P1 levanta esta bandera.     */

/* Pulsador P3. */
static volatile bool     g_boton_evento  = false;             /* ISR levanta esta bandera.           */
static volatile uint32_t g_ultimo_isr_ms = 0;                 /* Ultima pulsacion aceptada (debounce).*/

/*                    Callback del temporizador de impresion (Punto 1)              */

static void timer_imprimir_cb(void *arg)                      /* Ejecutado por esp_timer cada X seg. */
{
    g_imprimir_ya = true;                                      /* Le dice al bucle que toca imprimir. */
}

/*                          ISR del pulsador (Punto 3)                              */

static void IRAM_ATTR boton_isr_handler(void *arg)            /* Se ejecuta en flanco de bajada.     */
{
    uint32_t ahora = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    if ((ahora - g_ultimo_isr_ms) >= BUTTON_DEBOUNCE_MS) {    /* Filtra rebotes mecanicos.           */
        g_ultimo_isr_ms = ahora;
        g_boton_evento  = true;                                /* Senala al bucle la pulsacion valida.*/
    }
}

/*             Configuracion del ADC (ADC_UNIT_1: canales NTC y pot)          */

static void config_adc(
    adc_oneshot_unit_handle_t *adc_handle,
    adc_cali_handle_t         *cali_handle)
{

    /* Crear la unidad ADC1 compartida por NTC (CH4) y potenciometro (CH1).    */
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = NTC_ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, adc_handle));


    /* Canal 4: NTC termistor.                                                  */
    adc_oneshot_chan_cfg_t chan_ntc = {
        .atten    = NTC_ADC_ATTEN,
        .bitwidth = NTC_ADC_WIDTH
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_handle, NTC_ADC_CHANNEL, &chan_ntc));

    /* Canal 1: Potenciometro.                                                  */
    adc_oneshot_chan_cfg_t chan_pot = {
        .atten    = POT_ADC_ATTEN,
        .bitwidth = POT_ADC_WIDTH
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_handle, POT_ADC_CHANNEL, &chan_pot));

    /* Calibracion curve-fitting para el canal de la NTC (solo la NTC la usa). */
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = NTC_ADC_UNIT,
        .chan     = NTC_ADC_CHANNEL,
        .atten    = NTC_ADC_ATTEN,
        .bitwidth = NTC_ADC_WIDTH
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, cali_handle));
}



/*                    Lectura de la NTC con calibracion (Punto 1/Punto 3)                */

/* ========================================================================== */

/*                  Defino Polling ADC NTC (Punto 1/Punto 3)                  */

/* ========================================================================== */
static int leer_voltaje_ntc_mv(
    adc_oneshot_unit_handle_t adc_handle,
    adc_cali_handle_t         cali_handle)
{
    int suma = 0, muestra = 0, voltage_mv = 0;

    for (int i = 0; i < N_MUESTRAS_NTC; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, NTC_ADC_CHANNEL, &muestra));
        suma += muestra;
    }
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, suma / N_MUESTRAS_NTC, &voltage_mv));
    return voltage_mv;
}

static float voltaje_mv_a_celsius(int voltage_mv)
{
    if (voltage_mv <= 0)          voltage_mv = 1;
    if (voltage_mv >= NTC_VCC_MV) voltage_mv = NTC_VCC_MV - 1;

    float r_ntc = NTC_R_SERIE * (float)voltage_mv / (float)(NTC_VCC_MV - voltage_mv);
    float inv_T = (1.0f / NTC_T0_KELVIN) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0);
    return (1.0f / inv_T) - 273.15f;
}

/* ========================================================================== */

/*                  Defin Polling ADC Potenciometro (Punto 4)                */

/* ========================================================================== */

static int leer_pot_porcentaje(adc_oneshot_unit_handle_t adc_handle)
{
    int suma = 0, muestra = 0;

    for (int i = 0; i < N_MUESTRAS_POT; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, POT_ADC_CHANNEL, &muestra));
        suma += muestra;
    }
    int raw = suma / N_MUESTRAS_POT;
    return (raw * 100) / POT_ADC_MAX_RAW;
}


/*          Maquina de estados de temperatura con histeresis (Punto 3)      */


static color_estado_t actualizar_estado_temp(color_estado_t estado, float temperatura)
{
    switch (estado) {
    case COLOR_AZUL:
        if (temperatura >= TEMP_AZUL_A_VERDE) return COLOR_VERDE;
        break;
    case COLOR_VERDE:
        if (temperatura <  TEMP_VERDE_A_AZUL) return COLOR_AZUL;
        if (temperatura >  TEMP_VERDE_A_ROJO) return COLOR_ROJO;
        break;
    case COLOR_ROJO:
        if (temperatura <= TEMP_ROJO_A_VERDE) return COLOR_VERDE;
        break;
    }
    return estado;
}

static const char *estado_a_texto(color_estado_t estado)
{
    switch (estado) {
    case COLOR_AZUL:  return "Azul  (T<25C)";
    case COLOR_VERDE: return "Verde (25<=T<=35C)";
    case COLOR_ROJO:  return "Rojo  (T>35C)";
    default:          return "?";
    }
}


/*                Auxiliares para LED por rangos de temperatura (Punto 2)          */


static int clamp100(int v)
{
    if (v <   0) return 0;
    if (v > 100) return 100;
    return v;
}

static int brillo_color(const color_config_t *color, int value)
{
    if (value >= color->range_low && value <= color->range_high) return color->intensity;
    return 0;
}

static void aplicar_leds_manual(led_rgb_t *led_rgb,
                                 color_config_t colors[COLOR_COUNT],
                                 int value)
{
    set_led_rgb_percentage_given_values(led_rgb,
        brillo_color(&colors[COLOR_RED_IDX],   value),
        brillo_color(&colors[COLOR_GREEN_IDX], value),
        brillo_color(&colors[COLOR_BLUE_IDX],  value));
}


/*                       Configuracion del pulsador (Punto 3)                      */


static void config_boton(void)
{
    gpio_config_t boton_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),// Mascara de bits para el GPIO del boton.
        .mode         = GPIO_MODE_INPUT,// Configurado como entrada.
        .pull_up_en   = GPIO_PULLUP_ENABLE,// Habilitar resistencia de pull-up.
        .pull_down_en = GPIO_PULLDOWN_DISABLE,// Deshabilitar resistencia de pull-down.
        .intr_type    = GPIO_INTR_NEGEDGE   /* Flanco de bajada: momento exacto de presionar.  */
    };
    ESP_ERROR_CHECK(gpio_config(&boton_cfg)); 
    ESP_ERROR_CHECK(gpio_install_isr_service(0));          /* Solo se instala una vez en el sistema.  */
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, boton_isr_handler, NULL));
}

/* ========================================================================== */

/*                  Defin Polling UART (Punto 1/Punto 2/Punto 4)             */

/* ========================================================================== */

static void config_uart(void)//
{
    uart_config_t uart_cfg = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_cfg));
}


/*                       Ayuda de comandos combinada                           */


static void imprimir_ayuda(void)
{
    printf("\n=== Comandos UART disponibles ===\n");
    printf("-- Temperatura (P1) --\n");
    printf("  c             -> Celsius\n");
    printf("  f             -> Fahrenheit\n");
    printf("  t <seg>       -> intervalo de impresion (%d-%d s)\n",
           INTERVALO_MIN_S, INTERVALO_MAX_S);
    printf("-- LED por rangos de temperatura (P2) --\n");
    printf("  La NTC dispara los colores segun los rangos en grados C.\n");
    printf("  Si la temp cae en dos rangos solapados, ambos colores encienden.\n");
    printf("  ir N          -> intensidad rojo   (0-100 %%)\n");
    printf("  ig N          -> intensidad verde  (0-100 %%)\n");
    printf("  ib N          -> intensidad azul   (0-100 %%)\n");
    printf("  rr LO HI      -> rango rojo   en grados C (ej: rr 30 60)\n");
    printf("  rg LO HI      -> rango verde  en grados C (ej: rg 20 30)\n");
    printf("  rb LO HI      -> rango azul   en grados C (ej: rb 0 20)\n");
    printf("  info 2        -> estado de rangos, intensidades y temp actual\n");
    printf("  info          -> intervalo de impresion y unidad activa (P1)\n");
    printf("-- Potenciometro (P4) --\n");
    printf("  pot           -> nivel actual del pot (%%) y temperatura NTC\n");
    printf("  El pot define el umbral en C: si NTC >= pot%%, LED rojo.\n");
    printf("-- General --\n");
    printf("  ? / help      -> esta ayuda\n");
    printf("NOTA: GPIO%d -> pulsador alterna C/F sin UART\n\n", BUTTON_GPIO);
}

/* ========================================================================== */
/*                    Procesador de comandos UART combinado                    */
/* ========================================================================== */

/*
 * Devuelve el nuevo intervalo en segundos si el usuario lo cambio (Punto 1),
 * o 0 si el comando no modifica el intervalo.
 */

static int procesar_comando(
    const char        *linea,
    color_config_t     colors[COLOR_COUNT],
    float             *temp_actual,     /* Temperatura NTC actual en C (para mostrar en info). */
    int                nivel_pot,
    int               *intervalo_actual) /* Intervalo de impresion actual en segundos (P1).    */
{
    int v = 0, lo = 0, hi = 0;
    char inicial = (char)tolower((unsigned char)linea[0]);

    /* ── Temperatura (P1) ─────────────────────────────────────── */

    if (inicial == 'c' && (linea[1] == '\0' || linea[1] == ' ')) {
        g_unidad = UNIDAD_CELSIUS;
        printf("Unidad -> Celsius\n");
        return 0;
    }
    if (inicial == 'f' && (linea[1] == '\0' || linea[1] == ' ')) {
        g_unidad = UNIDAD_FAHRENHEIT;
        printf("Unidad -> Fahrenheit\n");
        return 0;
    }
    if (inicial == 't' && sscanf(linea + 1, " %d", &v) == 1) {
        if (v < INTERVALO_MIN_S || v > INTERVALO_MAX_S) {
            printf("Intervalo invalido. Use %d-%d s.\n", INTERVALO_MIN_S, INTERVALO_MAX_S);
            return 0;
        }
        printf("Intervalo -> %d s\n", v);
        return v;   /* Senal para reconfigurar el timer. */
    }
    if (inicial == '?' || strncmp(linea, "help", 4) == 0) {
        imprimir_ayuda();
        return 0;
    }

    /* ── LED por rangos de temperatura (P2) ─────────────────────── */

    if (sscanf(linea, "ir %d", &v) == 1) {// Intensidad del rojo.
        colors[COLOR_RED_IDX].intensity = clamp100(v);
        printf("Intensidad rojo = %d%%\n", colors[COLOR_RED_IDX].intensity);
        return 0;
    }
    if (sscanf(linea, "ig %d", &v) == 1) {// Intensidad del verde.
        colors[COLOR_GREEN_IDX].intensity = clamp100(v);
        printf("Intensidad verde = %d%%\n", colors[COLOR_GREEN_IDX].intensity);
        return 0;
    }
    if (sscanf(linea, "ib %d", &v) == 1) {// Intensidad del azul.
        colors[COLOR_BLUE_IDX].intensity = clamp100(v);
        printf("Intensidad azul = %d%%\n", colors[COLOR_BLUE_IDX].intensity);
        return 0;
    }
    if (sscanf(linea, "rr %d %d", &lo, &hi) == 2) {// Rango del rojo.
        colors[COLOR_RED_IDX].range_low  = clamp100(lo);
        colors[COLOR_RED_IDX].range_high = clamp100(hi);
        printf("Rango rojo = [%d, %d]\n",
               colors[COLOR_RED_IDX].range_low, colors[COLOR_RED_IDX].range_high);
        return 0;
    }
    if (sscanf(linea, "rg %d %d", &lo, &hi) == 2) {// Rango del verde.
        colors[COLOR_GREEN_IDX].range_low  = clamp100(lo);
        colors[COLOR_GREEN_IDX].range_high = clamp100(hi);
        printf("Rango verde = [%d, %d]\n",
               colors[COLOR_GREEN_IDX].range_low, colors[COLOR_GREEN_IDX].range_high);
        return 0;
    }
    if (sscanf(linea, "rb %d %d", &lo, &hi) == 2) {// Rango del azul.
        colors[COLOR_BLUE_IDX].range_low  = clamp100(lo);
        colors[COLOR_BLUE_IDX].range_high = clamp100(hi);
        printf("Rango azul = [%d, %d]\n",
               colors[COLOR_BLUE_IDX].range_low, colors[COLOR_BLUE_IDX].range_high);
        return 0;
    }
    if (strncmp(linea, "info 2", 6) == 0) {// Estado de los rangos, intensidades y temperatura actual.
        printf("\n--- Estado LED por rangos (Punto 2) ---\n");
        printf("La NTC dispara los rangos (temperatura actual = %.1f C)\n", *temp_actual);
        printf("R[%d-%d C]=%d%%  G[%d-%d C]=%d%%  B[%d-%d C]=%d%%\n\n",
               colors[COLOR_RED_IDX].range_low,   colors[COLOR_RED_IDX].range_high,
               colors[COLOR_RED_IDX].intensity,
               colors[COLOR_GREEN_IDX].range_low,  colors[COLOR_GREEN_IDX].range_high,
               colors[COLOR_GREEN_IDX].intensity,
               colors[COLOR_BLUE_IDX].range_low,   colors[COLOR_BLUE_IDX].range_high,
               colors[COLOR_BLUE_IDX].intensity);
        return 0;
    }
    if (strncmp(linea, "info", 4) == 0) {
        printf("\n--- Estado impresion (P1) ---\n");
        printf("Intervalo: %d s | Unidad: %s\n\n",
               *intervalo_actual,
               (g_unidad == UNIDAD_CELSIUS) ? "Celsius" : "Fahrenheit");
        return 0;
    }

    /* Nota: en P2 la temperatura de la NTC dispara los rangos automaticamente.
     * No hay comando de valor manual; los rangos se configuran en grados Celsius. */

    /* ── Potenciometro (Punto 4) ───────────────────────────────────── */

    if (strncmp(linea, "pot", 3) == 0) {
        printf("Pot: %d%% (umbral de temp) | NTC: %.1f C | LED rojo: %s\n",
               nivel_pot, *temp_actual,
               (*temp_actual >= (float)nivel_pot) ? "ON" : "OFF");
        return 0;
    }

    /* Comando no reconocido. */
    printf("Comando '%s' no reconocido. Escriba ? para ayuda.\n", linea);
    return 0;
}



/*                              Programa principal                            */


void app_main(void)
{
    /* ── Hardware compartido ─────────────────────────────────── */

    led_rgb_t led_rgb = {
        .led_red   = { .duty = 0, .gpio_num = LED_RGB_RED_GPIO,   .channel = LEDC_CHANNEL_0 },
        .led_green = { .duty = 0, .gpio_num = LED_RGB_GREEN_GPIO, .channel = LEDC_CHANNEL_1 },
        .led_blue  = { .duty = 0, .gpio_num = LED_RGB_BLUE_GPIO,  .channel = LEDC_CHANNEL_2 },
        .timer           = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .frequency       = 4000,
        .speed_mode      = LEDC_LOW_SPEED_MODE
    };

    adc_oneshot_unit_handle_t adc_handle  = NULL;
    adc_cali_handle_t         cali_handle = NULL;

    /* ── Estado Punto 1: impresion periodica de temperatura ────────── */
    int  intervalo_s = INTERVALO_DEFECTO_S;
    char cmd_buf[CMD_BUF_SIZE];
    int  cmd_len = 0;

    /* ── Estado Punto 2: LED por rangos de temperatura ───────────── */
    /* Rangos de temperatura en grados Celsius que activan cada color (P2).
     * La NTC dispara automaticamente el color cuyo rango incluye la temperatura actual.
     * Si la temperatura cae en dos rangos solapados, ambos colores encienden a la vez. */
    color_config_t colors[COLOR_COUNT] = {
        [COLOR_RED_IDX]   = { .intensity = 100, .range_low = 30, .range_high = 60 }, /* Rojo: 30-60 C (caliente).   */
        [COLOR_GREEN_IDX] = { .intensity = 100, .range_low = 20, .range_high = 30 }, /* Verde: 20-30 C (normal).    */
        [COLOR_BLUE_IDX]  = { .intensity = 100, .range_low =  0, .range_high = 20 }  /* Azul: 0-20 C (frio).        */
    };

    /* ── Estado Punto 3: histeresis para reporte serial ──────────── */
    color_estado_t estado_temp = COLOR_AZUL;
    float          temperatura = 0.0f;
    int            voltage_mv  = 0;

    /* ── Estado Punto 4: potenciometro ───────────────────────────── */
    int nivel_pot = 0;

    /* ── Contador de ticks del bucle ─────────────────────────── */
    uint32_t contador = 0;

    /* ── Inicializacion unica de perifericos compartidos ─────── */
    config_adc(&adc_handle, &cali_handle); /* ADC_UNIT_1: canales 4 (NTC) y 1 (pot). */
    config_led_rgb(&led_rgb);              /* Timer LEDC 0 + canales 0, 1, 2.        */
    config_uart();                         /* UART0 a 115200 bps.                     */
    config_boton();                        /* GPIO18 con pull-up + ISR flanco bajada. */

    /* Timer de software periodico para imprimir temperatura (P1). */
    esp_timer_handle_t timer_print = NULL;
    esp_timer_create_args_t timer_args = {
        .callback = timer_imprimir_cb,
        .name     = "print_temp"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_print));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_print,
        (uint64_t)intervalo_s * 1000000ULL));

    /* Estado inicial del LED. */
    aplicar_leds_manual(&led_rgb, colors, (int)temperatura);

    /* Bienvenida. */
    printf("=== PARCIAL STR - ESP32-C6 ===\n");
    printf("P1: NTC (GPIO4/ADC_CH4) -> temperatura UART cada %d s\n", intervalo_s);
    printf("P2: LED RGB por rangos de temperatura configurables (NTC dispara)\n");
    printf("P3: Boton GPIO%d alterna C/F | histeresis actualiza estado en serial\n", BUTTON_GPIO);
    printf("P4: Pot (GPIO1/ADC_CH1) define umbral en C -> LED rojo si NTC >= pot%%\n");
    imprimir_ayuda();

    /* ─────────────────────────────────────────────────────────── */
    /*                        BUCLE PRINCIPAL                      */
    /* ─────────────────────────────────────────────────────────── */
    while (1) {

        /* ── 1) Pulsador (P3): alterna C/F ──────────────────── */
        if (g_boton_evento) {
            g_boton_evento = false;
            g_unidad = (g_unidad == UNIDAD_CELSIUS)
                       ? UNIDAD_FAHRENHEIT
                       : UNIDAD_CELSIUS;
            printf(">> Unidad cambiada a %s\n",
                   (g_unidad == UNIDAD_CELSIUS) ? "Celsius" : "Fahrenheit");
        }

/* ========================================================================== */ 
    
        /* ── 2) Polling ADC NTC cada ~500 ms (P1/P3) ───────────── */

/* ========================================================================== */ 
        if ((contador % ADC_PERIOD_CYCLES) == 0) {
            voltage_mv  = leer_voltaje_ntc_mv(adc_handle, cali_handle);
            temperatura = voltaje_mv_a_celsius(voltage_mv);
            estado_temp = actualizar_estado_temp(estado_temp, temperatura);
        }
 

/* ========================================================================== */        

        /* ── 3) Polling ADC Potenciometro cada ~100 ms (P4) ──── */

/* ========================================================================== */

        if ((contador % POT_PERIOD_CYCLES) == 0) {
            nivel_pot = leer_pot_porcentaje(adc_handle);
        }




        /* ── 4) Aplicar LED con jerarquia de prioridad ────────── */
        /*
         *   Alta: P4 — NTC >= pot% -> LED rojo (override total).
         *            El potenciometro define el umbral de temperatura en %.
         *   Baja: P2 — rangos de temperatura configurables (NTC dispara).
         *   P3 sigue corriendo en background para actualizar estado_temp
         *   (usado en la impresion serial), no para controlar el LED.
         */
        if (temperatura >= (float)nivel_pot) {
            set_led_rgb_percentage_given_values(&led_rgb, 100, 0, 0);
        } else {
            aplicar_leds_manual(&led_rgb, colors, (int)temperatura);
        }

        /* ── 5) Imprimir temperatura cuando el timer lo indica (P1) */
        if (g_imprimir_ya) {
            g_imprimir_ya = false;
            float temp_mostrada = (g_unidad == UNIDAD_FAHRENHEIT)
                                  ? (temperatura * 9.0f / 5.0f + 32.0f)
                                  : temperatura;
            const char *sym = (g_unidad == UNIDAD_CELSIUS) ? "C" : "F";
            printf("[P1] Temp: %6.2f %s | V: %4d mV | Color: %s | Pot: %3d | LED rojo P4: %s\n",
                   temp_mostrada, sym, voltage_mv,
                   estado_a_texto(estado_temp),
                   nivel_pot,
                   (temperatura >= (float)nivel_pot) ? "ON" : "OFF");
        }

/* ========================================================================== */

        /* ── 6) Polling UART: leer y procesar comandos (P1/P2/P4) ── */

/* ========================================================================== */        

        uint8_t ch = 0;
        int recibido = uart_read_bytes(UART_PORT, &ch, 1, pdMS_TO_TICKS(POLL_DELAY_MS));
        if (recibido > 0) {
            if (ch == '\n' || ch == '\r') {              /* Enter: linea completa.              */
                if (cmd_len > 0) {
                    cmd_buf[cmd_len] = '\0';
                    int nuevo = procesar_comando(
                        cmd_buf, colors, &temperatura,
                        nivel_pot, &intervalo_s);

                    if (nuevo > 0) {                     /* Usuario cambio el intervalo (P1).   */
                        intervalo_s = nuevo;
                        ESP_ERROR_CHECK(esp_timer_stop(timer_print));
                        ESP_ERROR_CHECK(esp_timer_start_periodic(
                            timer_print,
                            (uint64_t)intervalo_s * 1000000ULL));
                    }
                    cmd_len = 0;
                }
            } else if (cmd_len < CMD_BUF_SIZE - 1) {    /* Acumular caracter en el buffer.     */
                cmd_buf[cmd_len++] = (char)ch;
            }
        }

        contador++;

    } /* while(1) */
}
