#include <stdio.h>                       /* printf: reportar estado por el monitor serial en cada ciclo. */
#include <math.h>                        /* logf: requerida por la ecuación Beta de Steinhart-Hart para la NTC. */
#include <stdint.h>                      /* uint32_t y tipos de tamaño fijo usados por FreeRTOS y LEDC. */

#include "freertos/FreeRTOS.h"           /* Base de FreeRTOS: debe incluirse antes de cualquier otra cabecera del RTOS. */
#include "freertos/task.h"               /* vTaskDelay: cede el CPU entre lecturas para no bloquear el scheduler. */

#include "driver/ledc.h"                 /* Constantes LEDC_CHANNEL_x y LEDC_TIMER_x del periférico PWM. */
#include "driver/gpio.h"                 /* gpio_set_direction, gpio_get_level: control digital del pulsador. */

#include "esp_adc/adc_oneshot.h"         /* API oneshot: lectura ADC bajo demanda, sin DMA ni conversión continua. */
#include "esp_adc/adc_cali.h"            /* adc_cali_raw_to_voltage: convierte lectura cruda a mV usando la curva real del chip. */
#include "esp_adc/adc_cali_scheme.h"     /* adc_cali_create_scheme_curve_fitting: esquema de calibración disponible en ESP32-C6. */
#include "esp_err.h"                     /* ESP_ERROR_CHECK: detiene el sistema si una llamada de ESP-IDF falla. */
#include "esp_attr.h"                    /* IRAM_ATTR: coloca la ISR del botón en RAM interna para ejecución rápida. */

#include "library_led_c.h"               /* led_rgb_t y funciones de control PWM del LED RGB de tres canales. */

/* ========================================================================== */
/*                              Pines del LED RGB                             */
/* ========================================================================== */

#define LED_RGB_RED_GPIO     (13)         /* GPIO del ánodo rojo del LED RGB. */
#define LED_RGB_GREEN_GPIO   (19)         /* GPIO del ánodo verde del LED RGB. */
#define LED_RGB_BLUE_GPIO    (5)          /* GPIO del ánodo azul del LED RGB. */

/* ========================================================================== */
/*                            Pin del pulsador                                */
/* ========================================================================== */

#define BUTTON_GPIO          (18)         /* GPIO del pulsador: permite cambiar unidades de temperatura. GPIO18 libre, sin strapping ni funciones reservadas. */
#define BUTTON_DEBOUNCE_MS   (200)        /* Tiempo de debounce en ms: ignora rebotes del contacto. 200 ms evita dobles disparos sin perder pulsaciones reales. */

/* ========================================================================== */
/*                         Entrada analógica de la NTC                        */
/* ========================================================================== */

#define NTC_ADC_UNIT         ADC_UNIT_1   /* Unidad ADC1 del ESP32-C6 que agrupa los canales disponibles. */
#define NTC_ADC_CHANNEL      ADC_CHANNEL_4 /* Canal 4 de ADC1, conectado físicamente al GPIO4 del ESP32-C6. */
#define NTC_ADC_ATTEN        ADC_ATTEN_DB_12 /* Atenuación 12 dB: permite medir hasta ~3.1 V, cubre todo el rango del divisor. */
#define NTC_ADC_WIDTH        ADC_BITWIDTH_12 /* Resolución de 12 bits: valores entre 0 y 4095. */

/*
 * Por qué oversampling: el ADC del ESP32-C6 tiene ruido de ±10-30 LSB, lo que
 * produce saltos de ±1-2 °C entre lecturas consecutivas sin cambio real en la NTC.
 * Promediar N muestras reduce el ruido en un factor de sqrt(N): con 16 muestras
 * el ruido efectivo cae a ±3-8 LSB, equivalente a menos de ±0.5 °C.
 */
#define N_MUESTRAS_ADC       (16)         /* Número de muestras que se promedian en cada ciclo de lectura. */

/* ========================================================================== */
/*                     Parámetros de la NTC (divisor de voltaje)              */
/* ========================================================================== */

/*
 * Circuito: 3.3V --- R_SERIE (10 kΩ) --- [GPIO4/ADC] --- NTC (5 kΩ) --- GND
 *
 * La NTC disminuye su resistencia al subir la temperatura.
 * A mayor temperatura → mayor tensión en el nodo ADC → mayor lectura cruda.
 *
 * Por qué usar voltaje en lugar de raw para calcular R_ntc:
 * La conversión raw → voltaje no es lineal por la curva de ganancia interna del ADC.
 * Usar adc_cali_raw_to_voltage aplica la curva de calibración del chip y da un
 * voltaje real, por lo que R_ntc y la temperatura son más precisas (~0.5-1 °C mejor).
 */

#define NTC_BETA             (3950.0f)    /* Coeficiente Beta de la NTC 5K: caracteriza su curva resistencia-temperatura. */
#define NTC_R0               (5000.0f)   /* Resistencia nominal (5kΩ) de la NTC a 25 °C en ohms. */
#define NTC_T0_KELVIN        (298.15f)   /* Temperatura nominal en Kelvin (25 °C + 273.15). */
#define NTC_R_SERIE          (10000.0f)  /* Resistencia en serie del divisor de voltaje en ohms. */
#define NTC_VCC_MV           (3300)      /* Tensión de alimentación del divisor en milivoltios. */

/* ========================================================================== */
/*                    Umbrales de temperatura con histéresis                  */
/* ========================================================================== */

/*
 * Por qué histéresis: sin ella, si la temperatura oscila justo en 25 °C o 35 °C
 * el LED cambia de color repetidamente (flickering). La histéresis define una
 * zona muerta de ±0.5 °C alrededor de cada umbral: el color solo cambia cuando
 * se supera el margen, evitando cambios inestables por ruido térmico o eléctrico.
 *
 * Lógica de transición:
 *   AZUL  → VERDE : T >= 25.5 °C
 *   VERDE → AZUL  : T <  24.5 °C
 *   VERDE → ROJO  : T >  35.5 °C
 *   ROJO  → VERDE : T <= 34.5 °C
 */

#define TEMP_AZUL_A_VERDE    (25.5f)     /* Umbral de subida: la NTC debe calentar más de 25.5 °C para pasar a verde. */
#define TEMP_VERDE_A_AZUL    (24.5f)     /* Umbral de bajada: la NTC debe enfriarse bajo 24.5 °C para volver a azul. */
#define TEMP_VERDE_A_ROJO    (35.5f)     /* Umbral de subida: la NTC debe calentar más de 35.5 °C para pasar a rojo. */
#define TEMP_ROJO_A_VERDE    (34.5f)     /* Umbral de bajada: la NTC debe enfriarse a 34.5 °C o menos para volver a verde. */

/* ========================================================================== */
/*                         Tiempos de ejecución                               */
/* ========================================================================== */

#define POLL_DELAY_MS        (20)         /* Tick rápido del bucle: muestrea el botón con buena respuesta y sin perder pulsaciones. */
#define ADC_PERIOD_CYCLES    (25)         /* Lee ADC/temperatura cada 25 ticks (~500 ms): sigue cambios térmicos lentos. */
#define PRINT_PERIOD_CYCLES  (100)        /* Imprime cada 100 ticks (~2 s) para no saturar el puerto serial. */

/* ========================================================================== */
/*                         Estado de color del LED                            */
/* ========================================================================== */

typedef enum {                            /* Tipo enumerado que representa el color activo del LED RGB. */
    COLOR_AZUL  = 0,                      /* Estado frío: temperatura por debajo del umbral inferior. */
    COLOR_VERDE,                          /* Estado templado: temperatura dentro del rango normal. */
    COLOR_ROJO                            /* Estado caliente: temperatura por encima del umbral superior. */
} color_estado_t;                         /* Nombre del tipo para usarlo en el resto del programa. */

/* ========================================================================== */
/*                     Unidades de temperatura disponibles                    */
/* ========================================================================== */

typedef enum {                            /* Tipo enumerado que representa la unidad de temperatura seleccionada. */
    UNIDAD_CELSIUS = 0,                   /* Mostrar temperatura en grados Celsius (°C). */
    UNIDAD_FAHRENHEIT                     /* Mostrar temperatura en grados Fahrenheit (°F). */
} unidad_temperatura_t;                   /* Nombre del tipo para el estado de unidades. */

/* ========================================================================== */
/*                    Configuración del ADC y su calibración                  */
/* ========================================================================== */

static void config_ntc_adc(                                          /* Configura la unidad ADC y crea el esquema de calibración. */
    adc_oneshot_unit_handle_t *adc_handle,                           /* Puntero donde se guardará el ADC creado. */
    adc_cali_handle_t         *cali_handle)                          /* Puntero donde se guardará el manejador de calibración. */
{                                                                    /* Inicia el bloque de código de config_ntc_adc. */
    adc_oneshot_unit_init_cfg_t init_cfg = {                         /* Estructura de inicialización de la unidad ADC. */
        .unit_id = NTC_ADC_UNIT                                      /* Selecciona la unidad ADC1 donde está el canal de la NTC. */
    };                                                               /* Termina la configuración de la unidad. */

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, adc_handle));    /* Crea la unidad ADC y guarda su manejador. */

    adc_oneshot_chan_cfg_t chan_cfg = {                               /* Estructura de configuración del canal ADC. */
        .atten    = NTC_ADC_ATTEN,                                   /* Atenuación para medir el rango completo del divisor de voltaje. */
        .bitwidth = NTC_ADC_WIDTH                                    /* Resolución de 12 bits para tener suficiente precisión. */
    };                                                               /* Termina la configuración del canal. */

    ESP_ERROR_CHECK(adc_oneshot_config_channel(                      /* Aplica la configuración al canal físico de la NTC. */
        *adc_handle,                                                 /* Manejador de la unidad ADC recién creada. */
        NTC_ADC_CHANNEL,                                             /* Canal ADC4 conectado a GPIO4. */
        &chan_cfg                                                     /* Configuración de atenuación y resolución. */
    ));                                                              /* Cierra la configuración del canal. */

    /*
     * Por qué calibrar: el ADC del ESP32-C6 tiene no linealidades de hasta ±100 mV
     * sin calibración. adc_cali_create_scheme_curve_fitting usa constantes grabadas
     * en eFuse durante la fabricación del chip para corregir esa curva y entregar
     * un voltaje real con error menor a ±20 mV en todo el rango.
     */
    adc_cali_curve_fitting_config_t cali_cfg = {                     /* Estructura de configuración del esquema de calibración. */
        .unit_id  = NTC_ADC_UNIT,                                    /* Unidad ADC a la que pertenece el canal calibrado. */
        .chan     = NTC_ADC_CHANNEL,                                 /* Canal específico para el que se crea la calibración. */
        .atten    = NTC_ADC_ATTEN,                                   /* Debe coincidir con la atenuación configurada en el canal. */
        .bitwidth = NTC_ADC_WIDTH                                    /* Debe coincidir con la resolución configurada en el canal. */
    };                                                               /* Termina la configuración de calibración. */

    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(            /* Crea el esquema de calibración curve-fitting para ESP32-C6. */
        &cali_cfg,                                                   /* Configuración del esquema de calibración. */
        cali_handle                                                  /* Puntero donde se guardará el manejador de calibración. */
    ));                                                              /* Cierra la creación del esquema de calibración. */
}                                                                    /* Termina la función config_ntc_adc. */

/* ========================================================================== */
/*               Lectura promediada del ADC con calibración                   */
/* ========================================================================== */

static int leer_voltaje_mv(                                          /* Promedia N lecturas crudas y las convierte a milivoltios calibrados. */
    adc_oneshot_unit_handle_t adc_handle,                            /* Manejador de la unidad ADC configurada. */
    adc_cali_handle_t         cali_handle)                           /* Manejador del esquema de calibración. */
{                                                                    /* Inicia el bloque de código de leer_voltaje_mv. */
    int suma       = 0;                                              /* Acumulador de las N lecturas crudas para calcular el promedio. */
    int muestra    = 0;                                              /* Almacena cada lectura cruda individual del ADC. */
    int voltage_mv = 0;                                              /* Almacena el voltaje en mV tras aplicar la calibración. */

    for (int i = 0; i < N_MUESTRAS_ADC; i++) {                      /* Repite la lectura N veces para promediar y reducir el ruido. */
        ESP_ERROR_CHECK(adc_oneshot_read(                            /* Toma una muestra cruda del canal ADC de la NTC. */
            adc_handle,                                              /* Manejador de la unidad ADC. */
            NTC_ADC_CHANNEL,                                         /* Canal conectado a la NTC. */
            &muestra                                                 /* Destino de la lectura cruda (0 a 4095). */
        ));                                                          /* Cierra la lectura y valida errores. */
        suma += muestra;                                             /* Acumula la muestra para calcular el promedio al final. */
    }                                                                /* Termina el ciclo de oversampling. */

    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(                         /* Convierte el promedio crudo a voltaje real usando la curva del chip. */
        cali_handle,                                                 /* Manejador del esquema de calibración. */
        suma / N_MUESTRAS_ADC,                                       /* Promedio de las N lecturas crudas. */
        &voltage_mv                                                  /* Destino del voltaje calibrado en milivoltios. */
    ));                                                              /* Cierra la conversión y valida errores. */

    return voltage_mv;                                               /* Devuelve el voltaje calibrado en mV listo para calcular temperatura. */
}                                                                    /* Termina la función leer_voltaje_mv. */

/* ========================================================================== */
/*              Conversión de voltaje calibrado a temperatura                 */
/* ========================================================================== */

static float voltaje_mv_a_celsius(int voltage_mv)                   /* Convierte el voltaje del divisor a temperatura en grados Celsius. */
{                                                                    /* Inicia el bloque de código de voltaje_mv_a_celsius. */
    if (voltage_mv <= 0) {                                           /* Evita división por cero si el ADC lee tensión nula (NTC abierta). */
        voltage_mv = 1;                                              /* Sustituye por 1 mV para que el cálculo sea matemáticamente válido. */
    }                                                                /* Termina la protección del límite inferior. */

    if (voltage_mv >= NTC_VCC_MV) {                                  /* Evita división por cero si el voltaje iguala o supera VCC (NTC en cortocircuito). */
        voltage_mv = NTC_VCC_MV - 1;                                 /* Sustituye por VCC - 1 mV para mantener el cálculo válido. */
    }                                                                /* Termina la protección del límite superior. */

    /*
     * Resistencia de la NTC a partir del divisor de voltaje:
     *   R_ntc = R_serie * V_adc / (VCC - V_adc)
     * Usar voltaje calibrado en lugar de raw elimina la no linealidad del ADC.
     */
    float r_ntc = NTC_R_SERIE * (float)voltage_mv /                 /* Calcula la resistencia real de la NTC usando el voltaje calibrado. */
                  (float)(NTC_VCC_MV - voltage_mv);

    /*
     * Ecuación Beta de Steinhart-Hart simplificada:
     *   1/T [K] = 1/T0 + (1/Beta) * ln(R / R0)
     */
    float inv_T = (1.0f / NTC_T0_KELVIN) +                          /* Término de referencia a 25 °C en escala Kelvin. */
                  (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0);         /* Término de corrección por la desviación de resistencia respecto a R0. */

    return (1.0f / inv_T) - 273.15f;                                /* Convierte el resultado de Kelvin a grados Celsius y lo devuelve. */
}                                                                    /* Termina la función voltaje_mv_a_celsius. */

/* ========================================================================== */
/*              Máquina de estados con histéresis de temperatura              */
/* ========================================================================== */

static color_estado_t actualizar_estado(                             /* Evalúa la temperatura y retorna el nuevo estado de color con histéresis. */
    color_estado_t estado,                                           /* Estado de color actual antes de revisar la temperatura. */
    float          temperatura)                                      /* Temperatura medida en grados Celsius en este ciclo. */
{                                                                    /* Inicia el bloque de código de actualizar_estado. */
    switch (estado) {                                                /* Evalúa el estado actual para aplicar los umbrales correctos. */

    case COLOR_AZUL:                                                 /* Cuando el LED está en azul solo puede subir a verde. */
        if (temperatura >= TEMP_AZUL_A_VERDE) {                     /* Revisa si la temperatura superó el umbral de subida (25.5 °C). */
            return COLOR_VERDE;                                      /* Transiciona a verde porque ya superó la zona de histéresis. */
        }                                                            /* Termina la condición de subida desde azul. */
        break;                                                       /* Sale del switch si no hay transición. */

    case COLOR_VERDE:                                                /* Cuando el LED está en verde puede bajar a azul o subir a rojo. */
        if (temperatura < TEMP_VERDE_A_AZUL) {                      /* Revisa si la temperatura bajó del umbral de bajada (24.5 °C). */
            return COLOR_AZUL;                                       /* Transiciona a azul porque bajó de la zona de histéresis inferior. */
        }                                                            /* Termina la condición de bajada desde verde. */
        if (temperatura > TEMP_VERDE_A_ROJO) {                      /* Revisa si la temperatura superó el umbral de subida (35.5 °C). */
            return COLOR_ROJO;                                       /* Transiciona a rojo porque superó la zona de histéresis superior. */
        }                                                            /* Termina la condición de subida desde verde. */
        break;                                                       /* Sale del switch si la temperatura permanece en la zona verde. */

    case COLOR_ROJO:                                                 /* Cuando el LED está en rojo solo puede bajar a verde. */
        if (temperatura <= TEMP_ROJO_A_VERDE) {                     /* Revisa si la temperatura bajó al umbral de bajada (34.5 °C). */
            return COLOR_VERDE;                                      /* Transiciona a verde porque entró de nuevo a la zona normal. */
        }                                                            /* Termina la condición de bajada desde rojo. */
        break;                                                       /* Sale del switch si no hay transición. */
    }                                                                /* Termina la evaluación de la máquina de estados. */

    return estado;                                                   /* Devuelve el mismo estado si ninguna condición de transición se cumplió. */
}                                                                    /* Termina la función actualizar_estado. */

/* ========================================================================== */
/*                    Aplicación del color al LED RGB                         */
/* ========================================================================== */

static void aplicar_color(led_rgb_t *led_rgb, color_estado_t estado) /* Enciende el canal del LED que corresponde al estado de temperatura. */
{                                                                    /* Inicia el bloque de código de aplicar_color. */
    switch (estado) {                                                /* Evalúa el estado para decidir qué canal encender. */
    case COLOR_AZUL:                                                 /* Temperatura fría: enciende solo el canal azul. */
        set_led_rgb_percentage_given_values(led_rgb, 0, 0, 100);    /* Rojo 0%, Verde 0%, Azul 100%. */
        break;                                                       /* Sale del switch después de encender azul. */
    case COLOR_VERDE:                                                /* Temperatura normal: enciende solo el canal verde. */
        set_led_rgb_percentage_given_values(led_rgb, 0, 100, 0);    /* Rojo 0%, Verde 100%, Azul 0%. */
        break;                                                       /* Sale del switch después de encender verde. */
    case COLOR_ROJO:                                                 /* Temperatura alta: enciende solo el canal rojo. */
        set_led_rgb_percentage_given_values(led_rgb, 100, 0, 0);    /* Rojo 100%, Verde 0%, Azul 0%. */
        break;                                                       /* Sale del switch después de encender rojo. */
    }                                                                /* Termina la selección de canal. */
}                                                                    /* Termina la función aplicar_color. */

/* ========================================================================== */
/*                       Texto descriptivo del estado                         */
/* ========================================================================== */

static const char *estado_a_texto(color_estado_t estado)            /* Devuelve una cadena con el nombre del estado para el monitor serial. */
{                                                                    /* Inicia el bloque de código de estado_a_texto. */
    switch (estado) {                                                /* Evalúa el estado para elegir el texto correspondiente. */
    case COLOR_AZUL:  return "Azul  (T < 25 C)";                    /* Texto del estado frío con su rango de temperatura. */
    case COLOR_VERDE: return "Verde (25 C <= T <= 35 C)";           /* Texto del estado normal con su rango de temperatura. */
    case COLOR_ROJO:  return "Rojo  (T > 35 C)";                    /* Texto del estado caliente con su rango de temperatura. */
    default:          return "Desconocido";                          /* Texto de seguridad para valores de estado inesperados. */
    }                                                                /* Termina la selección del texto. */
}                                                                    /* Termina la función estado_a_texto. */

/* ========================================================================== */
/*                   Conversión de Celsius a Fahrenheit                       */
/* ========================================================================== */

static float celsius_a_fahrenheit(float celsius)                    /* Convierte una temperatura en Celsius a Fahrenheit. */
{                                                                    /* Inicia el bloque de código de celsius_a_fahrenheit. */
    return (celsius * 9.0f / 5.0f) + 32.0f;                         /* Aplica la fórmula F = C * 9/5 + 32 y devuelve el resultado. */
}                                                                    /* Termina la función celsius_a_fahrenheit. */

/* ========================================================================== */
/*                     Obtener símbolo de la unidad                           */
/* ========================================================================== */

static const char *obtener_simbolo_unidad(unidad_temperatura_t unidad) /* Devuelve el símbolo de la unidad seleccionada. */
{                                                                    /* Inicia el bloque de código de obtener_simbolo_unidad. */
    switch (unidad) {                                                /* Evalúa la unidad para elegir el símbolo. */
    case UNIDAD_CELSIUS:    return "C";                             /* Símbolo para grados Celsius. */
    case UNIDAD_FAHRENHEIT: return "F";                             /* Símbolo para grados Fahrenheit. */
    default:                return "?";                             /* Símbolo de seguridad para unidad desconocida. */
    }                                                                /* Termina la selección del símbolo. */
}                                                                    /* Termina la función obtener_simbolo_unidad. */

/* ========================================================================== */
/*                    Configuración del GPIO del pulsador                     */
/* ========================================================================== */

/*
 * Bandera escrita por la ISR del botón y leída por el bucle principal.
 * volatile: obliga al compilador a releerla de memoria en cada acceso,
 * porque cambia de forma asíncrona dentro de la interrupción.
 */
static volatile bool     boton_evento    = false;                   /* Pasa a true cuando la ISR confirma una pulsación válida. */
static volatile uint32_t ultimo_isr_ms   = 0;                       /* Marca de tiempo de la última pulsación aceptada (para el debounce). */

/*
 * Rutina de interrupción del botón. Se ejecuta en cada flanco de bajada
 * (liberado→presionado). El debounce se hace por tiempo: ignora flancos que
 * llegan antes de BUTTON_DEBOUNCE_MS, que son rebotes mecánicos del contacto.
 * IRAM_ATTR mantiene la función en RAM interna para que la ISR sea rápida.
 */
static void IRAM_ATTR boton_isr_handler(void *arg)                  /* Manejador de interrupción del pulsador. */
{                                                                    /* Inicia el bloque de código de boton_isr_handler. */
    uint32_t ahora = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS; /* Tiempo actual en ms; versión segura para ISR. */

    if ((ahora - ultimo_isr_ms) >= BUTTON_DEBOUNCE_MS) {            /* Acepta el flanco solo si pasó el tiempo de debounce. */
        ultimo_isr_ms = ahora;                                       /* Actualiza la marca para rechazar los rebotes siguientes. */
        boton_evento  = true;                                        /* Señala al bucle principal que hubo una pulsación. */
    }                                                                /* Termina el filtrado por debounce. */
}                                                                    /* Termina la función boton_isr_handler. */

/* ========================================================================== */
/*                    Configuración del GPIO del pulsador                     */
/* ========================================================================== */

static void config_boton(void)                                       /* Configura el pin del pulsador como entrada con interrupción por flanco. */
{                                                                    /* Inicia el bloque de código de config_boton. */
    gpio_config_t boton_cfg = {                                      /* Estructura de configuración del pin del pulsador. */
        .pin_bit_mask = (1ULL << BUTTON_GPIO),                       /* Selecciona el pin específico del pulsador en la máscara de bits. */
        .mode         = GPIO_MODE_INPUT,                            /* Configura el pin como entrada digital. */
        .pull_up_en   = GPIO_PULLUP_ENABLE,                         /* Habilita la resistencia pull-up interna para estado lógico alto en reposo. */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,                      /* Deshabilita la resistencia pull-down. */
        .intr_type    = GPIO_INTR_NEGEDGE                           /* Interrumpe en el flanco de bajada (1→0): el momento exacto de presionar. */
    };                                                               /* Termina la estructura de configuración. */

    ESP_ERROR_CHECK(gpio_config(&boton_cfg));                        /* Aplica la configuración al GPIO del pulsador. */

    ESP_ERROR_CHECK(gpio_install_isr_service(0));                    /* Instala el servicio de interrupciones GPIO (una sola vez en el sistema). */
    ESP_ERROR_CHECK(gpio_isr_handler_add(                            /* Asocia la rutina de interrupción al pin del pulsador. */
        BUTTON_GPIO,                                                 /* Pin que disparará la interrupción. */
        boton_isr_handler,                                           /* Función que se ejecuta al detectar el flanco. */
        NULL                                                         /* Argumento que se pasa a la ISR (no se usa aquí). */
    ));                                                              /* Cierra el registro del manejador. */
}                                                                    /* Termina la función config_boton. */

/* ========================================================================== */
/*                              Programa principal                            */
/* ========================================================================== */

void app_main(void)                                                  /* Función principal que ESP-IDF ejecuta al iniciar el sistema. */
{                                                                    /* Inicia el bloque de código de app_main. */
    led_rgb_t led_rgb = {                                            /* Estructura que describe la configuración completa del LED RGB. */
        .led_red = {                                                 /* Configuración del canal rojo. */
            .duty     = 0,                                           /* Rojo apagado al inicio. */
            .gpio_num = LED_RGB_RED_GPIO,                            /* GPIO físico del ánodo rojo. */
            .channel  = LEDC_CHANNEL_0                               /* Canal PWM 0 asignado al rojo. */
        },                                                           /* Termina la configuración del canal rojo. */
        .led_green = {                                               /* Configuración del canal verde. */
            .duty     = 0,                                           /* Verde apagado al inicio. */
            .gpio_num = LED_RGB_GREEN_GPIO,                          /* GPIO físico del ánodo verde. */
            .channel  = LEDC_CHANNEL_1                               /* Canal PWM 1 asignado al verde. */
        },                                                           /* Termina la configuración del canal verde. */
        .led_blue = {                                                /* Configuración del canal azul. */
            .duty     = 0,                                           /* Azul apagado al inicio. */
            .gpio_num = LED_RGB_BLUE_GPIO,                           /* GPIO físico del ánodo azul. */
            .channel  = LEDC_CHANNEL_2                               /* Canal PWM 2 asignado al azul. */
        },                                                           /* Termina la configuración del canal azul. */
        .timer           = LEDC_TIMER_0,                             /* Timer LEDC 0 compartido por los tres canales PWM. */
        .duty_resolution = LEDC_TIMER_13_BIT,                        /* 13 bits de resolución: permite 8192 niveles de brillo. */
        .frequency       = 4000,                                     /* 4000 Hz: frecuencia PWM por encima del umbral de parpadeo visible. */
        .speed_mode      = LEDC_LOW_SPEED_MODE                       /* Modo de baja velocidad: único disponible en ESP32-C6. */
    };                                                               /* Termina la estructura del LED RGB. */

    adc_oneshot_unit_handle_t adc_handle  = NULL;                    /* Manejador de la unidad ADC, inicializado en NULL antes de configurar. */
    adc_cali_handle_t         cali_handle = NULL;                    /* Manejador de calibración, inicializado en NULL antes de configurar. */
    int              voltage_mv  = 0;                                /* Voltaje calibrado en mV leído en cada ciclo. */
    float            temperatura = 0.0f;                             /* Temperatura calculada en grados Celsius en cada ciclo. */
    float            temp_mostrada = 0.0f;                           /* Temperatura ya convertida a la unidad seleccionada para imprimir. */
    color_estado_t   estado      = COLOR_AZUL;                       /* Estado inicial: azul, asumiendo temperatura fría al arrancar. */
    unidad_temperatura_t unidad  = UNIDAD_CELSIUS;                   /* Unidad de impresión inicial; el pulsador alterna entre °C y °F. */
    uint32_t         contador    = 0;                                /* Contador de ticks del bucle para programar lecturas e impresión. */

    config_ntc_adc(&adc_handle, &cali_handle);                       /* Inicializa el ADC y crea el esquema de calibración. */
    config_led_rgb(&led_rgb);                                        /* Configura el timer LEDC y los tres canales PWM del LED RGB. */
    config_boton();                                                  /* Configura el GPIO del pulsador como entrada con pull-up interno. */
    aplicar_color(&led_rgb, estado);                                 /* Aplica el estado inicial al LED antes de entrar al bucle. */

    printf("=== Control RGB por temperatura NTC ===\n");             /* Encabezado del sistema en el monitor serial. */
    printf("T < 25 C         -> LED Azul\n");                        /* Indica la condición del rango frío. */
    printf("25 C <= T <= 35 C -> LED Verde\n");                      /* Indica la condición del rango normal. */
    printf("T > 35 C         -> LED Rojo\n");                        /* Indica la condición del rango caliente. */
    printf("Histeresis: ±0.5 C en cada umbral\n");                   /* Informa que los umbrales tienen zona muerta de ±0.5 °C. */
    printf("Oversampling: %d muestras por ciclo\n", N_MUESTRAS_ADC); /* Informa el número de muestras promediadas por lectura. */
    printf("Pulsador GPIO%d: alterna unidad C <-> F\n", BUTTON_GPIO); /* Informa el pin y la función del pulsador. */
    printf("NTC: ADC_CH%d (GPIO4) | LED R:%d G:%d B:%d | BTN:%d\n\n", /* Informa los pines físicos usados por el sistema. */
           NTC_ADC_CHANNEL, LED_RGB_RED_GPIO,
           LED_RGB_GREEN_GPIO, LED_RGB_BLUE_GPIO, BUTTON_GPIO);

    while (1) {                                                      /* Bucle infinito principal del sistema embebido. */

        if (boton_evento) {                                          /* La ISR marcó una pulsación: la atiende en contexto de tarea. */
            boton_evento = false;                                    /* Limpia la bandera para no procesar la misma pulsación dos veces. */
            unidad = (unidad == UNIDAD_CELSIUS)                      /* Alterna la unidad de impresión en cada pulsación válida. */
                     ? UNIDAD_FAHRENHEIT                             /* Si estaba en Celsius pasa a Fahrenheit. */
                     : UNIDAD_CELSIUS;                               /* Si estaba en Fahrenheit vuelve a Celsius. */
            printf(">> Unidad cambiada a %s\n",                      /* Confirma el cambio de inmediato en el monitor serial. */
                   obtener_simbolo_unidad(unidad));
        }                                                            /* Termina el manejo de la pulsación. */

        if ((contador % ADC_PERIOD_CYCLES) == 0) {                   /* Cada ~500 ms toma una nueva lectura térmica. */
            voltage_mv  = leer_voltaje_mv(adc_handle, cali_handle);  /* Toma 16 muestras, las promedia y aplica la calibración. */
            temperatura = voltaje_mv_a_celsius(voltage_mv);          /* Convierte el voltaje calibrado a temperatura en grados Celsius. */
            estado      = actualizar_estado(estado, temperatura);    /* Evalúa si la temperatura cruzó algún umbral con histéresis. */
            aplicar_color(&led_rgb, estado);                         /* Actualiza el PWM del LED RGB según el nuevo estado de color. */
        }                                                            /* Termina la lectura térmica periódica. */

        if ((contador % PRINT_PERIOD_CYCLES) == 0) {                 /* Cada ~2 s imprime el diagnóstico en la unidad seleccionada. */
            temp_mostrada = (unidad == UNIDAD_FAHRENHEIT)            /* Convierte la temperatura a la unidad activa antes de imprimir. */
                            ? celsius_a_fahrenheit(temperatura)      /* Fahrenheit si esa es la unidad seleccionada. */
                            : temperatura;                           /* Celsius en caso contrario. */
            printf("V: %4d mV | Temp: %6.2f %s | Color: %s\n",     /* Imprime el diagnóstico completo con la unidad seleccionada. */
                   voltage_mv, temp_mostrada,
                   obtener_simbolo_unidad(unidad), estado_a_texto(estado));
        }                                                            /* Termina la impresión periódica. */

        contador++;                                                  /* Avanza el contador de ticks del bucle. */
        vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));                    /* Pausa 20 ms: cede el CPU y marca el ritmo de muestreo del botón. */
    }                                                                /* Termina una vuelta del bucle y regresa al inicio. */
}                  