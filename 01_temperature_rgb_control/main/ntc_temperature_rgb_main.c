#include <stdio.h>                       /* printf: reportar estado por el monitor serial en cada ciclo. */
#include <math.h>                        /* logf: requerida por la ecuacion Beta de Steinhart-Hart para la NTC. */
#include <stdint.h>                      /* uint32_t y tipos de tamano fijo usados por FreeRTOS y LEDC. */

#include "freertos/FreeRTOS.h"           /* Base de FreeRTOS: debe incluirse antes de cualquier otra cabecera del RTOS. */
#include "freertos/task.h"               /* vTaskDelay: cede el CPU entre lecturas para no bloquear el scheduler. */

#include "driver/ledc.h"                 /* Constantes LEDC_CHANNEL_x y LEDC_TIMER_x del periferico PWM. */

#include "esp_adc/adc_oneshot.h"         /* API oneshot: lectura ADC bajo demanda, sin DMA ni conversion continua. */
#include "esp_adc/adc_cali.h"            /* adc_cali_raw_to_voltage: convierte lectura cruda a mV usando la curva real del chip. */
#include "esp_adc/adc_cali_scheme.h"     /* adc_cali_create_scheme_curve_fitting: esquema de calibracion disponible en ESP32-C6. */
#include "esp_err.h"                     /* ESP_ERROR_CHECK: detiene el sistema si una llamada de ESP-IDF falla. */

#include "library_led_c.h"               /* led_rgb_t y funciones de control PWM del LED RGB de tres canales. */

/* ========================================================================== */
/*                              Pines del LED RGB                             */
/* ========================================================================== */

#define LED_RGB_RED_GPIO     (13)         /* GPIO del anodo rojo del LED RGB. */
#define LED_RGB_GREEN_GPIO   (19)         /* GPIO del anodo verde del LED RGB. */
#define LED_RGB_BLUE_GPIO    (5)          /* GPIO del anodo azul del LED RGB. */

/* ========================================================================== */
/*                         Entrada analogica de la NTC                        */
/* ========================================================================== */

#define NTC_ADC_UNIT         ADC_UNIT_1   /* Unidad ADC1 del ESP32-C6 que agrupa los canales disponibles. */
#define NTC_ADC_CHANNEL      ADC_CHANNEL_4 /* Canal 4 de ADC1, conectado fisicamente al GPIO4 del ESP32-C6. */
#define NTC_ADC_ATTEN        ADC_ATTEN_DB_12 /* Atenuacion 12 dB: permite medir hasta ~3.1 V, cubre todo el rango del divisor. */
#define NTC_ADC_WIDTH        ADC_BITWIDTH_12 /* Resolucion de 12 bits: valores entre 0 y 4095. */

/*
 * Por que oversampling: el ADC del ESP32-C6 tiene ruido de ±10-30 LSB, lo que
 * produce saltos de ±1-2 °C entre lecturas consecutivas sin cambio real en la NTC.
 * Promediar N muestras reduce el ruido en un factor de sqrt(N): con 16 muestras
 * el ruido efectivo cae a ±3-8 LSB, equivalente a menos de ±0.5 °C.
 */
#define N_MUESTRAS_ADC       (16)         /* Numero de muestras que se promedian en cada ciclo de lectura. */

/* ========================================================================== */
/*                     Parametros de la NTC (divisor de voltaje)              */
/* ========================================================================== */

/*
 * Circuito: 3.3V --- R_SERIE (10 kΩ) --- [GPIO4/ADC] --- NTC (5 kΩ) --- GND
 *
 * La NTC disminuye su resistencia al subir la temperatura.
 * A mayor temperatura → mayor tension en el nodo ADC → mayor lectura cruda.
 *
 * Por que usar voltaje en lugar de raw para calcular R_ntc:
 * La conversion raw → voltaje no es lineal por la curva de ganancia interna del ADC.
 * Usar adc_cali_raw_to_voltage aplica la curva de calibracion del chip y da un
 * voltaje real, por lo que R_ntc y la temperatura son mas precisas (~0.5-1 °C mejor).
 */

#define NTC_BETA             (3900.0f)    /* Coeficiente Beta de la NTC 5K: caracteriza su curva resistencia-temperatura. */
#define NTC_R0               (5000.0f)   /* Resistencia nominal (5kΩ) de la NTC a 25 °C en ohms. */
#define NTC_T0_KELVIN        (298.15f)   /* Temperatura nominal en Kelvin (25 °C + 273.15). */
#define NTC_R_SERIE          (10000.0f)  /* Resistencia en serie del divisor de voltaje en ohms. */
#define NTC_VCC_MV           (3300)      /* Tension de alimentacion del divisor en milivoltios. */

/* ========================================================================== */
/*                    Umbrales de temperatura con histeresis                  */
/* ========================================================================== */

/*
 * Por que histeresis: sin ella, si la temperatura oscila justo en 25 °C o 35 °C
 * el LED cambia de color repetidamente (flickering). La histeresis define una
 * zona muerta de ±0.5 °C alrededor de cada umbral: el color solo cambia cuando
 * se supera el margen, evitando cambios inestables por ruido termico o electrico.
 *
 * Logica de transicion:
 *   AZUL  → VERDE : T >= 25.5 °C
 *   VERDE → AZUL  : T <  24.5 °C
 *   VERDE → ROJO  : T >  35.5 °C
 *   ROJO  → VERDE : T <= 34.5 °C
 */

#define TEMP_AZUL_A_VERDE    (25.5f)     /* Umbral de subida: la NTC debe calentar mas de 25.5 °C para pasar a verde. */
#define TEMP_VERDE_A_AZUL    (24.5f)     /* Umbral de bajada: la NTC debe enfriarse bajo 24.5 °C para volver a azul. */
#define TEMP_VERDE_A_ROJO    (35.5f)     /* Umbral de subida: la NTC debe calentar mas de 35.5 °C para pasar a rojo. */
#define TEMP_ROJO_A_VERDE    (34.5f)     /* Umbral de bajada: la NTC debe enfriarse a 34.5 °C o menos para volver a verde. */

/* ========================================================================== */
/*                         Tiempos de ejecucion                               */
/* ========================================================================== */

#define POLL_DELAY_MS        (500)        /* Pausa entre ciclos: 500 ms es suficiente para seguir cambios termicos lentos. */
#define PRINT_DELAY_CYCLES   (4)          /* Imprime cada 5 ciclos (2.5 s) para no saturar el puerto serial. */

/* ========================================================================== */
/*                         Estado de color del LED                            */
/* ========================================================================== */

typedef enum {                            /* Tipo enumerado que representa el color activo del LED RGB. */
    COLOR_AZUL  = 0,                      /* Estado frio: temperatura por debajo del umbral inferior. */
    COLOR_VERDE,                          /* Estado templado: temperatura dentro del rango normal. */
    COLOR_ROJO                            /* Estado caliente: temperatura por encima del umbral superior. */
} color_estado_t;                         /* Nombre del tipo para usarlo en el resto del programa. */

/* ========================================================================== */
/*                    Configuracion del ADC y su calibracion                  */
/* ========================================================================== */

static void config_ntc_adc(                                          /* Configura la unidad ADC y crea el esquema de calibracion. */
    adc_oneshot_unit_handle_t *adc_handle,                           /* Puntero donde se guardara eL ADC creado. */
    adc_cali_handle_t         *cali_handle)                          /* Puntero donde se guardara el manejador de calibracion. */
{                                                                    /* Inicia el bloque de codigo de config_ntc_adc. */
    adc_oneshot_unit_init_cfg_t init_cfg = {                         /* Estructura de inicializacion de la unidad ADC. */
        .unit_id = NTC_ADC_UNIT                                      /* Selecciona la unidad ADC1 donde esta el canal de la NTC. */
    };                                                               /* Termina la configuracion de la unidad. */

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, adc_handle));    /* Crea la unidad ADC y guarda su manejador. */

    adc_oneshot_chan_cfg_t chan_cfg = {                               /* Estructura de configuracion del canal ADC. */
        .atten    = NTC_ADC_ATTEN,                                   /* Atenuacion para medir el rango completo del divisor de voltaje. */
        .bitwidth = NTC_ADC_WIDTH                                    /* Resolucion de 12 bits para tener suficiente precision. */
    };                                                               /* Termina la configuracion del canal. */

    ESP_ERROR_CHECK(adc_oneshot_config_channel(                      /* Aplica la configuracion al canal fisico de la NTC. */
        *adc_handle,                                                 /* Manejador de la unidad ADC recien creada. */
        NTC_ADC_CHANNEL,                                             /* Canal ADC4 conectado a GPIO4. */
        &chan_cfg                                                     /* Configuracion de atenuacion y resolucion. */
    ));                                                              /* Cierra la configuracion del canal. */

    /*
     * Por que calibrar: el ADC del ESP32-C6 tiene no linealidades de hasta ±100 mV
     * sin calibracion. adc_cali_create_scheme_curve_fitting usa constantes grabadas
     * en eFuse durante la fabricacion del chip para corregir esa curva y entregar
     * un voltaje real con error menor a ±20 mV en todo el rango.
     */
    adc_cali_curve_fitting_config_t cali_cfg = {                     /* Estructura de configuracion del esquema de calibracion. */
        .unit_id  = NTC_ADC_UNIT,                                    /* Unidad ADC a la que pertenece el canal calibrado. */
        .chan     = NTC_ADC_CHANNEL,                                 /* Canal especifico para el que se crea la calibracion. */
        .atten    = NTC_ADC_ATTEN,                                   /* Debe coincidir con la atenuacion configurada en el canal. */
        .bitwidth = NTC_ADC_WIDTH                                    /* Debe coincidir con la resolucion configurada en el canal. */
    };                                                               /* Termina la configuracion de calibracion. */

    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(            /* Crea el esquema de calibracion curve-fitting para ESP32-C6. */
        &cali_cfg,                                                   /* Configuracion del esquema de calibracion. */
        cali_handle                                                  /* Puntero donde se guardara el manejador de calibracion. */
    ));                                                              /* Cierra la creacion del esquema de calibracion. */
}                                                                    /* Termina la funcion config_ntc_adc. */

/* ========================================================================== */
/*               Lectura promediada del ADC con calibracion                   */
/* ========================================================================== */

static int leer_voltaje_mv(                                          /* Promedia N lecturas crudas y las convierte a milivoltios calibrados. */
    adc_oneshot_unit_handle_t adc_handle,                            /* Manejador de la unidad ADC configurada. */
    adc_cali_handle_t         cali_handle)                           /* Manejador del esquema de calibracion. */
{                                                                    /* Inicia el bloque de codigo de leer_voltaje_mv. */
    int suma       = 0;                                              /* Acumulador de las N lecturas crudas para calcular el promedio. */
    int muestra    = 0;                                              /* Almacena cada lectura cruda individual del ADC. */
    int voltage_mv = 0;                                              /* Almacena el voltaje en mV tras aplicar la calibracion. */

    for (int i = 0; i < N_MUESTRAS_ADC; i++) {                      /* Repite la lectura N veces para promediar y reducir el ruido. */
        ESP_ERROR_CHECK(adc_oneshot_read(                            /* Toma una muestra cruda del canal ADC de la NTC. */
            adc_handle,                                              /* Manejador de la unidad ADC. */
            NTC_ADC_CHANNEL,                                         /* Canal conectado a la NTC. */
            &muestra                                                 /* Destino de la lectura cruda (0 a 4095). */
        ));                                                          /* Cierra la lectura y valida errores. */
        suma += muestra;                                             /* Acumula la muestra para calcular el promedio al final. */
    }                                                                /* Termina el ciclo de oversampling. */

    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(                         /* Convierte el promedio crudo a voltaje real usando la curva del chip. */
        cali_handle,                                                 /* Manejador del esquema de calibracion. */
        suma / N_MUESTRAS_ADC,                                       /* Promedio de las N lecturas crudas. */
        &voltage_mv                                                  /* Destino del voltaje calibrado en milivoltios. */
    ));                                                              /* Cierra la conversion y valida errores. */

    return voltage_mv;                                               /* Devuelve el voltaje calibrado en mV listo para calcular temperatura. */
}                                                                    /* Termina la funcion leer_voltaje_mv. */

/* ========================================================================== */
/*              Conversion de voltaje calibrado a temperatura                 */
/* ========================================================================== */

static float voltaje_mv_a_celsius(int voltage_mv)                   /* Convierte el voltaje del divisor a temperatura en grados Celsius. */
{                                                                    /* Inicia el bloque de codigo de voltaje_mv_a_celsius. */
    if (voltage_mv <= 0) {                                           /* Evita division por cero si el ADC lee tension nula (NTC abierta). */
        voltage_mv = 1;                                              /* Sustituye por 1 mV para que el calculo sea matematicamente valido. */
    }                                                                /* Termina la proteccion del limite inferior. */

    if (voltage_mv >= NTC_VCC_MV) {                                  /* Evita division por cero si el voltaje iguala o supera VCC (NTC en cortocircuito). */
        voltage_mv = NTC_VCC_MV - 1;                                 /* Sustituye por VCC - 1 mV para mantener el calculo valido. */
    }                                                                /* Termina la proteccion del limite superior. */

    /*
     * Resistencia de la NTC a partir del divisor de voltaje:
     *   R_ntc = R_serie * V_adc / (VCC - V_adc)
     * Usar voltaje calibrado en lugar de raw elimina la no linealidad del ADC.
     */
    float r_ntc = NTC_R_SERIE * (float)voltage_mv /                 /* Calcula la resistencia real de la NTC usando el voltaje calibrado. */
                  (float)(NTC_VCC_MV - voltage_mv);

    /*
     * Ecuacion Beta de Steinhart-Hart simplificada:
     *   1/T [K] = 1/T0 + (1/Beta) * ln(R / R0)
     */
    float inv_T = (1.0f / NTC_T0_KELVIN) +                          /* Termino de referencia a 25 °C en escala Kelvin. */
                  (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0);         /* Termino de correccion por la desviacion de resistencia respecto a R0. */

    return (1.0f / inv_T) - 273.15f;                                /* Convierte el resultado de Kelvin a grados Celsius y lo devuelve. */
}                                                                    /* Termina la funcion voltaje_mv_a_celsius. */

/* ========================================================================== */
/*              Maquina de estados con histeresis de temperatura              */
/* ========================================================================== */

static color_estado_t actualizar_estado(                             /* Evalua la temperatura y retorna el nuevo estado de color con histeresis. */
    color_estado_t estado,                                           /* Estado de color actual antes de revisar la temperatura. */
    float          temperatura)                                      /* Temperatura medida en grados Celsius en este ciclo. */
{                                                                    /* Inicia el bloque de codigo de actualizar_estado. */
    switch (estado) {                                                /* Evalua el estado actual para aplicar los umbrales correctos. */

    case COLOR_AZUL:                                                 /* Cuando el LED esta en azul solo puede subir a verde. */
        if (temperatura >= TEMP_AZUL_A_VERDE) {                     /* Revisa si la temperatura supero el umbral de subida (25.5 °C). */
            return COLOR_VERDE;                                      /* Transiciona a verde porque ya supero la zona de histeresis. */
        }                                                            /* Termina la condicion de subida desde azul. */
        break;                                                       /* Sale del switch si no hay transicion. */

    case COLOR_VERDE:                                                /* Cuando el LED esta en verde puede bajar a azul o subir a rojo. */
        if (temperatura < TEMP_VERDE_A_AZUL) {                      /* Revisa si la temperatura bajo del umbral de bajada (24.5 °C). */
            return COLOR_AZUL;                                       /* Transiciona a azul porque bajo de la zona de histeresis inferior. */
        }                                                            /* Termina la condicion de bajada desde verde. */
        if (temperatura > TEMP_VERDE_A_ROJO) {                      /* Revisa si la temperatura supero el umbral de subida (35.5 °C). */
            return COLOR_ROJO;                                       /* Transiciona a rojo porque supero la zona de histeresis superior. */
        }                                                            /* Termina la condicion de subida desde verde. */
        break;                                                       /* Sale del switch si la temperatura permanece en la zona verde. */

    case COLOR_ROJO:                                                 /* Cuando el LED esta en rojo solo puede bajar a verde. */
        if (temperatura <= TEMP_ROJO_A_VERDE) {                     /* Revisa si la temperatura bajo al umbral de bajada (34.5 °C). */
            return COLOR_VERDE;                                      /* Transiciona a verde porque entro de nuevo a la zona normal. */
        }                                                            /* Termina la condicion de bajada desde rojo. */
        break;                                                       /* Sale del switch si no hay transicion. */
    }                                                                /* Termina la evaluacion de la maquina de estados. */

    return estado;                                                   /* Devuelve el mismo estado si ninguna condicion de transicion se cumplio. */
}                                                                    /* Termina la funcion actualizar_estado. */

/* ========================================================================== */
/*                    Aplicacion del color al LED RGB                         */
/* ========================================================================== */

static void aplicar_color(led_rgb_t *led_rgb, color_estado_t estado) /* Enciende el canal del LED que corresponde al estado de temperatura. */
{                                                                    /* Inicia el bloque de codigo de aplicar_color. */
    switch (estado) {                                                /* Evalua el estado para decidir que canal encender. */
    case COLOR_AZUL:                                                 /* Temperatura fria: enciende solo el canal azul. */
        set_led_rgb_percentage_given_values(led_rgb, 0, 0, 100);    /* Rojo 0%, Verde 0%, Azul 100%. */
        break;                                                       /* Sale del switch despues de encender azul. */
    case COLOR_VERDE:                                                /* Temperatura normal: enciende solo el canal verde. */
        set_led_rgb_percentage_given_values(led_rgb, 0, 100, 0);    /* Rojo 0%, Verde 100%, Azul 0%. */
        break;                                                       /* Sale del switch despues de encender verde. */
    case COLOR_ROJO:                                                 /* Temperatura alta: enciende solo el canal rojo. */
        set_led_rgb_percentage_given_values(led_rgb, 100, 0, 0);    /* Rojo 100%, Verde 0%, Azul 0%. */
        break;                                                       /* Sale del switch despues de encender rojo. */
    }                                                                /* Termina la seleccion de canal. */
}                                                                    /* Termina la funcion aplicar_color. */

/* ========================================================================== */
/*                       Texto descriptivo del estado                         */
/* ========================================================================== */

static const char *estado_a_texto(color_estado_t estado)            /* Devuelve una cadena con el nombre del estado para el monitor serial. */
{                                                                    /* Inicia el bloque de codigo de estado_a_texto. */
    switch (estado) {                                                /* Evalua el estado para elegir el texto correspondiente. */
    case COLOR_AZUL:  return "Azul  (T < 25 C)";                    /* Texto del estado frio con su rango de temperatura. */
    case COLOR_VERDE: return "Verde (25 C <= T <= 35 C)";           /* Texto del estado normal con su rango de temperatura. */
    case COLOR_ROJO:  return "Rojo  (T > 35 C)";                    /* Texto del estado caliente con su rango de temperatura. */
    default:          return "Desconocido";                          /* Texto de seguridad para valores de estado inesperados. */
    }                                                                /* Termina la seleccion del texto. */
}                                                                    /* Termina la funcion estado_a_texto. */

/* ========================================================================== */
/*                              Programa principal                            */
/* ========================================================================== */

void app_main(void)                                                  /* Funcion principal que ESP-IDF ejecuta al iniciar el sistema. */
{                                                                    /* Inicia el bloque de codigo de app_main. */
    led_rgb_t led_rgb = {                                            /* Estructura que describe la configuracion completa del LED RGB. */
        .led_red = {                                                 /* Configuracion del canal rojo. */
            .duty     = 0,                                           /* Rojo apagado al inicio. */
            .gpio_num = LED_RGB_RED_GPIO,                            /* GPIO fisico del anodo rojo. */
            .channel  = LEDC_CHANNEL_0                               /* Canal PWM 0 asignado al rojo. */
        },                                                           /* Termina la configuracion del canal rojo. */
        .led_green = {                                               /* Configuracion del canal verde. */
            .duty     = 0,                                           /* Verde apagado al inicio. */
            .gpio_num = LED_RGB_GREEN_GPIO,                          /* GPIO fisico del anodo verde. */
            .channel  = LEDC_CHANNEL_1                               /* Canal PWM 1 asignado al verde. */
        },                                                           /* Termina la configuracion del canal verde. */
        .led_blue = {                                                /* Configuracion del canal azul. */
            .duty     = 0,                                           /* Azul apagado al inicio. */
            .gpio_num = LED_RGB_BLUE_GPIO,                           /* GPIO fisico del anodo azul. */
            .channel  = LEDC_CHANNEL_2                               /* Canal PWM 2 asignado al azul. */
        },                                                           /* Termina la configuracion del canal azul. */
        .timer           = LEDC_TIMER_0,                             /* Timer LEDC 0 compartido por los tres canales PWM. */
        .duty_resolution = LEDC_TIMER_13_BIT,                        /* 13 bits de resolucion: permite 8192 niveles de brillo. */
        .frequency       = 4000,                                     /* 4000 Hz: frecuencia PWM por encima del umbral de parpadeo visible. */
        .speed_mode      = LEDC_LOW_SPEED_MODE                       /* Modo de baja velocidad: unico disponible en ESP32-C6. */
    };                                                               /* Termina la estructura del LED RGB. */

    adc_oneshot_unit_handle_t adc_handle  = NULL;                    /* Manejador de la unidad ADC, inicializado en NULL antes de configurar. */
    adc_cali_handle_t         cali_handle = NULL;                    /* Manejador de calibracion, inicializado en NULL antes de configurar. */
    int              voltage_mv  = 0;                                /* Voltaje calibrado en mV leido en cada ciclo. */
    float            temperatura = 0.0f;                             /* Temperatura calculada en grados Celsius en cada ciclo. */
    color_estado_t   estado      = COLOR_AZUL;                       /* Estado inicial: azul, asumiendo temperatura fria al arrancar. */
    uint32_t         contador    = 0;                                /* Contador de ciclos para controlar la frecuencia de impresion. */

    config_ntc_adc(&adc_handle, &cali_handle);                       /* Inicializa el ADC y crea el esquema de calibracion. */
    config_led_rgb(&led_rgb);                                        /* Configura el timer LEDC y los tres canales PWM del LED RGB. */
    aplicar_color(&led_rgb, estado);                                 /* Aplica el estado inicial al LED antes de entrar al bucle. */

    printf("=== Control RGB por temperatura NTC ===\n");             /* Encabezado del sistema en el monitor serial. */
    printf("T < 25 C         -> LED Azul\n");                        /* Indica la condicion del rango frio. */
    printf("25 C <= T <= 35 C -> LED Verde\n");                      /* Indica la condicion del rango normal. */
    printf("T > 35 C         -> LED Rojo\n");                        /* Indica la condicion del rango caliente. */
    printf("Histeresis: ±0.5 C en cada umbral\n");                   /* Informa que los umbrales tienen zona muerta de ±0.5 °C. */
    printf("Oversampling: %d muestras por ciclo\n", N_MUESTRAS_ADC); /* Informa el numero de muestras promediadas por lectura. */
    printf("NTC: ADC_CH%d (GPIO4) | LED R:%d G:%d B:%d\n\n",        /* Informa los pines fisicos usados por el sistema. */
           NTC_ADC_CHANNEL, LED_RGB_RED_GPIO,
           LED_RGB_GREEN_GPIO, LED_RGB_BLUE_GPIO);

    while (1) {                                                      /* Bucle infinito principal del sistema embebido. */
        voltage_mv  = leer_voltaje_mv(adc_handle, cali_handle);      /* Toma 16 muestras, las promedia y aplica la calibracion. */
        temperatura = voltaje_mv_a_celsius(voltage_mv);              /* Convierte el voltaje calibrado a temperatura en grados Celsius. */
        estado      = actualizar_estado(estado, temperatura);        /* Evalua si la temperatura cruzo algun umbral con histeresis. */

        aplicar_color(&led_rgb, estado);                             /* Actualiza el PWM del LED RGB segun el nuevo estado de color. */

        if (contador >= PRINT_DELAY_CYCLES) {                        /* Revisa si pasaron suficientes ciclos para imprimir. */
            contador = 0;                                            /* Reinicia el contador de impresion. */
            printf("V: %4d mV | Temp: %6.2f C | Color: %s\n",      /* Imprime el diagnostico completo del sistema. */
                   voltage_mv, temperatura, estado_a_texto(estado));
        } else {                                                     /* No es momento de imprimir: solo incrementa el contador. */
            contador++;                                              /* Avanza el contador de ciclos sin impresion. */
        }                                                            /* Termina la logica de impresion periodica. */

        vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));                    /* Pausa 500 ms: cede el CPU y permite que el RTOS atienda otras tareas. */
    }                                                                /* Termina una vuelta del bucle y regresa al inicio. */
}                                                                    /* Termina la funcion app_main. */
