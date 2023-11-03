/**
 * Practica 2 - Tablero de control y transmision de estado.
 *
 * Conexiones:
 *
 * P1.1 (UART TX) -> Rx de modulo HC06.
 * P1.2 (UART RX) -> Tx de modulo GPS NEO-6.
 *
 * P1.3 (A3) -> Valor analogico con voltaje de la bateria.
 *
 * P1.4 (A4) -> Potenciometro de control de direccion (volante).
 * P1.5 (A5) -> Potenciometro de control de velocidad (pedal).
 *
 * P2.6 - ENA, PWM para velocidad del motor CD.
 * P2.0 - IN1 Motor CD
 * P2.2 - IN2 Motor CD
 * P2.3, P2.4, P2.5 y P2.7 -> In4 - In1 de motor a pasos para controlar direccion.
 *
 * P2.1 -> Input capture para medir velocidad de rotacion del motor CD.
 *
 * Timer A0 dedicado a PWM.
 * Timer A1 dedicado a Input Capture, ADC, stepper y tiempos internos.
 *
 * USCI A en modo UART (8/n/1).
 */
#include <msp430.h>
#include <stdint.h>

/**
 * Control de motores, incluyendo Input Capture.
 */
#define PERIODO_PWM_MOTOR_CD    (1000u - 1u)
#define PIN_PWM_MOTOR_CD        (BIT6)
#define PIN_MOTOR_CD_IN1        (BIT0)
#define PIN_MOTOR_CD_IN2        (BIT2)
#define PIN_INPUT_CAPTURE_VEL   (BIT1)

#define PINES_MOTOR_DIRECCION   (BIT3 | BIT4 | BIT5 | BIT7)

#define LEN_SECUENCIA_STEPPER   (8u)
#define PERIODO_VAL_STEPPER_MS  (1u)

/**
 * ADC
 */
#define PIN_ADC_IN_BATERIA      (BIT3)
#define CANAL_ADC_IN_BATERIA    (INCH_3)
#define PIN_ADC_IN_VOLANTE      (BIT4)
#define CANAL_ADC_IN_VOLANTE    (INCH_4)
#define PIN_ADC_IN_PEDAL        (BIT5)
#define CANAL_ADC_IN_PEDAL      (INCH_5)
#define ADC_INCH_MASK           (0xF000u)

#define PERIODO_SAMPLE_ADC_MS   (20u)

#define BATERIA_MIN_MV          (3600u)
#define BATERIA_MAX_MV          (6400u)

/**
 * GPS
 */
#define MIN_NUM_SATELITES_GPS   (4u)
#define MAX_NUM_SATELITES_GPS   (32u)

#define GPS_LAT_DIR_FLAG        (BIT0)
#define GPS_LON_DIR_FLAG        (BIT1)

/**
 * UART y comunicacion de estado
 */
#define PERIODO_TX_ESTADO_MS    (250u)

#define PINES_UART_TX           (BIT1 | BIT2)
#define DIVISOR_BITRATE_UART    (104u)

#define PAYLOAD_START_BYTE      (0x9Au)

#define UART_TX_BUF_LEN         (16u)

#define PAYLOAD_LEN_BYTE        (1u)
#define GPS_STATE_PAYLOAD_LEN   (11u)
#define SPEED_STATE_PAYLOAD_LEN (2u)
#define BATT_STATE_PAYLOAD_LEN  (3u)

#define LAT_INT_MSB (2u)
#define LAT_INT_LSB (3u)
#define LAT_DEC_MSB (4u)
#define LAT_DEC_XSB (5u)
#define LAT_DEC_LSB (6u)
#define LON_INT_MSB (7u)
#define LON_INT_LSB (8u)
#define LON_DEC_MSB (9u)
#define LON_DEC_XSB (10u)
#define LON_DEC_LSB (11u)
#define ESTADO_GPS  (12u)

#define VEL_RPM_MSB (2u)
#define VEL_RPM_LSB (3u)

#define BATT_MV_MSB (2u)
#define BATT_MV_LSB (3u)

// Banderas globales del sistema, indican estado.
#define UART_TX_IDLE                    (BIT3)
#define BATERIA_DISPONIBLE_FLAG         (BIT4)
#define VELOCIDAD_DISPONIBLE_FLAG       (BIT5)
#define COORD_GPS_DISPONIBLES_FLAG      (BIT6)
#define VALOR_STEPPER_PENDIENTE_FLAG    (BIT7)

/**
 * Funciones de utilidad y locales de este archivo.
 */
static void adc10_init(void);

static void pwm_init(void);
static void motors_init(void);
static void input_capture_init(void);

static void timers_start(void);

static void stepper_send_next_val(void);

static void uart_init(void);
static void transmit_buf_write_gps(void);
static void transmit_buf_write_speed(void);
static void transmit_buf_write_battery(void);

/**
 * Variables de estado global.
 */
static volatile uint8_t banderas_sistema = 0b00000000;

static volatile uint8_t transmit_buffer[UART_TX_BUF_LEN] = {};

// 1 vuelta = 8 valores * 512 pasos = 4096 valores
// Media vuelta = 2048 valores
static volatile uint16_t posicion_deseada = 1024u;

// Duty cycle para la velocidad objetivo de los motores CD (0% - 100%)
static volatile uint8_t duty_cycle_velocidad = 0u;

// El tiempo (ms) que tarda en girar 1/10 de revolucion la rueda.
static volatile uint16_t periodo_fracc_revolucion_ms = 0u;

static volatile uint16_t voltaje_bateria_mv = BATERIA_MAX_MV;

// Variables globales de estado del GPS.
static uint16_t lat_int;
static uint32_t lat_dec;
static uint16_t lon_int;
static uint32_t lon_dec;
static uint8_t n_sat;
static uint8_t gps_flags;

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer

    uart_init();

    adc10_init();

    pwm_init();

    motors_init();

    input_capture_init();

    timers_start();

    __bis_SR_register(GIE);

    while (1)
    {
        if (VALOR_STEPPER_PENDIENTE_FLAG & banderas_sistema)
        {
            // En vez de enviar el valor del stepper desde la ISR del timer,
            // se "pospone" hasta que vuelve a ejecutar el loop infinito de main().
            banderas_sistema &= ~VALOR_STEPPER_PENDIENTE_FLAG;
            stepper_send_next_val();
        }

        if (UART_TX_IDLE & banderas_sistema)
        {
            if (COORD_GPS_DISPONIBLES_FLAG & banderas_sistema)
            {
                // Escribir datos GPS recibidos en el buffer de estado que se envia por UART.
                banderas_sistema &= ~(COORD_GPS_DISPONIBLES_FLAG | UART_TX_IDLE);
                transmit_buf_write_gps();

                IE2 |= UCA0TXIE;
            }
            else if (BATERIA_DISPONIBLE_FLAG & banderas_sistema)
            {
                banderas_sistema &= ~(BATERIA_DISPONIBLE_FLAG | UART_TX_IDLE);
                transmit_buf_write_battery();

                IE2 |= UCA0TXIE;
            }
            else if (VELOCIDAD_DISPONIBLE_FLAG & banderas_sistema)
            {
                banderas_sistema &= ~(VELOCIDAD_DISPONIBLE_FLAG | UART_TX_IDLE);
                transmit_buf_write_speed();

                IE2 |= UCA0TXIE;
            }
        }
    }

    return 0;
}

#pragma vector=ADC10_VECTOR
__interrupt void adc10_isr(void)
{
    // Leer valor del canal ADC muestreado, luego cambiar al
    // otro canal.
    uint16_t canal_adc_muestreado = ADC10CTL1 & ADC_INCH_MASK;

    switch (canal_adc_muestreado)
    {
    case CANAL_ADC_IN_PEDAL:
        duty_cycle_velocidad = (((uint32_t) ADC10MEM) * 100u) / 1023u;
        TA0CCR1 = (duty_cycle_velocidad * 10u);

        canal_adc_muestreado = CANAL_ADC_IN_VOLANTE;

        break;
    case CANAL_ADC_IN_VOLANTE:
        posicion_deseada = ADC10MEM << 1u;

        canal_adc_muestreado = CANAL_ADC_IN_BATERIA;
        break;
    case CANAL_ADC_IN_BATERIA:
    {
        // Para cuatro baterias AA, 100% = 6.4v y 0% = 3.6v.
        uint16_t nuevo_voltaje_bateria = ((((uint32_t) ADC10MEM) * 3200) / 1023u) << 1u;

        if (BATERIA_MIN_MV > nuevo_voltaje_bateria)
        {
            nuevo_voltaje_bateria = BATERIA_MIN_MV;
        }

        canal_adc_muestreado = CANAL_ADC_IN_PEDAL;

        // El voltaje de la bateria solo puede bajar.
        if (nuevo_voltaje_bateria < voltaje_bateria_mv)
        {
            voltaje_bateria_mv = nuevo_voltaje_bateria;
            banderas_sistema |= BATERIA_DISPONIBLE_FLAG;
        }
    }
        break;
    }

    ADC10CTL0 &= ~ENC;
    ADC10CTL1 &= ~ADC_INCH_MASK;
    ADC10CTL1 |= canal_adc_muestreado;
    ADC10CTL0 |= ENC;
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void timer_a1_ccr0_isr(void)
{
    static uint16_t ms_antes_de_conversion_adc = PERIODO_SAMPLE_ADC_MS;
    static uint16_t ms_antes_de_valor_stepper = PERIODO_VAL_STEPPER_MS;
    static uint16_t ms_antes_de_tx_estado = PERIODO_TX_ESTADO_MS;

    ms_antes_de_conversion_adc--;

    if (0u == ms_antes_de_conversion_adc)
    {
        // Iniciar la siguiente conversion del ADC10.
        ADC10CTL0 |= ADC10SC;
        ms_antes_de_conversion_adc = PERIODO_SAMPLE_ADC_MS;
    }

    ms_antes_de_valor_stepper--;

    if (0u == ms_antes_de_valor_stepper)
    {
        // Notificar al main que debe enviar el siguiente valor del stepper.
        ms_antes_de_valor_stepper = PERIODO_VAL_STEPPER_MS;
        banderas_sistema |= VALOR_STEPPER_PENDIENTE_FLAG;
    }

    ms_antes_de_tx_estado--;

    if (0u == ms_antes_de_tx_estado)
    {
        ms_antes_de_tx_estado = PERIODO_TX_ESTADO_MS;
        banderas_sistema |= UART_TX_IDLE;
    }

    TA1CCR0 += 1000u;
}

#pragma vector=TIMER1_A1_VECTOR
__interrupt void timer_a1_a1_isr(void)
{
    // Esta ISR es invocada para CCR1, CCR2 y TAIFG (overflow).
    static volatile uint16_t cont_overflow_timer = 0u;
    static uint8_t i = 0u;
    static uint16_t flancos[2u] = {};

    switch (TA1IV)
    {
    case TA1IV_TACCR1:
        if (CCI & TA1CCTL1)
        {
            flancos[i++] = TA1CCR1;

            if (2u <= i)
            {
                uint32_t nuevo_periodo_us;
                i = 0u;

                if (flancos[0] > flancos[1])
                {
                    if (0u != cont_overflow_timer)
                    {
                        cont_overflow_timer--;
                    }

                    nuevo_periodo_us = ((uint32_t) (flancos[1] - flancos[0])) + (0xFFFFu * cont_overflow_timer);
                }
                else
                {
                    nuevo_periodo_us = ((uint32_t) (flancos[0] - flancos[1])) + (0xFFFFu * cont_overflow_timer);
                }

                if (nuevo_periodo_us != periodo_fracc_revolucion_ms)
                {
                    periodo_fracc_revolucion_ms = nuevo_periodo_us / 1000u;
                    banderas_sistema |= VELOCIDAD_DISPONIBLE_FLAG;
                }
            }

            cont_overflow_timer = 0u;
        }
        break;
    case TA1IV_TAIFG:
        cont_overflow_timer++;
        break;
    }

}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void usci_tx_isr(void)
{
    static uint8_t index_byte_trama = 0u;

    UCA0TXBUF = transmit_buffer[index_byte_trama];
    index_byte_trama++;

    if (index_byte_trama >= (transmit_buffer[PAYLOAD_LEN_BYTE] + 2u))
    {
        index_byte_trama = 0u;

        IE2 &= ~UCA0TXIE;
        IE2 |= UCA0RXIE;
    }
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void usci_rx_isr(void)
{
    static uint8_t parser_state = 0u;
    static uint8_t n = 0u;
    static uint8_t n_g = 0u;

    uint8_t c = UCA0RXBUF;

    switch (parser_state)
    {
    case 0:
        if ('$' == c)
        {
            n = 0u;
            n_g = 0u;
            parser_state = 1u;
        }
        break;
    case 1:
        n++;
        if ('G' == c) n_g++;

        if (3u == n_g)
        {
            lat_int = 0u;
            lat_dec = 0u;
            lon_int = 0u;
            lon_dec = 0u;
            n_sat = 0u;
            gps_flags = 0x00;
            parser_state = 2u;
        }
        else if (5u <= n) parser_state = 0u;
        break;
    case 2:
        if (',' == c) parser_state = 3u;
        break;
    case 3:
        if (',' == c) parser_state = 4u;
        break;
    case 4:
        if ('.' == c) parser_state = 5u;
        else lat_int = (lat_int * 10u) + (c - '0');
        break;
    case 5:
        if (',' == c) parser_state = 6u;
        else lat_dec = (lat_dec * 10u) + (c - '0');
        break;
    case 6:
        if ('N' == c || 'S' == c)
        {
            gps_flags |= (('S' == c) & GPS_LAT_DIR_FLAG);
            parser_state = 7u;
        }
        break;
    case 7:
        if (',' == c) parser_state = 8u;
        break;
    case 8:
        if ('.' == c) parser_state = 9u;
        else lon_int = (lon_int * 10u) + (c - '0');
        break;
    case 9:
        if (',' == c) parser_state = 10u;
        else lon_dec = (lon_dec * 10u) + (c - '0');
        break;
    case 10:
        if ('W' == c || 'E' == c)
        {
            gps_flags |= ((('E' == c) << 1) & GPS_LON_DIR_FLAG);
            parser_state = 11u;
        }
        break;
    case 11:
        if (',' == c) parser_state = 12u;
        break;
    case 12:
        if (',' == c) parser_state = 13u;
        break;
    case 13:
        if (',' == c)
        {
            if (MIN_NUM_SATELITES_GPS <= n_sat && n_sat <= MAX_NUM_SATELITES_GPS)
            {
                // Los datos de GPS son coherentes, hay suficientes satelites visibles.
                banderas_sistema |= COORD_GPS_DISPONIBLES_FLAG;
            }

            parser_state = 0u;
        }
        else n_sat = (n_sat * 10u) + (c - '0');
        break;
    }
}

void adc10_init(void)
{
    // Configurar el ADC en modo "un canal - una muestra".
    ADC10AE0 |= (PIN_ADC_IN_VOLANTE | PIN_ADC_IN_PEDAL | PIN_ADC_IN_BATERIA);
    ADC10CTL1 |= CANAL_ADC_IN_PEDAL;

    // Sample-Hold time, MultipleSampleConversion, ADC10ON, ADC10 Interrupt Enable
    ADC10CTL0 |= (ADC10SHT_2 | MSC | ADC10ON | ADC10IE | ENC);
}

void pwm_init(void)
{
    // Configurar funcionalidad PWM en pines seleccionados de P2.
    P2SEL |= PIN_PWM_MOTOR_CD;
    P2SEL &= ~BIT7;
    P2SEL2 &= ~(PIN_PWM_MOTOR_CD | BIT7);

    P2DIR |= PIN_PWM_MOTOR_CD;

    // Definir periodo y duty cycle del PWM generado.
    TA0CCR0 = PERIODO_PWM_MOTOR_CD;

    TA0CCR1 = 0u;
    TA0CCTL1 |= OUTMOD_7;
}

void motors_init(void)
{
    // Configurar el puerto 2 para controlar el motor a pasos y los motores CD.
    P2SEL &= ~(PINES_MOTOR_DIRECCION | PIN_MOTOR_CD_IN1 | PIN_MOTOR_CD_IN2);
    P2SEL2 &= ~(PINES_MOTOR_DIRECCION | PIN_MOTOR_CD_IN1 | PIN_MOTOR_CD_IN2);

    P2DIR |= (PINES_MOTOR_DIRECCION | PIN_MOTOR_CD_IN1 | PIN_MOTOR_CD_IN2);

    // Los motores CD (por ahora) no tienen reversa, configurar estado de pines IN desde el
    // inicio.
    P2OUT &= ~(PIN_MOTOR_CD_IN1 | PIN_MOTOR_CD_IN2);
    P2OUT |= PIN_MOTOR_CD_IN1;
}

void input_capture_init(void)
{
    P2SEL |= PIN_INPUT_CAPTURE_VEL;
    P2SEL2 &= ~PIN_INPUT_CAPTURE_VEL;

    P2DIR &= ~PIN_INPUT_CAPTURE_VEL;

    TA1CCTL1 |= (CM_1 | CCIS_0 | SCS | CAP | CCIE);
}

void timers_start(void)
{
    // Preparar timer generico para tiempos de ADC, IC, UART, control de stepper.
    TA1CCR0 = TA1R + 1000u;
    TA1CCTL0 |= CCIE;

    // Habilitar Timer A0 para generacion de PWM (modo UP).
    TA0CTL |= (TASSEL_2 | MC_1 | TACLR);

    // Habilitar Timer A1 para muestras de ADC10, input capture, y control de stepper (modo continuo).
    TA1CTL |= (TASSEL_2 | MC_2 | TAIE | TACLR);
}

void stepper_send_next_val(void)
{
    static const uint8_t secuencia_stepper[LEN_SECUENCIA_STEPPER] = { 0x80, 0xA0, 0x20, 0x30, 0x10, 0x18, 0x08, 0x88 };
    static uint8_t i = 0u;

    static uint16_t posicion_actual = 1024u;

    P2OUT &= ~PINES_MOTOR_DIRECCION;

    if (posicion_actual < posicion_deseada)
    {
        i--;
        posicion_actual++;

        if (i >= LEN_SECUENCIA_STEPPER)
        {
            i = LEN_SECUENCIA_STEPPER - 1;
        }

        P2OUT |= secuencia_stepper[i];
    }
    else if (posicion_actual > posicion_deseada)
    {
        i++;
        posicion_actual--;

        if (i >= LEN_SECUENCIA_STEPPER)
        {
            i = 0u;
        }

        P2OUT |= secuencia_stepper[i];
    }
}

void uart_init(void)
{
    UCA0CTL1 |= UCSWRST;

    // Escribir una vez la secuencia de inicio para cada trama.
    transmit_buffer[0u] = PAYLOAD_START_BYTE;

    // 1 MHz, 9600 bps (usualmente).
    UCA0CTL1 |= UCSSEL_2;
    UCA0BR0 = DIVISOR_BITRATE_UART;
    UCA0BR1 = (DIVISOR_BITRATE_UART >> 8u);

    // Seleccionar funcion de UART para los pines TX y RX.
    P1SEL |= PINES_UART_TX;
    P1SEL2 |= PINES_UART_TX;

    UCA0CTL1 &= ~UCSWRST;

    // Habilitar por defecto las interrupciones de RX.
    IE2 |= UCA0RXIE;
}

void transmit_buf_write_gps(void)
{
    transmit_buffer[PAYLOAD_LEN_BYTE] = GPS_STATE_PAYLOAD_LEN;
    transmit_buffer[LAT_INT_MSB] = (lat_int >> 8u);
    transmit_buffer[LAT_INT_LSB] = lat_int;
    transmit_buffer[LAT_DEC_MSB] = (lat_dec >> 16u);
    transmit_buffer[LAT_DEC_XSB] = (lat_dec >> 8u);
    transmit_buffer[LAT_DEC_LSB] = lat_dec;
    transmit_buffer[LON_INT_MSB] = (lon_int >> 8u);
    transmit_buffer[LON_INT_LSB] = lon_int;
    transmit_buffer[LON_DEC_MSB] = (lon_dec >> 16u);
    transmit_buffer[LON_DEC_XSB] = (lon_dec >> 8u);
    transmit_buffer[LON_DEC_LSB] = lon_dec;
    transmit_buffer[ESTADO_GPS] = ((n_sat << 2u) | (gps_flags & GPS_LON_DIR_FLAG) | (gps_flags & GPS_LAT_DIR_FLAG));
}

void transmit_buf_write_speed(void)
{
    transmit_buffer[PAYLOAD_LEN_BYTE] = SPEED_STATE_PAYLOAD_LEN;
    transmit_buffer[VEL_RPM_MSB] = (periodo_fracc_revolucion_ms >> 8u);
    transmit_buffer[VEL_RPM_LSB] = periodo_fracc_revolucion_ms;
}

void transmit_buf_write_battery(void)
{
    transmit_buffer[PAYLOAD_LEN_BYTE] = BATT_STATE_PAYLOAD_LEN;
    transmit_buffer[BATT_MV_MSB] = (voltaje_bateria_mv >> 8u);
    transmit_buffer[BATT_MV_LSB] = voltaje_bateria_mv;
    transmit_buffer[BATT_MV_LSB] = 0x00u;
}
