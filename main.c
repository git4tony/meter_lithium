#include <inttypes.h>

#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <util/delay.h>
#include <util/atomic.h>

#include <string.h>

#include "printf.h"
#include "pt.h"

/*
     a
    ---
  f| g |b
    ---
  e|   |c
    ---   -
     d    h
*/

//CHARECTOR
#define CHAR_0 0x3f
#define CHAR_1 0x06
#define CHAR_2 0x5b
#define CHAR_3 0x4f
#define CHAR_4 0x66
#define CHAR_5 0x6d 
#define CHAR_6 0x7d 
#define CHAR_7 0x07 
#define CHAR_8 0x7f 
#define CHAR_9 0x6f  
   
#define CHAR_A 0x77
#define CHAR_C 0x39
#define CHAR_E 0x79
#define CHAR_F 0x71
#define CHAR_H 0x76
#define CHAR_I 0x30
#define CHAR_L 0x38
#define CHAR_O 0x3f
#define CHAR_P 0x73
#define CHAR_S 0x6d
#define CHAR_b 0x7c
#define CHAR_c 0x58
#define CHAR_d 0x5e
#define CHAR_i 0x10
#define CHAR_n 0x54
#define CHAR_o 0x5c
#define CHAR_r 0x50
#define CHAR_t 0x78
#define CHAR_u 0x1c
#define CHAR_BAR 0x40

#define EEPROM_WORD_BOOT_COUNT  (uint16_t *)2
#define EEPROM_LONG_UPTIME      (uint32_t *)4
#define EEPROM_BYTE_NEXT        (uint8_t *)8

struct timer { int start, interval; };

int  timer_expired(struct timer *t);
void timer_set(struct timer *t, int interval);
int tick_time(void);

unsigned long uptime_get(void);
unsigned short adc_read(unsigned mux);
long map(long x, long in_min, long in_max, long out_min, long out_max);

unsigned char hex_tab[16] = {
   0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f,
   CHAR_A, CHAR_b, CHAR_C, CHAR_d, CHAR_E, CHAR_F
};

volatile unsigned short tick1ms = 0;
volatile unsigned long uptime = 0;

#if 0
enum { UP, DOWN };
ISR (TIMER1_OVF_vect)
{
    static uint16_t pwm;
    static uint8_t direction;

    switch (direction)
    {
        case UP:
            if (++pwm == TIMER1_TOP)
                direction = DOWN;
            break;

        case DOWN:
            if (--pwm == 0)
                direction = UP;
            break;
    }

    OCR = pwm;
}

void ioinit (void)
{
    /* Timer 1 is 10-bit PWM (8-bit PWM on some ATtinys). */
    TCCR1A = TIMER1_PWM_INIT;
    /*
     * Start timer 1.
     *
     * NB: TCCR1A and TCCR1B could actually be the same register, so
     * take care to not clobber it.
     */
    TCCR1B |= TIMER1_CLOCKSOURCE;
    /*
     * Run any device-dependent timer 1 setup hook if present.
     */
#if defined(TIMER1_SETUP_HOOK)
    TIMER1_SETUP_HOOK();
#endif

    /* Set PWM value to 0. */
    OCR = 0;

    /* Enable OC1 as output. */
    DDROC = _BV (OC1);

    /* Enable timer 1 overflow interrupt. */
    TIMSK = _BV (TOIE1);    
}
#endif

#define TCNT1_VALUE (65536u - 125)

void timer1_init(void)
{
    TCCR1B = 0x00; //stop

    TCNT1 = TCNT1_VALUE;
    OCR1AH = 0x8F;
    OCR1AL = 0xFF;
    OCR1BH = 0x8F;
    OCR1BL = 0xFF;
    ICR1H  = 0x8F;
    ICR1L  = 0xFF;

    TCCR1A = 0x00;
    TCCR1B = (0<<CS12) | (1<<CS11) | (1<<CS10); //start Timer clk/64

    TIFR  |= (1<<TOV1);
    TIMSK |= (1<<TOIE1);

    DDRB |= (1<<PB1);
}

ISR(TIMER1_OVF_vect)
{
    TCNT1 = TCNT1_VALUE;

    PORTB ^= (1<<PB1);

    tick1ms++;
}

//async mode 32.768 kHz
void timer2_init(void)
{
    //Disable timer2 interrupts
    TIMSK  &= ~(1<<TOIE2);
    //Enable asynchronous mode
    ASSR  = (1<<AS2);
    //set initial counter value
    TCNT2 = 0;
    //set prescaller 128
    TCCR2 |= (1<<CS22)|(1<<CS00);
    
    //wait for registers update
    //while (ASSR & ((1<<TCN2UB) | (1<<TCR2UB)));

    TIFR  |= (1<<TOV2);
    TIMSK  |= (1<<TOIE2);
    
    DDRD |= (1<<PD6);
}

unsigned char is_timer2_busy(void)
{
    return (ASSR & ((1<<TCN2UB) | (1<<TCR2UB)));
}

//Overflow ISR
ISR(TIMER2_OVF_vect)
{
    //Toggle pin PD6 every second
    PORTD ^= (1<<PD6);

    uptime++;
}

void spi_init(void)
{
   DDRB |= (1<<DDB2) | (1<<DDB3) | (1<<DDB5); //spi pins on port B MOSI SCK,SS outputs
   DDRB &= ~(1<<DDB4);
   SPSR |= (1<<SPI2X);
   SPCR = (1<<SPE) | (1<<MSTR) | (0<<SPR1) | (1<<SPR0) | (0<<CPOL) | (0<<CPHA);  // SPI enable, Master, f/16, mode 00
}

unsigned char spi_transmit(unsigned char data)
{
    SPDR = data;
    while ( !(SPSR & (1<<SPIF)) )
        ;
    return SPDR;
}

// 19200 @8.00 MHz
void uart_init(void)
{
   UCSRB = 0x00;
   UCSRA = (1<<U2X);
   UCSRC = (1<<URSEL) | 0x06;
   UBRRL = 0x33;
   UBRRH = 0x00;
   UCSRB = 0x18;    
}

#if 1
unsigned char uart_buf[128];
volatile unsigned char uart_head = 0;
volatile unsigned char uart_tail = 0;

void uart_transmit(unsigned char data)
{
   unsigned char tmphead;
  
   /* calculate buffer index */
   tmphead = (uart_head + 1) & (sizeof(uart_buf) - 1);
   
   /* wait for free space in buffer */
	while ( tmphead == uart_tail )
		;
      
   uart_buf[tmphead] = data; /* store data in buffer */
   uart_head = tmphead; /* store new index */
   UCSRB |= (1<<UDRIE); /* enable UDRE interrupt */
}

ISR(USART_UDRE_vect)
{
   unsigned char tmptail;

   /* check if all data is transmitted */
   if ( uart_head != uart_tail ) {
      /* calculate buffer index */
      tmptail = (uart_tail + 1) & (sizeof(uart_buf) - 1);
      uart_tail = tmptail; /* store new index */
      UDR = uart_buf[tmptail]; /* start transmition */
   }
	else {
      UCSRB &= ~(1<<UDRIE); /* disable UDRE interrupt */
   }
}
#else
void uart_transmit(unsigned char data)
{
    while ( !( UCSRA & (1<<UDRE)) )
        ;
    UDR = data;
}
#endif

unsigned char uart_recieve(void)
{
    while ( !((UCSRA) & (1<<RXC)) )
        ;
    return UDR;
}

int putchar(int c)
{
    if ( c == '\n' )
        putchar('\r');
    
    uart_transmit(c);
    return c;
}

void print_ul(unsigned long num)
{
    unsigned long div;
    char dg, dz = 0;

    div = 1000000000ul;
    while ( div ) {
        dg = (num / div) % 10;        
        if ( dz == 0 ) {
            if ( dg > 0 )
                dz = 1;
        }

        if ( dz || (div == 1) ) {
            putchar('0' + dg);
        }
        div /= 10;
    }
}

#define RCK_BIT PD5
#define RCK_0   PORTD &= ~(1<<RCK_BIT)
#define RCK_1   PORTD |= (1<<RCK_BIT)

void shift(unsigned char *p, unsigned char n)
{
    RCK_0;
    while ( n ) {
        spi_transmit(p[--n]);
    }
    RCK_1;
    _delay_us(2);
    RCK_0;
}

unsigned char shift_buf[4];
unsigned char led_buf[16];

void format_scan(unsigned char *out, unsigned char *in, unsigned char index)
{
    //out column
    out[0] = ~in[index];
    //out row
    out[1] = (1<<index);
}

PT_THREAD(scan_thread(struct pt *pt))
{    
    static unsigned char i;
    static struct timer t;

    //25*8 = 200
    PT_BEGIN(pt);
    for (;;) {
        for ( i = 0; i < 8; i++ ) {
            timer_set(&t, 5);
            memset(shift_buf, 0, sizeof(shift_buf));
            shift(shift_buf, sizeof(shift_buf));
            _delay_us(10);

            format_scan(&shift_buf[0], &led_buf[0], i);
            format_scan(&shift_buf[2], &led_buf[8], i);
            shift(shift_buf, sizeof(shift_buf));

            PT_WAIT_UNTIL(pt, timer_expired(&t));            
        }
        PT_YIELD(pt);
    }    
    PT_END(pt);
}

PT_THREAD(main_thread(struct pt *pt))
{
    static unsigned long last_uptime;
    static struct timer t;

    PT_BEGIN(pt);
    
    PT_WAIT_WHILE(pt, is_timer2_busy());

    uint16_t tmp;

    tmp = eeprom_read_word(EEPROM_WORD_BOOT_COUNT);
    tmp++;
    eeprom_busy_wait();
    eeprom_write_word(EEPROM_WORD_BOOT_COUNT, tmp);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        uptime = eeprom_read_dword(EEPROM_LONG_UPTIME);
        if ( uptime == 0xFFFFFFFFul ) {
            uptime = 0;
        }
    }
    last_uptime = uptime_get();

    for (;;) {
        uint16_t raw;
        uint16_t volt;
        uint8_t percent;
        
        if ( last_uptime != uptime_get() ) {
            eeprom_busy_wait();
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
            {
                eeprom_write_dword(EEPROM_LONG_UPTIME, uptime);
            }
        }

        timer_set(&t, 1000);
        printf("main_thread() %u ", tick_time());
        print_ul(uptime_get());

        raw = adc_read(0);
        volt = map(raw, 0, 1024, 0, 5000);        

        //Method 1) read directed
        //32.0V --> 2.667V 0%
        //48.0V --> 4.000V
        //58.4V --> 4.867V 100%
        //60.0V --> 5.000V
        //percent = map(volt, 2667, 4867, 0, 100);

        //Method 2) read output opamp
        //32.0V --> 0.000V 0%
        //58.4V --> 4.8867V 100%
        percent = map(volt, 0, 4867, 0, 100);

        printf(", %04x", raw); 
        printf(" %u", volt);
        printf(" %u", percent);

        raw = adc_read(0xe); //V(bg) ~1.30V
        volt = map(raw, 0, 1024, 0, 5000);
        printf(", %04x", raw); 
        printf(" %u\n", volt);

        PT_WAIT_UNTIL(pt, timer_expired(&t));

        PT_YIELD(pt);
    }
    PT_END(pt);
}

int tick_time(void)
{
    int t;
  
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        t = tick1ms;
    }
    return t;
}

unsigned long uptime_get(void)
{
    unsigned long t;
  
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        t = uptime;
    }
    return t;    
}

int timer_expired(struct timer *t)
{ 
    return (int)(tick_time() - t->start) >= (int)t->interval; 
}

void timer_set(struct timer *t, int interval)
{
    t->interval = interval; t->start = tick_time(); 
}

unsigned short adc_read(unsigned mux)
{
   unsigned short raw;
   
   ADCSRA  =  0;  // Turn off the ADC
   
   //ADMUX   =  ((1 << REFS1) | (1 << REFS0)) + mux; // Internal 2.56V
   ADMUX   =  ((0 << REFS1) | (1 << REFS0)) + mux; // AVCC
   ADCSRA  =  (1 << ADEN)  | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

   ADCSRA |= (1 << ADSC); // Start Conversion

   while ( ADCSRA & (1 << ADSC) )
      ;

   while ( !(ADCSRA & (1 << ADIF)) )
      ;

   raw  =  ADCL;
   raw += (ADCH << 8);

   ADCSRA = 0;    // turn off the ADC
   raw &= 0x03FF; // 10 bits
   
   return raw;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main (void)
{
    struct pt main_pt, scan_pt;

    cli();
    timer1_init();
    timer2_init();
    uart_init();
    spi_init();
    sei();

    printf("TCNT1 %u\n", TCNT1);

    printf("EEPROM 0 %02x\n", eeprom_read_byte((const uint8_t *)0));
    printf("EEPROM 1 %02x\n", eeprom_read_byte((const uint8_t *)1));
    printf("EEPROM 2 %02x\n", eeprom_read_byte((const uint8_t *)2));
    printf("EEPROM 3 %02x\n", eeprom_read_byte((const uint8_t *)3));
    printf("EEPROM 4 %02x\n", eeprom_read_byte((const uint8_t *)4));
    printf("EEPROM 5 %02x\n", eeprom_read_byte((const uint8_t *)5));
    printf("EEPROM 6 %02x\n", eeprom_read_byte((const uint8_t *)6));
    printf("EEPROM 7 %02x\n", eeprom_read_byte((const uint8_t *)7));

    //OE
    PORTD &= ~(1<<PD4);
    DDRD |= (1<<PD4);

    //RCK
    PORTD &= ~(1<<PD5);
    DDRD |= (1<<PD5);

    led_buf[0] = hex_tab[0];
    led_buf[1] = hex_tab[1];
    led_buf[2] = hex_tab[2];
    
    led_buf[3] = 0xFF;
    led_buf[4] = 0x03;

    led_buf[8] = hex_tab[0];
    led_buf[9] = hex_tab[1];
    led_buf[10] = hex_tab[2];
    led_buf[11] = hex_tab[3];

    led_buf[12] = hex_tab[10];
    led_buf[13] = hex_tab[11];
    led_buf[14] = hex_tab[12];
    
    PT_INIT(&main_pt);
    PT_INIT(&scan_pt);
    for (;;) {
        //static unsigned int count = 0;
        //sleep_mode();

        //putchar('A');
        //spi_transmit('U');

        //shift_buf[0] = count;
        //shift(shift_buf, sizeof(shift_buf));
        //printf("%d\n", count++);

        //printf("%u\n", tick1ms);

        //_delay_ms(100);
        main_thread(&main_pt);
        scan_thread(&scan_pt);
    }

    return (0);
}
