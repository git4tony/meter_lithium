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

#define DBG_BIT_DEF     PD6
#define DBG_DDR_DEF     DDRD
#define DBG_PORT_DEF    PORTD

#define DBG_BIT_OUT     DBG_DDR_DEF |= (1<<DBG_BIT_DEF)
#define DBG_BIT_0       DBG_PORT_DEF &= ~(1<<DBG_BIT_DEF)
#define DBG_BIT_1       DBG_PORT_DEF |= (1<<DBG_BIT_DEF)
#define DBG_BIT_TOGGLE  DBG_PORT_DEF ^= (1<<DBG_BIT_DEF)

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
int tick_get(void);

unsigned long uptime_get(void);
//unsigned short adc_read(unsigned char mux);
void adc_start(unsigned char mux);
unsigned char adc_is_started(void);
unsigned char adc_is_ready(void);
unsigned short adc_value(void);

long map(long x, long in_min, long in_max, long out_min, long out_max);

unsigned char hex_tab[16] = {
   0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f,
   CHAR_A, CHAR_b, CHAR_C, CHAR_d, CHAR_E, CHAR_F
};

volatile unsigned short tick1ms = 0;
volatile unsigned long uptime = 0;

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
}

unsigned char timer2_is_busy(void)
{
    return (ASSR & ((1<<TCN2UB) | (1<<TCR2UB)));
}

//Overflow ISR
ISR(TIMER2_OVF_vect)
{
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

static void shift(unsigned char *p, unsigned char n)
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

static void format(unsigned char *out, unsigned char *in, unsigned char index)
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

            format(&shift_buf[0], &led_buf[0], i);
            format(&shift_buf[2], &led_buf[8], i);
            shift(shift_buf, sizeof(shift_buf));

            PT_YIELD_UNTIL(pt, timer_expired(&t));            
        }        
    }

    PT_END(pt);
}

PT_THREAD(eeprom_write_thread(struct pt *pt, void *addr, void *buf, uint8_t size))
{
    static uint8_t *a;
    static uint8_t *p;
    static uint8_t n;

    PT_BEGIN(pt);
    
    a = (uint8_t *)addr;
    p = (uint8_t *)buf;
    for ( n = 0; n < size; n++ ) {
        eeprom_write_byte(a++, *p++);    
        PT_YIELD_UNTIL(pt, eeprom_is_ready());
    }

    PT_END(pt);
}

#define PT_EEPROM_WRITE(a, b, c)    PT_SPAWN(pt, &child, eeprom_write_thread(&child, (void *)(a), b, c))
#define PT_EEPROM_WRITE_BYTE(a, b)  PT_EEPROM_WRITE(a, &b, 1)
#define PT_EEPROM_WRITE_WORD(a, b)  PT_EEPROM_WRITE(a, &b, 2)
#define PT_EEPROM_WRITE_DWORD(a, b) PT_EEPROM_WRITE(a, &b, 4)

PT_THREAD(adc_read_thread(struct pt *pt, uint8_t mux, uint16_t *raw))
{
    PT_BEGIN(pt);

    adc_start(mux);
    PT_YIELD_UNTIL(pt, adc_is_started());
    PT_YIELD_UNTIL(pt, adc_is_ready());
    *raw = adc_value();

    PT_END(pt);
}

#define PT_ADC_READ(a, b)   PT_SPAWN(pt, &child, adc_read_thread(&child, a, b))

PT_THREAD(main_thread(struct pt *pt))
{
    //static struct timer t;
    static struct pt child;
    static unsigned long tmp_uptime;
    static uint16_t tmp;

    PT_BEGIN(pt);
    
    PT_YIELD_UNTIL(pt, !timer2_is_busy());

    tmp = eeprom_read_word(EEPROM_WORD_BOOT_COUNT);
    tmp++;
    DBG_BIT_1;
    PT_EEPROM_WRITE_WORD(EEPROM_WORD_BOOT_COUNT, tmp);
    DBG_BIT_0;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        uptime = eeprom_read_dword(EEPROM_LONG_UPTIME);
        if ( uptime == 0xFFFFFFFFul ) {
            uptime = 0;
        }
    }

    tmp_uptime = uptime_get();
    for (;;) {
        static uint16_t raw;
        static uint16_t volt;
        static uint8_t percent;
        static uint16_t xvolt;
        
        printf("main_thread() %u ", tick_get());
        print_ul(uptime_get());

#if 1
        DBG_BIT_1;
        PT_ADC_READ(0, &raw);
        DBG_BIT_0;

        volt = map(raw, 0, 1024, 0, 5000);        
        xvolt = map(volt, 0, 5000, 0, 600);

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
        printf(" %u%%", percent);        
        printf(" %u", xvolt);

        DBG_BIT_1;
        PT_ADC_READ(0xe, &raw); //V(bg) ~1.30V
        DBG_BIT_0;

        volt = map(raw, 0, 1024, 0, 5000);
        printf(", %04x", raw); 
        printf(" %u", volt);
#endif
        printf("\n");

        PT_WAIT_UNTIL(pt, tmp_uptime != uptime_get());
        tmp_uptime = uptime_get();
        
        DBG_BIT_1;
        PT_EEPROM_WRITE_DWORD(EEPROM_LONG_UPTIME, tmp_uptime);
        DBG_BIT_0;
    }
    PT_END(pt);
}

int tick_get(void)
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
    return (int)(tick_get() - t->start) >= (int)t->interval; 
}

void timer_set(struct timer *t, int interval)
{
    t->interval = interval; t->start = tick_get(); 
}

#if 0
unsigned short adc_read(unsigned char mux)
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
#endif

void adc_start(unsigned char mux)
{
   ADCSRA  =  0;  // Turn off the ADC
   
   //ADMUX   =  ((1 << REFS1) | (1 << REFS0)) + mux; // Internal 2.56V
   ADMUX   =  ((0 << REFS1) | (1 << REFS0)) + mux; // AVCC
   ADCSRA  =  (1 << ADEN)  | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
   ADCSRA |= (1 << ADSC); // Start Conversion    
}

unsigned char adc_is_started(void)
{
    return !(ADCSRA & (1 << ADSC));
}

unsigned char adc_is_ready(void)
{
    return (ADCSRA & (1 << ADIF));
}

unsigned short adc_value(void)
{
   unsigned short raw;
    
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
    
    DBG_BIT_0;
    DBG_BIT_OUT;

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
