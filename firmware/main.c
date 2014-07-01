#define F_CPU 16000000ul

#include <stdint.h>

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/cpufunc.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdbool.h>

#define FIFO_SIZE 128

#define NUM_FRAMES 3

// The framebuffer lines are stored in the order data is shifted into the 5940.
// i.e. most significant byte first blocks of blue. green, red
volatile uint8_t framebuffer[16 * 3 * 8 * NUM_FRAMES];

static uint8_t write_frame;
static volatile uint8_t *write_framebuffer;

static uint8_t display_frame;
static volatile uint8_t *display_framebuffer;

volatile uint8_t next_anode;

volatile uint8_t ticks;

#define FIFO_MASK (FIFO_SIZE - 1)
#if (FIFO_SIZE & FIFO_MASK) != 0
#error
#endif
volatile uint8_t fifo[FIFO_SIZE];

volatile uint8_t fifo_head;

#define SET_XLAT() PORTD |= _BV(5)
#define CLEAR_XLAT() PORTD &= ~_BV(5)

#define SET_BLANK() PORTD |= _BV(7)
#define CLEAR_BLANK() PORTD &= ~_BV(7)

#define SET_VPRG() PORTC |= _BV(2)
#define CLEAR_VPRG() PORTC &= ~_BV(2)

#define SET_DCPRG() PORTB |= _BV(0);
#define CLEAR_DCPRG() PORTB &= ~_BV(0);

// enable pullups on unused pins
static void
init_unused(void)
{
  // Not connected
  //PORTC |= _BV(1) | _BV(2) | _BV(4) | _BV(5);
  //PORTD |= _BV(0);
}

static void
init_i2c(void)
{
  // We do not currently use I2C.
  // However we do use them for ID so enable pullups
  PORTC |= _BV(4) | _BV(5);
}

static void
init_spi_slave(void)
{
  // Mode3, slave mode, MSB first, enable interrupt
  SPCR = _BV(SPIE) | _BV(SPE) | _BV(CPOL) | _BV(CPHA);

  // Enable pullups on SS, MOSI, MISO, SCK
  PORTB |= _BV(2) | _BV(3) | _BV(4) | _BV(5);
}

static void
init_spi_master(void)
{
  // Configure UART in SPI master mode, clk/2, mode0, MSB first
  UCSR0C = _BV(UMSEL01) | _BV(UMSEL00);
  UBRR0 = 0;
  UCSR0B = _BV(TXEN0);

  // Enable clock and data pins
  DDRD |= _BV(1) | _BV(4);
  // Enable pullup on input
  PORTD |= _BV(0);
}

// 2kHz refresh timer
static void
init_timer(void)
{
  // CTC mode
  TCNT1 = 0;
  OCR1A = 8000; // 8000 cycles = 500us = 2Khz
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
  // Enable OCA interrupt
  TIMSK1 = _BV(OCIE1A);
}

static void
init_gsclk(void)
{
  // CTC mode, toggle OC0A
  TCCR0A = _BV(COM0A0) | _BV(WGM01);
  TCCR0B = 0;
  TCNT0 = 0;
  // 570kHz output clock.  Combined with a 2kHz row mux this gives a maximum
  // PWM value of 0xff, plus a bit of extra delay for latching in new data.
  OCR0A = 14;
  // Enable OC0A output
  DDRD |= _BV(6);
}

static inline void
enable_gsclk(void)
{
  TCCR0B = _BV(CS00);
}

static inline void
disable_gsclk(void)
{
  TCCR0B = 0;
  TCNT0 = 0;
  // Make sure output line is low
  _NOP();
  if (PINB & _BV(6)) {
      TCCR0B |= _BV(FOC0A);
  }
}

static uint8_t eeprom_address EEMEM;
static uint8_t eeprom_dc_r EEMEM;
static uint8_t eeprom_dc_g EEMEM;
static uint8_t eeprom_dc_b EEMEM;

static uint8_t my_address;

static inline void
set_pixel(uint16_t n, uint8_t val)
{
  write_framebuffer[n] = val;
}

static uint8_t __attribute__((unused))
get_pixel(uint16_t n)
{
  return write_framebuffer[n];
}

static volatile uint8_t fb_offset;
static volatile bool sending_frame;
static volatile uint8_t dc_bytes;
static volatile bool dc_changed;
static volatile uint8_t dc_red;
static volatile uint8_t dc_green;
static volatile uint8_t dc_blue;

static void
send_data(void)
{
  uint8_t tmp;

  if ((UCSR0A & _BV(UDRE0)) == 0)
    return;

  //  All data is send MSB first
  if (dc_bytes) {
      // Dot correction data
      if (dc_bytes > 6)
	tmp = dc_blue;
      else if (dc_bytes > 3)
	tmp = dc_green;
      else
	tmp = dc_red;
      dc_bytes--;
      UDR0 = (tmp << 2) | tmp >> 4;
      while ((UCSR0A & _BV(UDRE0)) == 0)
	/* no-op */;
      UDR0 = (tmp << 4) | tmp >> 2;
      while ((UCSR0A & _BV(UDRE0)) == 0)
	/* no-op */;
      UDR0 = (tmp << 6) | tmp;
      if (dc_bytes == 0) {
	  // Wait for the transmit to complete
	  _delay_us(5);
	  // Latch data into register
	  SET_XLAT();
	  CLEAR_XLAT();
	  SET_DCPRG();
	  CLEAR_VPRG();
      }
  } else if (sending_frame) {
      // PWM data
      // The count registers are 12 bits, so a pair of values expand to 
      // 3 bytes of data.  We can not submit all this immediately, but the USART
      // double buffering means we probably only stall for 16 cycles and it is
      // not worth trying to be clever.
      tmp = display_framebuffer[fb_offset++];
      UDR0 = tmp >> 4;
      tmp <<= 4;
      while ((UCSR0A & _BV(UDRE0)) == 0)
	/* no-op */;
      UDR0 = tmp;
      tmp = display_framebuffer[fb_offset++];
      while ((UCSR0A & _BV(UDRE0)) == 0)
	/* no-op */;
      UDR0 = tmp;
      if (fb_offset == 16 * 3) {
	  _delay_us(5);
	  sending_frame = false;
      }
  }
}

static void
set_address(uint8_t address)
{
  if (address == 0xff)
    return;
  my_address = address;
  eeprom_update_byte(&eeprom_address, address);
}

static void
set_dc(uint8_t red, uint8_t green, uint8_t blue)
{
  eeprom_update_byte(&eeprom_dc_r, red);
  eeprom_update_byte(&eeprom_dc_g, green);
  eeprom_update_byte(&eeprom_dc_b, blue);
  /* Dot correction data is only 6 bits.  */
  dc_red = red >> 2;
  dc_green = green >> 2;
  dc_blue = blue >> 2;
  dc_changed = true;
}

enum
{
  SM_IDLE,
  SM_READY,
  SM_ACTIVE,
};

static void
do_data(void)
{
  uint8_t n;
  uint8_t cmd;
  uint8_t d0;
  uint8_t d1;
  uint8_t d2;
  uint8_t fifo_tail;
  uint8_t sm;

  fifo_tail = 0;
  while (true) {
      send_data();
      n = (fifo_head - fifo_tail) & FIFO_MASK;
      if (n < 4) {
	  continue;
      }
      cmd = fifo[fifo_tail];
      fifo_tail = (fifo_tail + 1) & FIFO_MASK;
      if (cmd >= 0xf0) {
	  sm = SM_IDLE;
	  continue;
      }
      if (cmd < 0xd0 && sm != SM_ACTIVE) {
	  fifo_tail = (fifo_tail + 3) & FIFO_MASK;
	  continue;
      }
      d0 = fifo[fifo_tail];
      fifo_tail = (fifo_tail + 1) & FIFO_MASK;
      d1 = fifo[fifo_tail];
      fifo_tail = (fifo_tail + 1) & FIFO_MASK;
      d2 = fifo[fifo_tail];
      fifo_tail = (fifo_tail + 1) & FIFO_MASK;

      if (cmd == 0xe0) {
	  if (d0 == 0xf0 && d1 == 0xf1 && d2 == 0xf2)
	    sm = SM_READY;
	  else
	    sm = SM_IDLE;
	  continue;
      }
      if (sm == SM_IDLE)
	continue;
      if (cmd < 16 * 8) {
	  // Map from packed RGB to the planar MSB framebuffer
	  uint16_t pos = ((uint16_t)(cmd & 0xf0)) * 3;
	  pos += 0xf - (cmd & 0xf);
	  set_pixel(pos + 32, d0);
	  set_pixel(pos + 16, d1);
	  set_pixel(pos, d2);
      } else if (cmd == 0x80u) {
	  /* Set framebuffer (page flip).  */
	  display_frame = d1 % NUM_FRAMES;
	  write_frame = d2 % NUM_FRAMES;
	  write_framebuffer = &framebuffer[(uint16_t)write_frame * 16 * 8 * 3];
      } else if (cmd == 0xc0) {
	  /* Set brightness */
	  set_dc(d0, d1, d2);
      } else if (cmd == 0xe1) {
	  if (d0 == my_address || d0 == 0xff)
	    sm = SM_ACTIVE;
	  else
	    sm = SM_READY;
      } else {
	  /* Unknown command.  */
      }
  }
}

#ifdef __AVR_ATmega328P__X
ISR(SPI_STC_vect, ISR_NAKED)
{
  asm (
"\n push r28"
"\n push r29"
"\n push r30"
"\n push r31"
"\n in r28, __SREG__"
"\n lds r30, fifo_head"
"\n1:"
"\n ldi r31, 0"
"\n in r29, 0x2e" // SPDR
"\n subi r30,lo8(-(fifo))"
"\n sbci r31,hi8(-(fifo))"
"\n st Z+,r29"
"\n subi r30,lo8(fifo)"
"\n andi r30, lo8(63)" // FIFO_MASK
"\n in r29, 0x2d" // SPSR
"\n sbrc r29, 7" // SPIF
"\n rjmp 1b"
"\n sts fifo_head, r29"
"\n out __SREG__, r28"
"\n pop r31"
"\n pop r30"
"\n pop r29"
"\n pop r28"
"\n reti" // Total 44 cycles
      );
}
#else
ISR(SPI_STC_vect)
{
  fifo[fifo_head] = SPDR;
  fifo_head = (fifo_head + 1) & FIFO_MASK;
}
#endif

ISR(TIMER1_COMPA_vect)
{
  static uint8_t overload;
  ticks++;
  // Enable interrupts so we are not blocking the SPI slave interrupt.
  // The clock period is long enough that we don't have to worry about
  // nested timer interrupts
  sei();
  if (sending_frame)
    overload = 0xff;

  disable_gsclk();
  SET_BLANK();
  if (overload) {
      overload--;
      return;
  }
  if (next_anode != 0xff && !overload) {
      // Latch data into the register
      SET_XLAT();
      CLEAR_XLAT();
      PORTC |= _BV(0);
      PORTB |= _BV(1);
      // Select the next anode
      if (next_anode == 0) {
	  PORTC &= ~_BV(0);
      } else if (next_anode == 1) {
	  PORTB &= ~_BV(1);
      }
      // Delay a few us for everything to settle.
      _delay_us(5);
  }
  next_anode = (next_anode + 1) & 7;
  // Shift in the next set of anode data
  display_framebuffer = &framebuffer[(((uint16_t)display_frame * 8) + next_anode) * 16 * 3];
  fb_offset = 0;
  sending_frame = true;
  // And dot correction data if needed
  if (dc_changed) {
      dc_bytes = 12;
      SET_VPRG();
      dc_changed = false;
  }
  // Triggering the greyscale clock is timing critical, so disable interrupts
  cli();
  // Trigger the output pulse
  CLEAR_BLANK();
  enable_gsclk();
}

static void
init_5940(void)
{
  init_gsclk();
  init_spi_slave();
  // Setup XLAT, BLANK
  DDRD |= _BV(5) | _BV(7);
  SET_BLANK();
  CLEAR_XLAT();
  // Setup VPRG
  DDRC |= _BV(2);
  CLEAR_VPRG();
  // Setup DCPRG
  DDRB |= _BV(0);
  CLEAR_DCPRG();
}

static void
init_eeprom()
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t address;

  // Set pullups on the ID pins.
  PORTC |= _BV(4) | _BV(5);

  address = eeprom_read_byte(&eeprom_address);
  if (address == 0xff) {
      address = 0;
      r = 0x40;
      g = 0x80;
      b = 0x60;
  } else {
      r = eeprom_read_byte(&eeprom_dc_r);
      g = eeprom_read_byte(&eeprom_dc_g);
      b = eeprom_read_byte(&eeprom_dc_b);
  }
  // Override address based on ID pins
  address = 0;
  if ((PINC & _BV(4)) == 0)
    address |= 1;
  if ((PINC & _BV(5)) == 0)
    address |= 2;
  set_address(address);
  set_dc(r, g, b);
}

static void
init_anodes(void)
{
  /* Anode select pins.  */
  DDRC |= _BV(0);
  DDRB |= _BV(1);
  PORTC |= _BV(0);
  PORTB |= _BV(1);
}

int
main()
{
  /* Do this early to minimize amount of time pins are left floating.  */
  SET_BLANK();
  init_anodes();

  fifo_head = 0;
  next_anode = 0xff;
  sending_frame = false;
  write_framebuffer = &framebuffer[0];
  display_framebuffer = &framebuffer[0];

  init_eeprom();
  init_unused();
  init_i2c();
  init_5940();
  init_spi_master();
  init_timer();

  sei();
  do_data();
  return 0;
}
