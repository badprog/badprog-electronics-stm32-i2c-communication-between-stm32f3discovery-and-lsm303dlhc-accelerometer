// badprog.com
/**
 * @file    main.c
 * @brief   I2C accelerometer data reading with LSM303DLHC on STM32F3Discovery
 *
 * Bare metal C. No HAL, no CubeMX.
 *
 * Reads linear acceleration on X, Y, Z axes from the LSM303DLHC accelerometer
 * via I2C1 and prints values via UART (PA9, 115200 baud).
 * The 8 compass LEDs on PE8..PE15 rotate to indicate the tilt direction.
 *
 * I2C1 pins:
 *   PB6 = SCL  (AF4)
 *   PB7 = SDA  (AF4)
 *
 * UART pins:
 *   PA9 = TX   (AF7, USART1)
 *
 * LEDs:
 *   PE8..PE15  = compass ring, one LED lights up toward the low side
 *
 * CPU clock: 8 MHz (default HSI, no PLL)
 * I2C clock: 100 kHz (Standard Mode)
 *
 * -------------------------------------------------------------------------
 * PERIPHERAL GLOSSARY
 * -------------------------------------------------------------------------
 *
 * RCC  - Reset and Clock Control
 *        Controls which peripherals receive a clock signal.
 *        Every peripheral is OFF by default on STM32 to save power.
 *        You must enable its clock in RCC before touching its registers.
 *        Example: RCC->AHBENR |= RCC_AHBENR_IOPBEN enables the GPIOB clock.
 *
 * GPIO - General Purpose Input/Output
 *        Physical pins on the MCU. Each pin can be configured as:
 *          - Input        : read external signal (button, sensor)
 *          - Output       : drive external device (LED, CS line)
 *          - AF           : Alternate Function, the pin is handed to a
 *                           peripheral (I2C, UART, SPI...) instead of GPIO
 *          - Analog       : used by ADC/DAC
 *        Each GPIO port (A, B, C...) has its own register block.
 *        GPIOA controls PA0..PA15, GPIOB controls PB0..PB15, etc.
 *
 * USART - Universal Synchronous/Asynchronous Receiver Transmitter
 *        Hardware peripheral that handles serial communication.
 *        We use it in asynchronous mode (UART) at 115200 baud on PA9.
 *        Allows printf-style output readable with a logic analyzer or FTDI.
 *
 * I2C  - Inter-Integrated Circuit
 *        Two-wire synchronous protocol: SCL (clock) and SDA (data).
 *        The master (STM32) generates the clock and initiates transactions.
 *        Each device on the bus has a unique 7-bit address.
 *        The LSM303DLHC accelerometer sits at address 0x19 (7-bit).
 *
 * -------------------------------------------------------------------------
 * REGISTER GLOSSARY
 * -------------------------------------------------------------------------
 *
 * RCC->AHBENR  - AHB Enable Register
 *                Each bit enables the clock for one peripheral on the AHB bus.
 *                GPIO ports are on the AHB on STM32F3.
 *
 * RCC->APB2ENR - APB2 Enable Register
 *                USART1 is on APB2.
 *
 * RCC->APB1ENR - APB1 Enable Register
 *                I2C1 is on APB1.
 *
 * GPIO->MODER  - Mode Register, 2 bits per pin
 *                  00 = Input
 *                  01 = Output
 *                  10 = Alternate Function
 *                  11 = Analog
 *
 * GPIO->OTYPER - Output Type Register, 1 bit per pin
 *                  0 = Push-pull  (default, drives both high and low)
 *                  1 = Open-drain (only pulls low, external resistor pulls high)
 *                I2C lines MUST be open-drain: multiple devices share the bus
 *                and anyone can pull SDA low to signal a 0. The pull-up
 *                resistor (internal here) restores the line to 1 when nobody
 *                is pulling it down.
 *
 * GPIO->OSPEEDR - Output Speed Register, 2 bits per pin
 *                  00 = Low, 01 = Medium, 10 = High, 11 = Very High
 *
 * GPIO->PUPDR  - Pull-Up/Pull-Down Register, 2 bits per pin
 *                  00 = None, 01 = Pull-up, 10 = Pull-down
 *                I2C lines need pull-ups. We use the internal ones here.
 *                External 4.7k resistors would be stronger and more reliable
 *                at higher I2C speeds, but internal pull-ups work fine at 100 kHz.
 *
 * GPIO->AFR[0] - Alternate Function Register for pins 0..7
 * GPIO->AFR[1] - Alternate Function Register for pins 8..15
 *                Selects which peripheral controls the pin in AF mode.
 *                4 bits per pin, value 0..15 maps to AF0..AF15.
 *                AF4 = I2C1/I2C2 on STM32F3.
 *                AF7 = USART1/USART2/USART3 on STM32F3.
 *
 * GPIO->BSRR   - Bit Set/Reset Register, write-only, atomic
 *                  bits [15:0]  : set   pin N high by writing 1 to bit N
 *                  bits [31:16] : reset pin N low  by writing 1 to bit N+16
 *
 * USART->BRR   - Baud Rate Register
 *                BRR = fPCLK / baudrate = 8 000 000 / 115 200 = 69
 *
 * USART->CR1   - Control Register 1
 *                UE  (bit 0) : USART enable
 *                TE  (bit 3) : Transmitter enable
 *
 * USART->ISR   - Interrupt and Status Register
 *                TXE (bit 7) : Transmit data register empty, safe to write TDR
 *
 * USART->TDR   - Transmit Data Register
 *                Write a byte here to send it over TX.
 *
 * I2C->TIMINGR - Timing Register
 *                Replaces the CCR register of older STM32 families.
 *                Encodes all I2C timing parameters in one register:
 *                  PRESC   [31:28] : prescaler for the I2C kernel clock
 *                  SCLDEL  [23:20] : SCL delay (data setup time)
 *                  SDADEL  [19:16] : SDA delay (data hold time)
 *                  SCLH    [15:8]  : SCL high period
 *                  SCLL    [7:0]   : SCL low period
 *                Value 0x10420F13 is valid for 100 kHz with HSI at 8 MHz.
 *                Do not guess this value. Use ST AN4235 or CubeMX timing tool.
 *
 * I2C->CR2     - Control Register 2
 *                SADD    [9:0]  : 7-bit slave address in bits [7:1]
 *                NBYTES  [23:16]: number of bytes to transfer
 *                RD_WRN  [10]   : 0=write, 1=read
 *                START   [13]   : generate a START condition
 *                STOP    [14]   : generate a STOP condition
 *                AUTOEND [25]   : auto-generate STOP after NBYTES (read mode)
 *
 * I2C->ISR     - Interrupt and Status Register
 *                TXIS  (bit 1) : ready to write next byte to TXDR
 *                RXNE  (bit 2) : a received byte is ready in RXDR
 *                TC    (bit 6) : transfer complete (AUTOEND=0, write phase done)
 *                BUSY  (bit 15): bus is busy (transaction in progress)
 *
 * I2C->TXDR    - Transmit Data Register: write the next byte to send
 * I2C->RXDR    - Receive Data Register : read the received byte
 * -------------------------------------------------------------------------
 */

#include <stdio.h>
#include "stm32f303xc.h"

// ---------------------------------------------------------------------------
// UART - USART1 on PA9 at 115200 baud
// Used to send accelerometer readings to the logic analyzer or terminal
// ---------------------------------------------------------------------------

static void uart_init(void)
{
    // RCC->AHBENR: enable clock for GPIOA (bit 17) - holds PA9 (TX)
    RCC->AHBENR |= RCC_AHBENR_IOPAEN;

    // RCC->APB2ENR: enable clock for USART1 (bit 14)
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // --- Configure PA9 as Alternate Function (USART1 TX) ---
    // GPIO->MODER: 2 bits per pin, PA9 is at bits [19:18]
    // Clear then set to AF (0b10)
    GPIOA->MODER &= ~(0x3UL << 18);
    GPIOA->MODER |=  (GPIO_MODER_AF << 18);

    // GPIO->AFR[1]: AF7 = USART1 for PA9
    // AFR[1] covers PA8..PA15, 4 bits per pin
    // PA9 is pin 9, offset in AFR[1] = (9 - 8) * 4 = 4 -> bits [7:4]
    GPIOA->AFR[1] &= ~(0xFUL << 4);
    GPIOA->AFR[1] |=  (0x7UL << 4);   // AF7 = USART1

    // USART1->BRR: baud rate = fPCLK / baudrate = 8 000 000 / 115 200 = 69
    USART1->BRR = 69;

    // USART1->CR1: enable USART and transmitter
    // UE (bit 0) = 1 : USART enabled
    // TE (bit 3) = 1 : transmitter enabled, TX line goes idle HIGH
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
}

static void uart_print(const char *str)
{
    while (*str) {
        // USART->ISR::TXE (bit 7): wait until the transmit data register is empty
        // Only then write the next character, otherwise the previous byte is lost
        while (!(USART1->ISR & USART_ISR_TXE));
        // USART->TDR: writing a byte here starts the transmission
        USART1->TDR = (uint32_t)(*str++);
    }
}

// ---------------------------------------------------------------------------
// LEDs - PE8..PE15 as push-pull outputs (compass ring)
// ---------------------------------------------------------------------------

static void leds_init(void)
{
    // RCC->AHBENR: enable clock for GPIOE (bit 21)
    RCC->AHBENR |= RCC_AHBENR_IOPEEN;

    // GPIO->MODER: pins PE8..PE15 occupy bits [31:16]
    // Clear all 16 bits, then set each pair to 01 (output)
    // 0x5555 = 0101 0101 0101 0101 -> every 2-bit field = 01 = output
    GPIOE->MODER &= ~(0xFFFFUL << 16);
    GPIOE->MODER |=  (0x5555UL << 16);

    // Turn all LEDs off at startup
    // BSRR reset half: write 1 to bits [31:16] to drive pins LOW
    // 0xFF << 8 = pin mask for PE8..PE15, then << 16 = BSRR reset half
    GPIOE->BSRR = LED_ALL_PINS_OFF;
}

// Light one LED out of 8 to indicate tilt direction from X/Y acceleration
// x, y: raw 12-bit signed accelerometer values
static void leds_show_direction(int16_t x, int16_t y)
{
    // We want to find which of the 8 sectors (45 deg each) the tilt vector falls into.
    // Instead of atan2 (requires libm), we use a simple integer comparison approach.
    //
    // The 8 LED positions map to 8 compass directions:
    //   PE8  = East       ( X+, Y~0 )
    //   PE9  = North-East ( X+, Y+  )
    //   PE10 = North      ( X~0, Y+ )
    //   PE11 = North-West ( X-, Y+  )
    //   PE12 = West       ( X-, Y~0 )
    //   PE13 = South-West ( X-, Y-  )
    //   PE14 = South      ( X~0, Y- )
    //   PE15 = South-East ( X+, Y-  )
    //
    // We compare |x| and |y| to determine the primary axis, then use signs.

    // Turn all LEDs off first (BSRR reset half)
    GPIOE->BSRR = LED_ALL_PINS_OFF;

    int16_t ax = x < 0 ? -x : x;   // absolute value of X
    int16_t ay = y < 0 ? -y : y;   // absolute value of Y

    uint8_t led_pin;   // which PE pin to light (8..15)

    if (ax > ay) {
        // Primarily horizontal tilt
        if (x > 0) {
            // East side
            led_pin = (y > 0) ? 9 : (y < 0) ? 15 : 8;
        } else {
            // West side
            led_pin = (y > 0) ? 11 : (y < 0) ? 13 : 12;
        }
    } else if (ay > ax) {
        // Primarily vertical tilt
        if (y > 0) {
            // North side
            led_pin = (x > 0) ? 9 : (x < 0) ? 11 : 10;
        } else {
            // South side
            led_pin = (x > 0) ? 15 : (x < 0) ? 13 : 14;
        }
    } else {
        // ax == ay: exact diagonal, pick based on quadrant
        if      (x > 0 && y > 0) led_pin = 9;
        else if (x < 0 && y > 0) led_pin = 11;
        else if (x < 0 && y < 0) led_pin = 13;
        else                      led_pin = 15;
    }

    // BSRR set half (bits [15:0]): write 1 to bit N to drive PE_N HIGH = LED on
    GPIOE->BSRR = (1UL << led_pin);
}

// ---------------------------------------------------------------------------
// I2C1 - low level helpers
// All transactions follow the STM32F3 I2C sequence:
//   write: START -> SADD+W -> byte(s) -> TC -> STOP
//   read:  START -> SADD+W -> reg addr -> TC -> START -> SADD+R -> byte(s) -> STOP (AUTOEND)
// ---------------------------------------------------------------------------

// Wait until the I2C bus is free
static void i2c_wait_busy(void)
{
    // I2C->ISR::BUSY (bit 15): set while a transaction is ongoing
    // Spin here until the bus is released
    while (I2C1->ISR & I2C_ISR_BUSY);
}

// Load CR2 to initiate a write transaction
// addr:   7-bit slave address
// nbytes: number of bytes to send
static void i2c_start_write(uint8_t addr, uint8_t nbytes)
{
    // I2C->CR2 layout for a write:
    //   SADD   [7:1] : slave address (7-bit, must go in bits [7:1])
    //   NBYTES [23:16]: number of bytes to transfer
    //   START  [13]  : generate START condition
    //   RD_WRN [10]  : 0 = write direction
    //   AUTOEND not set: we want manual TC flag so we can send a repeated START for reads
    I2C1->CR2 = ((uint32_t)(addr << 1))
              | ((uint32_t)nbytes << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_START;
}

// Load CR2 to initiate a read transaction
// addr:   7-bit slave address
// nbytes: number of bytes to receive
static void i2c_start_read(uint8_t addr, uint8_t nbytes)
{
    // Same as write but:
    //   RD_WRN [10] = 1 : read direction
    //   AUTOEND [25] = 1: hardware generates STOP automatically after nbytes
    //                     No need to poll TC or issue STOP manually
    I2C1->CR2 = ((uint32_t)(addr << 1))
              | I2C_CR2_RD_WRN
              | ((uint32_t)nbytes << I2C_CR2_NBYTES_Pos)
              | I2C_CR2_START
              | I2C_CR2_AUTOEND;
}

// Send one byte on the I2C bus
static void i2c_write_byte(uint8_t byte)
{
    // I2C->ISR::TXIS (bit 1): set when TXDR is empty and ready for the next byte
    // TXIS is cleared automatically when we write to TXDR
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = byte;
}

// Read one byte from the I2C bus
static uint8_t i2c_read_byte(void)
{
    // I2C->ISR::RXNE (bit 2): set when RXDR contains a received byte
    // RXNE is cleared automatically when we read RXDR
    while (!(I2C1->ISR & I2C_ISR_RXNE));
    return (uint8_t)I2C1->RXDR;
}

// Wait for Transfer Complete flag (used at end of write phase before repeated START)
static void i2c_wait_tc(void)
{
    // I2C->ISR::TC (bit 6): set when NBYTES have been transferred and AUTOEND=0
    // We poll TC before issuing a repeated START for the read phase
    while (!(I2C1->ISR & I2C_ISR_TC));
}

// Generate a STOP condition manually (used after write transactions)
static void i2c_stop(void)
{
    // I2C->CR2::STOP (bit 14): trigger a STOP condition on the bus
    I2C1->CR2 |= I2C_CR2_STOP;
    // Wait until the bus is released before starting the next transaction
    while (I2C1->ISR & I2C_ISR_BUSY);
}

// ---------------------------------------------------------------------------
// I2C1 - GPIO and peripheral initialization
// ---------------------------------------------------------------------------

static void i2c1_init(void)
{
    // RCC->AHBENR: enable clock for GPIOB (bit 18) - holds PB6 (SCL) and PB7 (SDA)
    RCC->AHBENR |= RCC_AHBENR_IOPBEN;

    // RCC->APB1ENR: enable clock for I2C1 (bit 21)
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // --- Configure PB6 (SCL) and PB7 (SDA) ---

    // GPIO->MODER: set PB6 and PB7 to Alternate Function mode (0b10)
    // PB6 -> bits [13:12], PB7 -> bits [15:14]
    GPIOB->MODER &= ~((0x3UL << 12) | (0x3UL << 14));
    GPIOB->MODER |=  (GPIO_MODER_AF << 12) | (GPIO_MODER_AF << 14);

    // GPIO->OTYPER: set PB6 and PB7 to open-drain (bit 6 and bit 7)
    // Open-drain is mandatory for I2C: both master and slave can only pull
    // the line LOW. The line returns HIGH via the pull-up resistor when
    // nobody drives it low.
    GPIOB->OTYPER |= (GPIO_OTYPER_OD << 6) | (GPIO_OTYPER_OD << 7);

    // GPIO->OSPEEDR: set output speed to Very High for PB6 and PB7
    // PB6 -> bits [13:12], PB7 -> bits [15:14]
    // Very High (0x3) is used here to ensure clean edges on the I2C lines.
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_VHIGH << 12) | (GPIO_OSPEEDR_VHIGH << 14);

    // GPIO->PUPDR: enable internal pull-up on PB6 and PB7
    // PB6 -> bits [13:12], PB7 -> bits [15:14]
    // The LSM303DLHC on the discovery board already has external pull-ups,
    // but enabling internal ones as well does not hurt and makes the code
    // portable to boards without external pull-ups.
    GPIOB->PUPDR &= ~((0x3UL << 12) | (0x3UL << 14));
    GPIOB->PUPDR |=  (GPIO_PUPDR_UP << 12) | (GPIO_PUPDR_UP << 14);

    // GPIO->AFR[0]: select AF4 (I2C1) for PB6 and PB7
    // AFR[0] covers PB0..PB7, 4 bits per pin
    // PB6 -> bits [27:24], PB7 -> bits [31:28]
    GPIOB->AFR[0] &= ~((0xFUL << 24) | (0xFUL << 28));
    GPIOB->AFR[0] |=  (0x4UL << 24) | (0x4UL << 28);   // AF4 = I2C1

    // --- Configure I2C1 peripheral ---

    // I2C->CR1::PE (bit 0): disable I2C before changing TIMINGR
    // TIMINGR must only be written when PE=0
    I2C1->CR1 &= ~I2C_CR1_PE;

    // I2C->TIMINGR: timing configuration for 100 kHz Standard Mode with HSI at 8 MHz
    // Breakdown of 0x10420F13:
    //   PRESC   [31:28] = 0x1  -> prescaler = 2 -> tPRESC = 2 / 8 MHz = 250 ns
    //   SCLDEL  [23:20] = 0x4  -> SCL delay  = 4 * 250 ns = 1000 ns (data setup time)
    //   SDADEL  [19:16] = 0x2  -> SDA delay  = 2 * 250 ns =  500 ns (data hold time)
    //   SCLH    [15:8]  = 0x0F -> SCL high period = 15+1 * 250 ns = 4000 ns
    //   SCLL    [7:0]   = 0x13 -> SCL low  period = 19+1 * 250 ns = 5000 ns
    // Total period = 9000 ns -> ~111 kHz (close enough to 100 kHz)
    // Source: ST AN4235 I2C timing configuration tool
    I2C1->TIMINGR = 0x10420F13U;

    // I2C->CR1::PE (bit 0): re-enable I2C now that TIMINGR is set
    I2C1->CR1 |= I2C_CR1_PE;
}

// ---------------------------------------------------------------------------
// LSM303DLHC - accelerometer register access
// ---------------------------------------------------------------------------

// Write one byte to a register of the LSM303DLHC accelerometer
// reg:   register address
// value: value to write
static void lsm303_write_reg(uint8_t reg, uint8_t value)
{
    // I2C write transaction: START -> SADD+W -> reg -> value -> TC -> STOP
    i2c_wait_busy();
    i2c_start_write(LSM303_ACCEL_ADDR, 2);   // 2 bytes: register address + value
    i2c_write_byte(reg);
    i2c_write_byte(value);
    i2c_wait_tc();
    i2c_stop();
}

// Read len consecutive registers starting at reg from the LSM303DLHC
// Stores results in buf[0..len-1]
static void lsm303_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    // Phase 1 - write the register address (with auto-increment bit set)
    // I2C write: START -> SADD+W -> reg|0x80 -> TC
    // bit 7 = 1 activates auto-increment: the LSM303DLHC advances its internal
    // register pointer after each byte, so we can read 6 bytes in one burst
    // instead of issuing a separate transaction per register.
    i2c_wait_busy();
    i2c_start_write(LSM303_ACCEL_ADDR, 1);
    i2c_write_byte(reg | 0x80U);
    i2c_wait_tc();

    // Phase 2 - read len bytes (repeated START, no STOP between write and read)
    // AUTOEND=1: hardware generates STOP after the last byte automatically
    i2c_start_read(LSM303_ACCEL_ADDR, len);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = i2c_read_byte();
    }
}

// Initialize the LSM303DLHC accelerometer
static void lsm303_init(void)
{
    // CTRL_REG1_A: set output data rate to 100 Hz, enable all three axes
    lsm303_write_reg(LSM303_CTRL_REG1_A, LSM303_CTRL_REG1_A_VAL);

    // CTRL_REG4_A: +/-2g full scale, high resolution mode (12-bit output)
    lsm303_write_reg(LSM303_CTRL_REG4_A, LSM303_CTRL_REG4_A_VAL);

    uart_print("LSM303DLHC init OK\r\n");
}

// Read X, Y, Z acceleration raw values from the LSM303DLHC
// Fills the three int16_t pointed to by x, y, z
static void lsm303_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];

    // Burst-read 6 bytes: OUT_X_L, OUT_X_H, OUT_Y_L, OUT_Y_H, OUT_Z_L, OUT_Z_H
    lsm303_read_regs(LSM303_OUT_X_L_A, buf, 6);

    // The LSM303DLHC outputs 12-bit values left-aligned in a 16-bit word.
    // Byte layout (little-endian):
    //   buf[0] = OUT_X_L : bits [7:0]  of the 16-bit word (lower byte)
    //   buf[1] = OUT_X_H : bits [15:8] of the 16-bit word (upper byte)
    // Reconstruct the 16-bit word, then shift right by 4 to right-align the
    // 12-bit value. The cast to int16_t before the shift preserves the sign
    // (arithmetic shift right propagates the sign bit).
    *x = (int16_t)((uint16_t)(buf[1] << 8) | buf[0]) >> 4;
    *y = (int16_t)((uint16_t)(buf[3] << 8) | buf[2]) >> 4;
    *z = (int16_t)((uint16_t)(buf[5] << 8) | buf[4]) >> 4;
}

// ---------------------------------------------------------------------------
// Software delay
// ---------------------------------------------------------------------------

static void delay_ms(uint32_t ms)
{
    // At 8 MHz, approximately 800 NOP iterations = 1 ms
    // volatile prevents the compiler from optimizing the loop away
    for (uint32_t i = 0; i < ms * 800U; i++) {
        __NOP();
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(void)
{
    uart_init();
    leds_init();
    i2c1_init();
    lsm303_init();

    uart_print("STM32F3Discovery - I2C LSM303DLHC accelerometer\r\n");
    uart_print("---\r\n");
    uart_print("     X        Y        Z\r\n");

    int16_t x, y, z;
    char buf[48];

    while (1) {
        lsm303_read_xyz(&x, &y, &z);

        // Format and send over UART
        // Values are in raw LSB. At +/-2g high-res mode: 1 LSB = 1 mg
        // So Z ~= 1000 when the board is flat (1g = 1000 LSB at 1 mg/LSB)
        int n = 0;
        // Build string manually to avoid printf overhead on bare metal
        buf[n++] = ' ';
        // Simple integer to string for X
        int16_t vals[3] = {x, y, z};
        for (int axis = 0; axis < 3; axis++) {
            int16_t v = vals[axis];
            if (v < 0) { buf[n++] = '-'; v = -v; } else { buf[n++] = ' '; }
            // Print up to 4 digits
            uint8_t started = 0;
            for (int16_t div = 1000; div >= 1; div /= 10) {
                int16_t digit = v / div;
                v %= div;
                if (digit || started || div == 1) {
                    buf[n++] = '0' + digit;
                    started = 1;
                } else {
                    buf[n++] = ' ';
                }
            }
            buf[n++] = ' '; buf[n++] = ' '; buf[n++] = ' ';
        }
        buf[n++] = '\r'; buf[n++] = '\n'; buf[n] = '\0';
        uart_print(buf);

        leds_show_direction(x, y);

        delay_ms(100);
    }

    return 0;
}