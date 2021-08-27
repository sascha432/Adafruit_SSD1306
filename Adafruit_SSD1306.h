/*!
 * @file Adafruit_SSD1306.h
 *
 * This is part of for Adafruit's SSD1306 library for monochrome
 * OLED displays: http://www.adafruit.com/category/63_98
 *
 * These displays use I2C or SPI to communicate. I2C requires 2 pins
 * (SCL+SDA) and optionally a RESET pin. SPI requires 4 pins (MOSI, SCK,
 * select, data/command) and optionally a reset pin. Hardware SPI or
 * 'bitbang' software SPI are both supported.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries, with
 * contributions from the open source community.
 *
 * BSD license, all text above, and the splash screen header file,
 * must be included in any redistribution.
 *
 */

#pragma once

#pragma GCC optimize("Os")

#ifdef __AVR__
#    include <avr/pgmspace.h>
#elif defined(ESP8266) || defined(ESP32)
#    include <pgmspace.h>
#else
#    define pgm_read_byte(addr) \
        (*(const unsigned char *)(addr)) ///< PROGMEM workaround for non-AVR
#endif

#ifndef __CONSTEXPR17
#    if __GNUC__ >= 7
#        define __CONSTEXPR17 constexpr
#    else
#        define __CONSTEXPR17
#    endif
#endif

#if !defined(__ARM_ARCH) && !defined(ENERGIA) && !defined(ESP8266) && !defined(ESP32) && !defined(__arc__)
#    include <util/delay.h>
#endif

// ONE of the following three lines must be #defined:
//#define SSD1306_128_64 ///< DEPRECTAED: old way to specify 128x64 screen
#define SSD1306_128_32 ///< DEPRECATED: old way to specify 128x32 screen
//#define SSD1306_96_16  ///< DEPRECATED: old way to specify 96x16 screen
// This establishes the screen dimensions in old Adafruit_SSD1306 sketches
// (NEW CODE SHOULD IGNORE THIS, USE THE CONSTRUCTORS THAT ACCEPT WIDTH
// AND HEIGHT ARGUMENTS).

// reduce code size with fixed width and height
#if !defined(ADAFRUIT_SSD1306_FIXED_WIDTH) || !defined(ADAFRUIT_SSD1306_FIXED_HEIGHT)
#    define ADAFRUIT_SSD1306_FIXED_WIDTH  128
#    define ADAFRUIT_SSD1306_FIXED_HEIGHT 32
#endif

#if defined(ADAFRUIT_SSD1306_FIXED_WIDTH) && defined(ADAFRUIT_SSD1306_FIXED_HEIGHT)
#    define ADAFRUIT_SSD1306_FIXED_SIZE (ADAFRUIT_SSD1306_FIXED_WIDTH * ((ADAFRUIT_SSD1306_FIXED_HEIGHT + 7) / 8))
#else
#    define ADAFRUIT_SSD1306_FIXED_SIZE 0
#endif

// do not include splash screen to safe space
#ifndef ADAFRUIT_SSD1306_DISPLAY_SPLASH
#    define ADAFRUIT_SSD1306_DISPLAY_SPLASH 0
#endif

// remove code for SPI
#ifndef ADAFRUIT_SSD1306_NO_SPI
#    define ADAFRUIT_SSD1306_NO_SPI 1
#endif

// remove destructor from class
#ifndef ADAFRUIT_SSD1306_NO_DESTRUCTOR
#    define ADAFRUIT_SSD1306_NO_DESTRUCTOR 1
#endif

// remove code to change clock speed
#ifndef ADAFRUIT_SSD1306_WIRE_SET_CLOCK
#    define ADAFRUIT_SSD1306_WIRE_SET_CLOCK 0
#endif

// remove code for wire variable and use Wire
#ifndef ADAFRUIT_SSD1306_FIXED_WIRE
#    define ADAFRUIT_SSD1306_FIXED_WIRE 1
#endif

// set fixed vcc state
#ifndef ADAFRUIT_SSD1306_FIXED_VCCSTATE
#    define ADAFRUIT_SSD1306_FIXED_VCCSTATE 1
#endif

// remove variable
#ifndef ADAFRUIT_SSD1306_VCCSTATE
#    define ADAFRUIT_SSD1306_VCCSTATE SSD1306_SWITCHCAPVCC
#endif

// set fixed rotation 0-3 or -1 for reading it from the GFX class
#ifndef ADAFRUIT_SSD1306_FIXED_ROTATION
#    define ADAFRUIT_SSD1306_FIXED_ROTATION 0
#endif

#if defined(ARDUINO_STM32_FEATHER)
typedef class HardwareSPI SPIClass;
#endif

#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Wire.h>

// SOME DEFINES AND STATIC VARIABLES USED INTERNALLY -----------------------

#if defined(BUFFER_LENGTH)
#    define WIRE_MAX BUFFER_LENGTH ///< AVR or similar Wire lib
#elif defined(SERIAL_BUFFER_SIZE)
#    define WIRE_MAX (SERIAL_BUFFER_SIZE - 1) ///< Newer Wire uses RingBuffer
#else
#    define WIRE_MAX 32 ///< Use common Arduino core default
#endif

#define ssd1306_swap(a, b) \
    (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b))) ///< No-temp-var swap operation

#if ARDUINO >= 100
#    define WIRE_WRITE __wire()->write ///< Wire write function in recent Arduino lib
#else
#    define WIRE_WRITE __wire()->send ///< Wire write function in older Arduino lib
#endif

#if ADAFRUIT_SSD1306_NO_SPI
#    define SSD1306_SELECT
#    define SSD1306_DESELECT
#    define SSD1306_MODE_COMMAND
#    define SSD1306_MODE_DATA
#elif defined(HAVE_PORTREG)
#    define SSD1306_SELECT       *csPort &= ~csPinMask; ///< Device select
#    define SSD1306_DESELECT     *csPort |= csPinMask; ///< Device deselect
#    define SSD1306_MODE_COMMAND *dcPort &= ~dcPinMask; ///< Command mode
#    define SSD1306_MODE_DATA    *dcPort |= dcPinMask; ///< Data mode
#else
#    define SSD1306_SELECT       digitalWrite(csPin, LOW); ///< Device select
#    define SSD1306_DESELECT     digitalWrite(csPin, HIGH); ///< Device deselect
#    define SSD1306_MODE_COMMAND digitalWrite(dcPin, LOW); ///< Command mode
#    define SSD1306_MODE_DATA    digitalWrite(dcPin, HIGH); ///< Data mode
#endif

#if defined(__AVR__)
typedef volatile uint8_t PortReg;
typedef uint8_t PortMask;
#    define HAVE_PORTREG
#elif defined(__SAM3X8E__)
typedef volatile RwReg PortReg;
typedef uint32_t PortMask;
#    define HAVE_PORTREG
#elif (defined(__arm__) || defined(ARDUINO_FEATHER52)) && !defined(ARDUINO_ARCH_MBED)
typedef volatile uint32_t PortReg;
typedef uint32_t PortMask;
#    define HAVE_PORTREG
#endif

/// The following "raw" color names are kept for backwards client compatability
/// They can be disabled by predefining this macro before including the Adafruit header
/// client code will then need to be modified to use the scoped enum values directly
#ifndef NO_ADAFRUIT_SSD1306_COLOR_COMPATIBILITY
#    define BLACK   SSD1306_BLACK ///< Draw 'off' pixels
#    define WHITE   SSD1306_WHITE ///< Draw 'on' pixels
#    define INVERSE SSD1306_INVERSE ///< Invert pixels
#endif
/// fit into the SSD1306_ naming scheme
#define SSD1306_BLACK   0 ///< Draw 'off' pixels
#define SSD1306_WHITE   1 ///< Draw 'on' pixels
#define SSD1306_INVERSE 2 ///< Invert pixels

#define SSD1306_MEMORYMODE          0x20 ///< See datasheet
#define SSD1306_COLUMNADDR          0x21 ///< See datasheet
#define SSD1306_PAGEADDR            0x22 ///< See datasheet
#define SSD1306_SETCONTRAST         0x81 ///< See datasheet
#define SSD1306_CHARGEPUMP          0x8D ///< See datasheet
#define SSD1306_SEGREMAP            0xA0 ///< See datasheet
#define SSD1306_DISPLAYALLON_RESUME 0xA4 ///< See datasheet
#define SSD1306_DISPLAYALLON        0xA5 ///< Not currently used
#define SSD1306_NORMALDISPLAY       0xA6 ///< See datasheet
#define SSD1306_INVERTDISPLAY       0xA7 ///< See datasheet
#define SSD1306_SETMULTIPLEX        0xA8 ///< See datasheet
#define SSD1306_DISPLAYOFF          0xAE ///< See datasheet
#define SSD1306_DISPLAYON           0xAF ///< See datasheet
#define SSD1306_COMSCANINC          0xC0 ///< Not currently used
#define SSD1306_COMSCANDEC          0xC8 ///< See datasheet
#define SSD1306_SETDISPLAYOFFSET    0xD3 ///< See datasheet
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5 ///< See datasheet
#define SSD1306_SETPRECHARGE        0xD9 ///< See datasheet
#define SSD1306_SETCOMPINS          0xDA ///< See datasheet
#define SSD1306_SETVCOMDETECT       0xDB ///< See datasheet

#define SSD1306_SETLOWCOLUMN  0x00 ///< Not currently used
#define SSD1306_SETHIGHCOLUMN 0x10 ///< Not currently used
#define SSD1306_SETSTARTLINE  0x40 ///< See datasheet

#define SSD1306_EXTERNALVCC  0x01 ///< External display voltage source
#define SSD1306_SWITCHCAPVCC 0x02 ///< Gen. display voltage from 3.3V

#define SSD1306_RIGHT_HORIZONTAL_SCROLL              0x26 ///< Init rt scroll
#define SSD1306_LEFT_HORIZONTAL_SCROLL               0x27 ///< Init left scroll
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29 ///< Init diag scroll
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A ///< Init diag scroll
#define SSD1306_DEACTIVATE_SCROLL                    0x2E ///< Stop scroll
#define SSD1306_ACTIVATE_SCROLL                      0x2F ///< Start scroll
#define SSD1306_SET_VERTICAL_SCROLL_AREA             0xA3 ///< Set scroll range

// Deprecated size stuff for backwards compatibility with old sketches
#if defined SSD1306_128_64
#    define SSD1306_LCDWIDTH  128 ///< DEPRECATED: width w/SSD1306_128_64 defined
#    define SSD1306_LCDHEIGHT 64 ///< DEPRECATED: height w/SSD1306_128_64 defined
#endif
#if defined SSD1306_128_32
#    define SSD1306_LCDWIDTH  128 ///< DEPRECATED: width w/SSD1306_128_32 defined
#    define SSD1306_LCDHEIGHT 32 ///< DEPRECATED: height w/SSD1306_128_32 defined
#endif
#if defined SSD1306_96_16
#    define SSD1306_LCDWIDTH  96 ///< DEPRECATED: width w/SSD1306_96_16 defined
#    define SSD1306_LCDHEIGHT 16 ///< DEPRECATED: height w/SSD1306_96_16 defined
#endif

#if defined(SPI_HAS_TRANSACTION) && !defined(ADAFRUIT_SSD1306_NO_SPI)
#    define SPI_TRANSACTION_START spi->beginTransaction(spiSettings) ///< Pre-SPI
#    define SPI_TRANSACTION_END   spi->endTransaction() ///< Post-SPI
#else // SPI transactions likewise not present in older Arduino SPI lib
#    define SPI_TRANSACTION_START ///< Dummy stand-in define
#    define SPI_TRANSACTION_END   ///< keeps compiler happy
#endif

#if (ARDUINO >= 157) && !defined(ARDUINO_STM32_FEATHER) && ADAFRUIT_SSD1306_WIRE_SET_CLOCK
#    define SETWIRECLOCK __wire()->setClock(wireClk) ///< Set before I2C transfer
#    define RESWIRECLOCK __wire()->setClock(restoreClk) ///< Restore after I2C xfer
#else // setClock() is not present in older Arduino Wire lib (or WICED)
#    if ADAFRUIT_SSD1306_WIRE_SET_CLOCK
#        undef ADAFRUIT_SSD1306_WIRE_SET_CLOCK
#        define ADAFRUIT_SSD1306_WIRE_SET_CLOCK 0
#    endif
#    define SETWIRECLOCK ///< Dummy stand-in define
#    define RESWIRECLOCK ///< keeps compiler happy
#endif

#if ADAFRUIT_SSD1306_NO_SPI == 0

// Check first if Wire, then hardware SPI, then soft SPI:
#    define TRANSACTION_START          \
        if (__hasWire()) {             \
            SETWIRECLOCK;              \
        } else {                       \
            if (spi) {                 \
                SPI_TRANSACTION_START; \
            }                          \
            SSD1306_SELECT;            \
        } ///< Wire, SPI or bitbang transfer setup
#    define TRANSACTION_END          \
        if (__hasWire()) {           \
            RESWIRECLOCK;            \
        } else {                     \
            SSD1306_DESELECT;        \
            if (spi) {               \
                SPI_TRANSACTION_END; \
            }                        \
        } ///< Wire, SPI or bitbang transfer end
#else

// Check first if Wire, then hardware SPI, then soft SPI:
#    define TRANSACTION_START \
        SETWIRECLOCK;         \
        ///< Wire, SPI or bitbang transfer setup
#    define TRANSACTION_END \
        RESWIRECLOCK;       \
        ///< Wire, SPI or bitbang transfer end
#endif

#ifndef ADAFRUIT_SSD1306_DEFAULT_I2C_ADDRESS
#    define ADAFRUIT_SSD1306_DEFAULT_I2C_ADDRESS 0x3c
#endif

/*!
    @brief  Class that stores state and functions for interacting with
            SSD1306 OLED displays.
*/
class Adafruit_SSD1306 : public Adafruit_GFX {
public:
    Adafruit_SSD1306(
#if ADAFRUIT_SSD1306_FIXED_SIZE == 0
        uint8_t w, uint8_t h,
#endif
        uint8_t i2cAddress,
#if ADAFRUIT_SSD1306_FIXED_WIRE == 0
        TwoWire *twi,
#endif
        int8_t rst_pin
#if ADAFRUIT_SSD1306_WIRE_SET_CLOCK
        ,
        uint32_t clkDuring = 400000UL, uint32_t clkAfter = 100000UL
#endif
    );

#if ADAFRUIT_SSD1306_NO_SPI == 0
    Adafruit_SSD1306(uint8_t w, uint8_t h, int8_t mosi_pin, int8_t sclk_pin, int8_t dc_pin, int8_t rst_pin, int8_t cs_pin);
    Adafruit_SSD1306(uint8_t w, uint8_t h, SPIClass *spi, int8_t dc_pin, int8_t rst_pin, int8_t cs_pin, uint32_t bitrate = 8000000UL);
#endif

#ifndef ADAFRUIT_SSD1306_NO_DESTRUCTOR
    ~Adafruit_SSD1306(void);
#endif

    bool begin(
#if ADAFRUIT_SSD1306_FIXED_VCCSTATE == 0
        uint8_t switchvcc,
#endif
        uint8_t i2caddr, boolean reset = true, boolean periphBegin = true);

    void display(void);
    void clearDisplay(void);
    void invertDisplay(boolean i);
    void dim(boolean dim);
    void drawPixel(int16_t x, int16_t y, uint16_t color);
    virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
    virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
    void startscrollright(uint8_t start, uint8_t stop);
    void startscrollleft(uint8_t start, uint8_t stop);
    void startscrolldiagright(uint8_t start, uint8_t stop);
    void startscrolldiagleft(uint8_t start, uint8_t stop);
    void stopscroll(void);
    void ssd1306_command(uint8_t c);
    bool getPixel(int16_t x, int16_t y);
    uint8_t *getBuffer(void);

private:
#if ADAFRUIT_SSD1306_NO_SPI == 0
    void SPIwrite(uint8_t d);
#endif
    void drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color);
    void drawFastVLineInternal(int16_t x, int16_t y, int16_t h, uint16_t color);
    void ssd1306_command1(uint8_t c);
    void ssd1306_commandList(const uint8_t *c, uint8_t n);

#if ADAFRUIT_SSD1306_NO_SPI == 0
    SPIClass *spi;
#endif
#if ADAFRUIT_SSD1306_FIXED_WIRE == 0
    TwoWire *wire;
    inline __attribute__((always_inline)) bool __hasWire() const
    {
        return wire != nullptr;
    }
    inline __attribute__((always_inline)) TwoWire *__wire() const
    {
        return wire;
    }
#else
    constexpr bool __hasWire() const
    {
        return true;
    }
    constexpr TwoWire *__wire() const
    {
        return &Wire;
    }
#endif
#if ADAFRUIT_SSD1306_FIXED_SIZE
    constexpr uint8_t __width() const
    {
        return ADAFRUIT_SSD1306_FIXED_WIDTH;
    }
    constexpr uint8_t __height() const
    {
        return ADAFRUIT_SSD1306_FIXED_HEIGHT;
    }
    constexpr uint16_t __size() const
    {
        return ADAFRUIT_SSD1306_FIXED_SIZE;
    }
    uint8_t buffer[ADAFRUIT_SSD1306_FIXED_SIZE];
#else
    uint16_t __width() const
    {
        return WIDTH;
    }
    uint16_t __height() const
    {
        return HEIGHT;
    }
    uint16_t __size() const
    {
        return __width() * ((__height() + 7) / 8));
    }
    uint8_t *buffer;
#endif
    int8_t i2caddr;
#if ADAFRUIT_SSD1306_FIXED_VCCSTATE
    constexpr int8_t __vccstate() const
    {
        return ADAFRUIT_SSD1306_VCCSTATE;
    }
#else
    int8_t vccstate;
    int8_t __vccstate() const
    {
        return vccstate;
    }

#endif
    // int8_t page_end;
    int8_t rstPin;
#if ADAFRUIT_SSD1306_NO_SPI == 0
    int8_t mosiPin, clkPin, dcPin, csPin;
#    ifdef HAVE_PORTREG
    PortReg *mosiPort, *clkPort, *dcPort, *csPort;
    PortMask mosiPinMask, clkPinMask, dcPinMask, csPinMask;
#    endif
#    if defined(SPI_HAS_TRANSACTION)
    SPISettings spiSettings;
#    endif

#    if ARDUINO >= 157 && ADAFRUIT_SSD1306_WIRE_SET_CLOCK
    uint32_t wireClk; // Wire speed for SSD1306 transfers
    uint32_t restoreClk; // Wire speed following SSD1306 transfers
#    endif
#endif
};

inline Adafruit_SSD1306::Adafruit_SSD1306(
#if ADAFRUIT_SSD1306_FIXED_SIZE == 0
    uint8_t w, uint8_t h,
#endif
#if ADAFRUIT_SSD1306_FIXED_WIRE == 0
    TwoWire *twi,
#endif
    uint8_t i2cAddress,
    int8_t rst_pin
#if ADAFRUIT_SSD1306_WIRE_SET_CLOCK
    ,
    uint32_t clkDuring, uint32_t clkAfter
#endif
    ) :
#if ADAFRUIT_SSD1306_FIXED_SIZE
    Adafruit_GFX(__width(), __height()),
#else
    Adafruit_GFX(w, h),
#endif
#if ADAFRUIT_SSD1306_NO_SPI == 0
    spi(NULL),
#endif
#if ADAFRUIT_SSD1306_FIXED_WIRE == 0
    wire(twi ? twi : &Wire),
#endif
#if ADAFRUIT_SSD1306_FIXED_SIZE == 0
    buffer(NULL),
#endif
    i2caddr(i2cAddress),
    rstPin(rst_pin)
#if ADAFRUIT_SSD1306_NO_SPI == 0
    ,
    mosiPin(-1), clkPin(-1), dcPin(-1), csPin(-1)
#endif
#if ADAFRUIT_SSD1306_WIRE_SET_CLOCK
    ,
    wireClk(clkDuring), restoreClk(clkAfter)
#endif
{
}

#if ADAFRUIT_SSD1306_NO_DESTRUCTOR == 0
inline Adafruit_SSD1306::~Adafruit_SSD1306(void)
{
#    if ADAFRUIT_SSD1306_FIXED_SIZE == 0
    if (buffer) {
        free(buffer);
        buffer = NULL;
    }
#    endif
}
#endif

inline bool Adafruit_SSD1306::begin(
#if ADAFRUIT_SSD1306_FIXED_VCCSTATE == 0
    uint8_t vcs,
#endif
    uint8_t addr, bool reset, bool periphBegin)
{
#if ADAFRUIT_SSD1306_FIXED_SIZE == 0

    if ((!buffer) && !(buffer = (uint8_t *)malloc(__width() * ((__height() + 7) / 8)))) {
        return false;
    }

#endif

    i2caddr = addr;
#if ADAFRUIT_SSD1306_FIXED_VCCSTATE == 0
    vccstate = vcs;
#endif

    clearDisplay();
#if ADAFRUIT_SSD1306_DISPLAY_SPLASH
    if (__height() > 32) {
        drawBitmap((__width() - splash1_width) / 2, (__height() - splash1_height) / 2,
            splash1_data, splash1_width, splash1_height, 1);
    } else {
        drawBitmap((__width() - splash2_width) / 2, (__height() - splash2_height) / 2,
            splash2_data, splash2_width, splash2_height, 1);
    }
#endif

    // Setup pin directions
#if ADAFRUIT_SSD1306_NO_SPI == 0
    if (__hasWire())
#endif
    {
        // Using I2C
        // TwoWire begin() function might be already performed by the calling
        // function if it has unusual circumstances (e.g. TWI variants that
        // can accept different SDA/SCL pins, or if two SSD1306 instances
        // with different addresses -- only a single begin() is needed).
        if (periphBegin) {
            __wire()->begin();
        }
    }
#if ADAFRUIT_SSD1306_NO_SPI == 0
    else { // Using one of the SPI modes, either soft or hardware
        pinMode(dcPin, OUTPUT); // Set data/command pin as output
        pinMode(csPin, OUTPUT); // Same for chip select
#    ifdef HAVE_PORTREG
        dcPort = (PortReg *)portOutputRegister(digitalPinToPort(dcPin));
        dcPinMask = digitalPinToBitMask(dcPin);
        csPort = (PortReg *)portOutputRegister(digitalPinToPort(csPin));
        csPinMask = digitalPinToBitMask(csPin);
#    endif
        SSD1306_DESELECT
        if (spi) { // Hardware SPI
            // SPI peripheral begin same as wire check above.
            if (periphBegin)
                spi->begin();
        } else { // Soft SPI
            pinMode(mosiPin, OUTPUT); // MOSI and SCLK outputs
            pinMode(clkPin, OUTPUT);
#    ifdef HAVE_PORTREG
            mosiPort = (PortReg *)portOutputRegister(digitalPinToPort(mosiPin));
            mosiPinMask = digitalPinToBitMask(mosiPin);
            clkPort = (PortReg *)portOutputRegister(digitalPinToPort(clkPin));
            clkPinMask = digitalPinToBitMask(clkPin);
            *clkPort &= ~clkPinMask; // Clock low
#    else
            digitalWrite(clkPin, LOW); // Clock low
#    endif
        }
    }
#endif

    // Reset SSD1306 if requested and reset pin specified in constructor
    if (reset && (rstPin >= 0)) {
        pinMode(rstPin, OUTPUT);
        digitalWrite(rstPin, HIGH);
        delay(1); // VDD goes high at start, pause for 1 ms
        digitalWrite(rstPin, LOW); // Bring reset low
        delay(10); // Wait 10 ms
        digitalWrite(rstPin, HIGH); // Bring out of reset
    }

    TRANSACTION_START

    // Init sequence
    static const uint8_t PROGMEM init1[] = {
        SSD1306_DISPLAYOFF, // 0xAE
        SSD1306_SETDISPLAYCLOCKDIV, // 0xD5
        0x80, // the suggested ratio 0x80
        SSD1306_SETMULTIPLEX
    }; // 0xA8
    ssd1306_commandList(init1, sizeof(init1));
    ssd1306_command1(__height() - 1);

    static const uint8_t PROGMEM init2[] = {
        SSD1306_SETDISPLAYOFFSET, // 0xD3
        0x0, // no offset
        SSD1306_SETSTARTLINE | 0x0, // line #0
        SSD1306_CHARGEPUMP
    }; // 0x8D
    ssd1306_commandList(init2, sizeof(init2));

    ssd1306_command1((__vccstate() == SSD1306_EXTERNALVCC) ? 0x10 : 0x14);

    static const uint8_t PROGMEM init3[] = {
        SSD1306_MEMORYMODE, // 0x20
        0x00, // 0x0 act like ks0108
        SSD1306_SEGREMAP | 0x1,
        SSD1306_COMSCANDEC
    };
    ssd1306_commandList(init3, sizeof(init3));

#if ADAFRUIT_SSD1306_FIXED_SIZE
#    if ADAFRUIT_SSD1306_FIXED_WIDTH == 128 && ADAFRUIT_SSD1306_FIXED_HEIGHT == 32
    static const uint8_t PROGMEM init4a[] = {
        SSD1306_SETCOMPINS, // 0xDA
        0x02,
        SSD1306_SETCONTRAST, // 0x81
        0x8F
    };
    ssd1306_commandList(init4a, sizeof(init4a));
#    elif ADAFRUIT_SSD1306_FIXED_WIDTH == 128 && ADAFRUIT_SSD1306_FIXED_HEIGHT == 64
    static const uint8_t PROGMEM init4b[] = {
        SSD1306_SETCOMPINS, // 0xDA
        0x12,
        SSD1306_SETCONTRAST
    }; // 0x81
    ssd1306_commandList(init4b, sizeof(init4b));
    ssd1306_command1((__vccstate() == SSD1306_EXTERNALVCC) ? 0x9F : 0xCF);
#    else
#        errpr not supported
#    endif
#else
    if ((__width() == 128) && (__height() == 32)) {
        static const uint8_t PROGMEM init4a[] = {
            SSD1306_SETCOMPINS, // 0xDA
            0x02,
            SSD1306_SETCONTRAST, // 0x81
            0x8F
        };
        ssd1306_commandList(init4a, sizeof(init4a));
    } else if ((__width() == 128) && (__height() == 64)) {
        static const uint8_t PROGMEM init4b[] = {
            SSD1306_SETCOMPINS, // 0xDA
            0x12,
            SSD1306_SETCONTRAST
        }; // 0x81
        ssd1306_commandList(init4b, sizeof(init4b));
        ssd1306_command1((__vccstate() == SSD1306_EXTERNALVCC) ? 0x9F : 0xCF);
    } else if ((__width() == 96) && (__height() == 16)) {
        static const uint8_t PROGMEM init4c[] = {
            SSD1306_SETCOMPINS, // 0xDA
            0x2, // ada x12
            SSD1306_SETCONTRAST
        }; // 0x81
        ssd1306_commandList(init4c, sizeof(init4c));
        ssd1306_command1((__vccstate() == SSD1306_EXTERNALVCC) ? 0x10 : 0xAF);
    } else {
        // Other screen varieties -- TBD
    }
#endif

    ssd1306_command1(SSD1306_SETPRECHARGE); // 0xd9
    ssd1306_command1((__vccstate() == SSD1306_EXTERNALVCC) ? 0x22 : 0xF1);
    static const uint8_t PROGMEM init5[] = {
        SSD1306_SETVCOMDETECT, // 0xDB
        0x40,
        SSD1306_DISPLAYALLON_RESUME, // 0xA4
        SSD1306_NORMALDISPLAY, // 0xA6
        SSD1306_DEACTIVATE_SCROLL,
        SSD1306_DISPLAYON
    }; // Main screen turn on
    ssd1306_commandList(init5, sizeof(init5));

    TRANSACTION_END

    return true; // Success
}

inline void Adafruit_SSD1306::ssd1306_command(uint8_t c)
{
    TRANSACTION_START
    ssd1306_command1(c);
    TRANSACTION_END
}

#if ADAFRUIT_SSD1306_NO_SPI == 0
inline
    __attribute__((always_inline)) void
    Adafruit_SSD1306::SPIwrite(uint8_t d)
{
    if (spi) {
        (void)spi->transfer(d);
    } else {
        for (uint8_t bit = 0x80; bit; bit >>= 1) {
#    ifdef HAVE_PORTREG
            if (d & bit)
                *mosiPort |= mosiPinMask;
            else
                *mosiPort &= ~mosiPinMask;
            *clkPort |= clkPinMask; // Clock high
            *clkPort &= ~clkPinMask; // Clock low
#    else
            digitalWrite(mosiPin, d & bit);
            digitalWrite(clkPin, HIGH);
            digitalWrite(clkPin, LOW);
#    endif
        }
    }
}

inline Adafruit_SSD1306::Adafruit_SSD1306(uint8_t w, uint8_t h,
    int8_t mosi_pin, int8_t sclk_pin, int8_t dc_pin, int8_t rst_pin,
    int8_t cs_pin) :
    Adafruit_GFX(w, h),
    spi(NULL),
#    if ADAFRUIT_SSD1306_FIXED_WIRE == 0
    wire(NULL),
#    endif
#    if ADAFRUIT_SSD1306_FIXED_SIZE == 0
    buffer(NULL),
#    endif
    rstPin(rst_pin),
    mosiPin(mosi_pin),
    clkPin(sclk_pin),
    dcPin(dc_pin),
    csPin(cs_pin)
{
}

inline Adafruit_SSD1306::Adafruit_SSD1306(uint8_t w, uint8_t h, SPIClass *spi, int8_t dc_pin, int8_t rst_pin, int8_t cs_pin, uint32_t bitrate) :
    Adafruit_GFX(w, h),
    spi(spi ? spi : &SPI),
#    if ADAFRUIT_SSD1306_FIXED_WIRE == 0
    wire(NULL),
#    endif
#    if ADAFRUIT_SSD1306_FIXED_SIZE == 0
    buffer(NULL),
#    endif
    rstPin(rst_pin),
    mosiPin(-1),
    clkPin(-1),
    dcPin(dc_pin),
    csPin(cs_pin)
{
#    ifdef SPI_HAS_TRANSACTION
    spiSettings = SPISettings(bitrate, MSBFIRST, SPI_MODE0);
#    endif
}

#endif

inline void Adafruit_SSD1306::ssd1306_command1(uint8_t c)
{
#if ADAFRUIT_SSD1306_NO_SPI == 0
    if (__hasWire()) { // I2C1
#endif
        __wire()->beginTransmission(i2caddr);
        WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
        WIRE_WRITE(c);
        __wire()->endTransmission();
#if ADAFRUIT_SSD1306_NO_SPI == 0
    } else { // SPI (hw or soft) -- transaction started in calling function
        SSD1306_MODE_COMMAND
        SPIwrite(c);
    }
#endif
}

inline void Adafruit_SSD1306::ssd1306_commandList(const uint8_t *c, uint8_t n)
{
#if ADAFRUIT_SSD1306_NO_SPI == 0
    if (__hasWire()) { // I2C
#endif
        __wire()->beginTransmission(i2caddr);
        WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
        uint8_t bytesOut = 1;
        while (n--) {
            if (bytesOut >= WIRE_MAX) {
                __wire()->endTransmission();
                __wire()->beginTransmission(i2caddr);
                WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
                bytesOut = 1;
            }
            WIRE_WRITE(pgm_read_byte(c++));
            bytesOut++;
        }
        __wire()->endTransmission();
#if ADAFRUIT_SSD1306_NO_SPI == 0
    } else { // SPI -- transaction started in calling function
        SSD1306_MODE_COMMAND
        while (n--)
            SPIwrite(pgm_read_byte(c++));
    }
#endif
}

inline uint8_t *Adafruit_SSD1306::getBuffer(void)
{
    return buffer;
}

inline void Adafruit_SSD1306::display(void)
{
    TRANSACTION_START
    static const uint8_t PROGMEM dlist1[] = {
        SSD1306_PAGEADDR,
        0, // Page start address
        0xFF, // Page end (not really, but works here)
        SSD1306_COLUMNADDR,
        0
    }; // Column start address
    ssd1306_commandList(dlist1, sizeof(dlist1));
    ssd1306_command1(width() - 1); // Column end address

#if defined(ESP8266)
    // ESP8266 needs a periodic yield() call to avoid watchdog reset.
    // With the limited size of SSD1306 displays, and the fast bitrate
    // being used (1 MHz or more), I think one yield() immediately before
    // a screen write and one immediately after should cover it.  But if
    // not, if this becomes a problem, yields() might be added in the
    // 32-byte transfer condition below.
    yield();
#endif
    uint16_t count = __width() * ((__height() + 7) / 8);
    uint8_t *ptr = buffer;
#if ADAFRUIT_SSD1306_NO_SPI == 0
    if (__hasWire()) { // I2C
#endif
        __wire()->beginTransmission(i2caddr);
        WIRE_WRITE((uint8_t)0x40);
        uint8_t bytesOut = 1;
        while (count--) {
            if (bytesOut >= WIRE_MAX) {
                __wire()->endTransmission();
                __wire()->beginTransmission(i2caddr);
                WIRE_WRITE((uint8_t)0x40);
                bytesOut = 1;
            }
            WIRE_WRITE(*ptr++);
            bytesOut++;
        }
        __wire()->endTransmission();
#if ADAFRUIT_SSD1306_NO_SPI == 0
    } else { // SPI
        SSD1306_MODE_DATA
        while (count--)
            SPIwrite(*ptr++);
    }
#endif
    TRANSACTION_END
#if defined(ESP8266)
    yield();
#endif
}

inline void Adafruit_SSD1306::clearDisplay(void)
{
    memset(buffer, 0, __width() * ((__height() + 7) / 8));
}

inline void Adafruit_SSD1306::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    if (static_cast<uint16_t>(x) < __width() && static_cast<uint16_t>(y) < __height()) {
        // if ((x >= 0) && (x < __width()) && (y >= 0) && (y < __height())) {
        // Pixel is in-bounds. Rotate coordinates if needed.
#if ADAFRUIT_SSD1306_FIXED_ROTATION == -1
        switch (getRotation()) {
        case 1:
            ssd1306_swap(x, y);
            x = __width() - x - 1;
            break;
        case 2:
            x = __width() - x - 1;
            y = __height() - y - 1;
            break;
        case 3:
            ssd1306_swap(x, y);
            y = __height() - y - 1;
            break;
        }
#elif ADAFRUIT_SSD1306_FIXED_ROTATION == 0
        // no change
#elif ADAFRUIT_SSD1306_FIXED_ROTATION == 1
        ssd1306_swap(x, y);
        x = __width() - x - 1;
#elif ADAFRUIT_SSD1306_FIXED_ROTATION == 2
        x = __width() - x - 1;
        y = __height() - y - 1;
#elif ADAFRUIT_SSD1306_FIXED_ROTATION == 3
        ssd1306_swap(x, y);
        y = __height() - y - 1;
#else
#    error invalid value for ADAFRUIT_SSD1306_FIXED_ROTATION
#endif
        auto pos = x + (y / 8) * __width();
        switch (color) {
        case SSD1306_WHITE:
            buffer[pos] |= (1 << (y & 7));
            break;
        case SSD1306_BLACK:
            buffer[pos] &= ~(1 << (y & 7));
            break;
        case SSD1306_INVERSE:
            buffer[pos] ^= (1 << (y & 7));
            break;
        }
    }
}

inline boolean Adafruit_SSD1306::getPixel(int16_t x, int16_t y)
{
    if (static_cast<uint16_t>(x) < __width() && static_cast<uint16_t>(y) < __height()) {
#if ADAFRUIT_SSD1306_FIXED_ROTATION == -1
        // Pixel is in-bounds. Rotate coordinates if needed.
        switch (getRotation()) {
        case 1:
            ssd1306_swap(x, y);
            x = __width() - x - 1;
            break;
        case 2:
            x = __width() - x - 1;
            y = __height() - y - 1;
            break;
        case 3:
            ssd1306_swap(x, y);
            y = __height() - y - 1;
            break;
        }
#elif ADAFRUIT_SSD1306_FIXED_ROTATION == 1
        ssd1306_swap(x, y);
        x = __width() - x - 1;
#elif ADAFRUIT_SSD1306_FIXED_ROTATION == 2
        x = __width() - x - 1;
        y = __height() - y - 1;
#elif ADAFRUIT_SSD1306_FIXED_ROTATION == 3
        ssd1306_swap(x, y);
        y = __height() - y - 1;
#endif
        return (buffer[x + (y / 8) * __width()] & (1 << (y & 7)));
    }
    return false; // Pixel out of bounds
}

inline void Adafruit_SSD1306::startscrollright(uint8_t start, uint8_t stop)
{
    TRANSACTION_START
    static const uint8_t PROGMEM scrollList1a[] = {
        SSD1306_RIGHT_HORIZONTAL_SCROLL,
        0X00
    };
    ssd1306_commandList(scrollList1a, sizeof(scrollList1a));
    ssd1306_command1(start);
    ssd1306_command1(0X00);
    ssd1306_command1(stop);
    static const uint8_t PROGMEM scrollList1b[] = {
        0X00,
        0XFF,
        SSD1306_ACTIVATE_SCROLL
    };
    ssd1306_commandList(scrollList1b, sizeof(scrollList1b));
    TRANSACTION_END
}

inline void Adafruit_SSD1306::startscrollleft(uint8_t start, uint8_t stop)
{
    TRANSACTION_START
    static const uint8_t PROGMEM scrollList2a[] = {
        SSD1306_LEFT_HORIZONTAL_SCROLL,
        0X00
    };
    ssd1306_commandList(scrollList2a, sizeof(scrollList2a));
    ssd1306_command1(start);
    ssd1306_command1(0X00);
    ssd1306_command1(stop);
    static const uint8_t PROGMEM scrollList2b[] = {
        0X00,
        0XFF,
        SSD1306_ACTIVATE_SCROLL
    };
    ssd1306_commandList(scrollList2b, sizeof(scrollList2b));
    TRANSACTION_END
}

inline void Adafruit_SSD1306::startscrolldiagright(uint8_t start, uint8_t stop)
{
    TRANSACTION_START
    static const uint8_t PROGMEM scrollList3a[] = {
        SSD1306_SET_VERTICAL_SCROLL_AREA,
        0X00
    };
    ssd1306_commandList(scrollList3a, sizeof(scrollList3a));
    ssd1306_command1(__height());
    static const uint8_t PROGMEM scrollList3b[] = {
        SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL,
        0X00
    };
    ssd1306_commandList(scrollList3b, sizeof(scrollList3b));
    ssd1306_command1(start);
    ssd1306_command1(0X00);
    ssd1306_command1(stop);
    static const uint8_t PROGMEM scrollList3c[] = {
        0X01,
        SSD1306_ACTIVATE_SCROLL
    };
    ssd1306_commandList(scrollList3c, sizeof(scrollList3c));
    TRANSACTION_END
}

inline void Adafruit_SSD1306::startscrolldiagleft(uint8_t start, uint8_t stop)
{
    TRANSACTION_START
    static const uint8_t PROGMEM scrollList4a[] = {
        SSD1306_SET_VERTICAL_SCROLL_AREA,
        0X00
    };
    ssd1306_commandList(scrollList4a, sizeof(scrollList4a));
    ssd1306_command1(__height());
    static const uint8_t PROGMEM scrollList4b[] = {
        SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL,
        0X00
    };
    ssd1306_commandList(scrollList4b, sizeof(scrollList4b));
    ssd1306_command1(start);
    ssd1306_command1(0X00);
    ssd1306_command1(stop);
    static const uint8_t PROGMEM scrollList4c[] = {
        0X01,
        SSD1306_ACTIVATE_SCROLL
    };
    ssd1306_commandList(scrollList4c, sizeof(scrollList4c));
    TRANSACTION_END
}

inline void Adafruit_SSD1306::stopscroll(void)
{
    TRANSACTION_START
    ssd1306_command1(SSD1306_DEACTIVATE_SCROLL);
    TRANSACTION_END
}

inline void Adafruit_SSD1306::invertDisplay(boolean i)
{
    TRANSACTION_START
    ssd1306_command1(i ? SSD1306_INVERTDISPLAY : SSD1306_NORMALDISPLAY);
    TRANSACTION_END
}

inline void Adafruit_SSD1306::dim(boolean dim)
{
    uint8_t contrast;

    if (dim) {
        contrast = 0; // Dimmed display
    } else {
        contrast = (__vccstate() == SSD1306_EXTERNALVCC) ? 0x9F : 0xCF;
    }
    // the range of contrast to too small to be really useful
    // it is useful to dim the display
    TRANSACTION_START
    ssd1306_command1(SSD1306_SETCONTRAST);
    ssd1306_command1(contrast);
    TRANSACTION_END
}

inline void Adafruit_SSD1306::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
#if ADAFRUIT_SSD1306_FIXED_ROTATION == -1
    boolean bSwap = false;
    switch (rotation) {
    case 1:
        // 90 degree rotation, swap x & y for rotation, then invert x
        bSwap = true;
        ssd1306_swap(x, y);
        x = __width() - x - 1;
        break;
    case 2:
        // 180 degree rotation, invert x and y, then shift y around for height.
        x = __width() - x - 1;
        y = __height() - y - 1;
        x -= (w - 1);
        break;
    case 3:
        // 270 degree rotation, swap x & y for rotation,
        // then invert y and adjust y for w (not to become h)
        bSwap = true;
        ssd1306_swap(x, y);
        y = __height() - y - 1;
        y -= (w - 1);
        break;
    }

    if (bSwap) {
        drawFastVLineInternal(x, y, w, color);
    } else {
        drawFastHLineInternal(x, y, w, color);
    }
#elif ADAFRUIT_SSD1306_FIXED_ROTATION == 0
    drawFastHLineInternal(x, y, w, color);
#elif ADAFRUIT_SSD1306_FIXED_ROTATION == 1
    ssd1306_swap(x, y);
    x = __width() - x - 1;
    drawFastVLineInternal(x, y, w, color);
#elif ADAFRUIT_SSD1306_FIXED_ROTATION == 2
    x = __width() - x - 1;
    y = __height() - y - 1;
    x -= (w - 1);
    drawFastHLineInternal(x, y, w, color);
#elif ADAFRUIT_SSD1306_FIXED_ROTATION == 3
    ssd1306_swap(x, y);
    y = __height() - y - 1;
    y -= (w - 1);
    drawFastVLineInternal(x, y, w, color);
#else
#    error invalid
#endif
}

inline void Adafruit_SSD1306::drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color)
{
    if ((y >= 0) && (y < __height())) { // Y coord in bounds?
        if (x < 0) { // Clip left
            w += x;
            x = 0;
        }
        if ((x + w) > __width()) { // Clip right
            w = (__width() - x);
        }
        if (w > 0) { // Proceed only if width is positive
            uint8_t *pBuf = &buffer[(y / 8) * __width() + x];
            uint8_t mask = 1 << (y & 7);
            switch (color) {
            case SSD1306_WHITE:
                while (w--) {
                    *pBuf++ |= mask;
                };
                break;
            case SSD1306_BLACK:
                mask = ~mask;
                while (w--) {
                    *pBuf++ &= mask;
                };
                break;
            case SSD1306_INVERSE:
                while (w--) {
                    *pBuf++ ^= mask;
                };
                break;
            }
        }
    }
}

inline void Adafruit_SSD1306::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
#if ADAFRUIT_SSD1306_FIXED_ROTATION == -1
    boolean bSwap = false;
    switch (rotation) {
    case 1:
        // 90 degree rotation, swap x & y for rotation,
        // then invert x and adjust x for h (now to become w)
        bSwap = true;
        ssd1306_swap(x, y);
        x = __width() - x - 1;
        x -= (h - 1);
        break;
    case 2:
        // 180 degree rotation, invert x and y, then shift y around for height.
        x = __width() - x - 1;
        y = __height() - y - 1;
        y -= (h - 1);
        break;
    case 3:
        // 270 degree rotation, swap x & y for rotation, then invert y
        bSwap = true;
        ssd1306_swap(x, y);
        y = __height() - y - 1;
        break;
    }
    if (bSwap) {
        drawFastHLineInternal(x, y, h, color);
    } else {
        drawFastVLineInternal(x, y, h, color);
    }

#elif ADAFRUIT_SSD1306_FIXED_ROTATION == 0
    drawFastVLineInternal(x, y, h, color);
#elif ADAFRUIT_SSD1306_FIXED_ROTATION == 1
    ssd1306_swap(x, y);
    x = __width() - x - 1;
    x -= (h - 1);
    drawFastHLineInternal(x, y, h, color);
#elif ADAFRUIT_SSD1306_FIXED_ROTATION == 2
    x = __width() - x - 1;
    y = __height() - y - 1;
    y -= (h - 1);
    drawFastVLineInternal(x, y, h, color);
#elif ADAFRUIT_SSD1306_FIXED_ROTATION == 3
    ssd1306_swap(x, y);
    y = __height() - y - 1;
    drawFastHLineInternal(x, y, h, color);
#else
#    error invalid
#endif
}

inline void Adafruit_SSD1306::drawFastVLineInternal(int16_t x, int16_t __y, int16_t __h, uint16_t color)
{
    if ((x >= 0) && (x < __width())) { // X coord in bounds?
        if (__y < 0) { // Clip top
            __h += __y;
            __y = 0;
        }
        if ((__y + __h) > __height()) { // Clip bottom
            __h = (__height() - __y);
        }
        if (__h > 0) { // Proceed only if height is now positive
            // this display doesn't need ints for coordinates,
            // use local byte registers for faster juggling
            uint8_t y = __y, h = __h;
            uint8_t *pBuf = &buffer[(y / 8) * __width() + x];

            // do the first partial byte, if necessary - this requires some masking
            uint8_t mod = (y & 7);
            if (mod) {
                // mask off the high n bits we want to set
                mod = 8 - mod;
                // note - lookup table results in a nearly 10% performance
                // improvement in fill* functions
                // uint8_t mask = ~(0xFF >> mod);
                static const uint8_t PROGMEM premask[8] = { 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
                uint8_t mask = pgm_read_byte(&premask[mod]);
                // adjust the mask if we're not going to reach the end of this byte
                if (h < mod) {
                    mask &= (0XFF >> (mod - h));
                }

                switch (color) {
                case SSD1306_WHITE:
                    *pBuf |= mask;
                    break;
                case SSD1306_BLACK:
                    *pBuf &= ~mask;
                    break;
                case SSD1306_INVERSE:
                    *pBuf ^= mask;
                    break;
                }
                pBuf += __width();
            }

            if (h >= mod) { // More to go?
                h -= mod;
                // Write solid bytes while we can - effectively 8 rows at a time
                if (h >= 8) {
                    if (color == SSD1306_INVERSE) {
                        // separate copy of the code so we don't impact performance of
                        // black/white write version with an extra comparison per loop
                        do {
                            *pBuf ^= 0xFF; // Invert byte
                            pBuf += __width(); // Advance pointer 8 rows
                            h -= 8; // Subtract 8 rows from height
                        } while (h >= 8);
                    } else {
                        // store a local value to work with
                        uint8_t val = (color != SSD1306_BLACK) ? 255 : 0;
                        do {
                            *pBuf = val; // Set byte
                            pBuf += __width(); // Advance pointer 8 rows
                            h -= 8; // Subtract 8 rows from height
                        } while (h >= 8);
                    }
                }

                if (h) { // Do the final partial byte, if necessary
                    mod = h & 7;
                    // this time we want to mask the low bits of the byte,
                    // vs the high bits we did above
                    // uint8_t mask = (1 << mod) - 1;
                    // note - lookup table results in a nearly 10% performance
                    // improvement in fill* functions
                    static const uint8_t PROGMEM postmask[8] = { 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
                    uint8_t mask = pgm_read_byte(&postmask[mod]);
                    switch (color) {
                    case SSD1306_WHITE:
                        *pBuf |= mask;
                        break;
                    case SSD1306_BLACK:
                        *pBuf &= ~mask;
                        break;
                    case SSD1306_INVERSE:
                        *pBuf ^= mask;
                        break;
                    }
                }
            }
        } // endif positive height
    } // endif x in bounds
}
