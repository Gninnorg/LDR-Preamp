#ifndef DAC7678_H
#define DAC7678_H

#include "Arduino.h"
#include <Wire.h>

#define ENABLE 1
#define DISABLE 0

// VREF options (Flexible Mode)
#define EXT 0 // External VREF
#define INT 1 // Internal VREF

// DAC power mode
#define L100K 0x40
#define L1K 0x20
#define HIGHZ 0x60
#define ON	1
#define OFF 0

// Clear pin Modes
#define NOCLR 3
#define MID 1
#define FULL 2
#define ZERO 0

// Channel # to Pin name numbering
#define PIN_A 0
#define PIN_B 1
#define PIN_C 2
#define PIN_D 3
#define PIN_E 4
#define PIN_F 5
#define PIN_G 6
#define PIN_H 7

#define CMD_WRITE_BASE_ADDR     0x00
#define CMD_SELECT              0x10
#define CMD_IS_LDAC_BASE_ADDR   0x30
#define CMD_POWER_DOWN          0x40
#define CMD_CLEAR_CODE          0x50
#define CMD_LDAC                0x60
#define CMD_SOFT_RESET          0x70
#define CMD_INTREF_RS           0x80

class DAC7678 {

  public:
    DAC7678();
    DAC7678(unsigned char _address);
    void begin();
    void begin(unsigned char _address);
    void reset();
    void setVREF(bool _refstate);
    void LDAC(bool _state);
    void LDAC(unsigned char _channel, bool _state);
    void offMode(unsigned char channel, unsigned char mode);
    void offMode(unsigned char mode);
    void enable();
    void disable();
    void enableChannel(unsigned char channel);
    void disableChannel(unsigned char channel);
    void set(int _value);
    void set(unsigned char channel, int _value);
    void select(unsigned char _channel);
    void update(unsigned char _channel, int _value);
    void clrMode(int _value);
    unsigned char DAC;
    unsigned int readChan(unsigned char _command);
    unsigned int readDAC(unsigned char _command);

    // deprecated, kept for backwards compatibility
    void init();
    void enable(unsigned char state);
    void enable(unsigned char channel, unsigned char state);

  private:
    int dac7678_address;
    unsigned char off_mode[8];
    unsigned char LDAC_reg = 0xFF;
    void transmit(unsigned char _command, unsigned char _msdb, unsigned char _lsdb);

};

#endif
