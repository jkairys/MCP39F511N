#include <Arduino.h>
#include <SoftwareSerial.h>

//#define MCP_DEBUG

#ifdef MCP_DEBUG
 #define DEBUG_PRINT(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
#endif

// Configuration
#define MCP_BUFFER_LEN 64
#define MCP_TIMEOUT_MS 100

// Constants
#define MCP_HEADER_BYTE 0xA5
#define MCP_FREQUENCY_DIVISOR 1000

// Commands
#define MCP_CMD_REGISTER_READ 0x4E
#define MCP_CMD_REGISTER_WRITE 0x4D
#define MCP_CMD_SET_ADDRESS_POINTER 0x41
#define MCP_CMD_SAVE_TO_FLASH 0x53
#define MCP_CMD_READ_EEPROM 0x42
#define MCP_CMD_WRITE_EEPROM 0x50
#define MCP_CMD_ERASE_EEPROM 0x4F
#define MCP_CMD_AUTO_CALIBRATE_GAIN 0x5A
#define MCP_CMD_AUTO_CALIBRATE_Q_GAIN 0x7A
#define MCP_CMD_AUTO_CALIBRATE_F 0x76

// Registers
#define MCP_REG_INST_POINTER    0x0000 // Instruction Pointer 6.2 R u16 Address pointer for read or write commands
#define MCP_REG_SYSTEM_STATUS   0x0002 // System Status 6.3 R b16 System Status Register
#define MCP_REG_SYSTEM_VERSION  0x0004 // System Version 6.4 R u16 System version date code information for MCP39F511N, set at the Microchip factory; format YMDD
#define MCP_REG_VOLTS           0x0006 // Voltage RMS 5.4 R u16 RMS Voltage output
#define MCP_REG_FREQUENCY       0x0008 // Line Frequency 5.1.1 R u16 Line Frequency output
#define MCP_REG_PF1             0x000A // Power Factor1 5.8 R s16 Power Factor output from channel 1
#define MCP_REG_PF2             0x000C // Power Factor2 5.8 R s16 Power Factor output from channel 2
#define MCP_REG_AMPS1           0x000E // Current RMS1 5.4 R u32 RMS Current output from channel 1
#define MCP_REG_AMPS2           0x0012 // Current RMS2 5.4 R u32 RMS Current output from channel 2
#define MCP_REG_WATTS1          0x0016 // Active Power1 5.7 R u32 Active Power output from channel 1
#define MCP_REG_WATTS2          0x001A // Active Power2 5.7 R u32 Active Power output from channel 2
#define MCP_REG_VARS1           0x001E // Reactive Power1 5.9 R u32 Reactive Power output from channel 1
#define MCP_REG_VARS2           0x0022 // Reactive Power2 5.9 R u32 Reactive Power output from channel 2
#define MCP_REG_VA1             0x0026 // Apparent Power1 5.4 R u32 Apparent Power output from channel 1
#define MCP_REG_VA2             0x002A // Apparent Power2 5.4 R u32 Apparent Power output from channel 2
#define MCP_REG_IMPORT_WH1      0x002E // Import Energy Active Counter 1 5.6 R u64 Accumulator for Active Energy, Import, channel 1
#define MCP_REG_IMPORT_WH2      0x0036 // Import Energy Active Counter 2 5.6 R u64 Accumulator for Active Energy, Import, channel 2
#define MCP_REG_EXPORT_WH1      0x003E // Export Energy Active Counter 1 5.6 R u64 Accumulator for Active Energy, Export, channel 1
#define MCP_REG_EXPORT_WH2      0x0046 // Export Energy Active Counter 2 5.6 R u64 Accumulator for Active Energy, Export, channel 2
#define MCP_REG_IMPORT_VAR1     0x004E // Import Energy Reactive Counter 1 5.6 R u64 Accumulator for Reactive Energy, Import,channel 1
#define MCP_REG_IMPORT_VAR2     0x0056 // Import Energy Reactive Counter 2 5.6 R u64 Accumulator for Reactive Energy, Import,channel 2
#define MCP_REG_EXPORT_VAR1     0x005E // Export Energy Reactive Counter 1 5.6 R u64 Accumulator for Reactive Energy, Export,channel 1
#define MCP_REG_EXPORT_VAR2     0x0066 // Export Energy Reactive Counter 2 5.6 R u64 Accumulator for Reactive Energy, Export,channel 2


#define MCP_REG_FACTORY_GAINS   0x006E // Load factory default gains at startup
#define MCP_REG_GAIN_AMPS1      0x0070 // Gain factor for RMS current channel 1
#define MCP_REG_GAIN_AMPS2      0x0072 // Gain factor for RMS current channel 2
#define MCP_REG_GAIN_VOLTS      0x0074 // Gain factor for RMS voltage
#define MCP_REG_GAIN_WATTS1     0x0076 // Gain factor for active power 1
#define MCP_REG_GAIN_WATTS2     0x0078 // Gain factor for active power 2
#define MCP_REG_GAIN_VARS1      0x007A // Gain factor for reactive power 1
#define MCP_REG_GAIN_VARS2      0x007C // Gain factor for reactive power 2
#define MCP_REG_GAIN_FREQUENCY  0x007E // Gain factor for frequency

#define MCP_REG_RANGE1          0x00AE
#define MCP_REG_RANGE2          0x00BE

#define MCP_REG_DIVISOR_DIGITS1 0x009C
#define MCP_REG_DIVISOR_DIGITS2 0x009E

// Response codes
#define MCP_ACK                 0x06
#define MCP_NACK                0x15
#define MCP_CSFAIL              0x51

// Error codes
#define MCP_STATUS_RX_COMPLETE 1
#define MCP_STATUS_RX_TIMEOUT 2
#define MCP_STATUS_ERR_HEADER_BYTE 10
#define MCP_STATUS_ERR_MSG_TOO_SHORT 11
#define MCP_STATUS_ERR_MSG_BUFFER_OVERFLOW 12
#define MCP_STATUS_ERR_CSFAIL_TX 13
#define MCP_STATUS_ERR_CSFAIL_RX 14


#define MCP_SSR_AC_STATUS   1   << 12
#define MCP_SSR_SIGN_PR_CH2 1   << 7 // 1 = Q1, Q2 (positive), 0 = Q3, Q4 (negative)
#define MCP_SSR_SIGN_PA_CH2 1   << 6 // 1 = Q1, Q4 (positive), 0 = Q2, Q3 (negative)
#define MCP_SSR_SIGN_PR_CH1 1   << 5
#define MCP_SSR_SIGN_PA_CH1 1   << 4

class MCP39F511N
{
  private:
    uint8_t _msg_buffer[MCP_BUFFER_LEN];
    uint8_t _buf_count;
    uint8_t _rx_status;

    uint8_t _pin_rx;
    uint8_t _pin_tx;
    uint32_t _baud_rate;
    SoftwareSerial * _ser;

    uint16_t ssr;     // system status register
    uint16_t volts;   // volts  * 10^1

    uint32_t amps1;   // amps   * 10^3?
    uint32_t vars1;   // vars   * 10^3?
    uint32_t watts1;  // watts  * 10^3?
    uint32_t amps2;
    uint32_t vars2;
    uint32_t watts2;

    uint16_t frequency; // frequency * 10^3
    uint16_t pf1;
    uint16_t pf2;


    uint16_t gain_amps1;
    uint16_t gain_amps2;
    uint16_t gain_volts;
    uint16_t gain_watts1;
    uint16_t gain_watts2;
    uint16_t gain_vars1;
    uint16_t gain_vars2;
    uint16_t gain_frequency;

    uint8_t range_volts;
    uint8_t range_amps1;
    uint8_t range_power1;
    uint8_t range_amps2;
    uint8_t range_power2;

    uint16_t divisor1;
    uint16_t divisor2;

    // output register precision (i.e. how many decimals...)
    uint8_t _precisionVolts;
    uint8_t _precisionAmps1;
    uint8_t _precisionPower1;
    uint8_t _precisionAmps2;
    uint8_t _precisionPower2;

    // start a new outgoing message
    void _start_tx_frame();
    // complete outgoing message (byte count, checksum)
    void _complete_tx_frame();

    // send command
    void _send_command(uint8_t cmd);

    // receive a byte into the buffer
    uint8_t _rx_byte(uint8_t b);
    uint8_t _set_rx_status(uint8_t status);

    // Calculate the checksum of data in the array, up to len bytes of data
    uint8_t _checksum(uint8_t * data, uint8_t len);
    uint8_t _processBuffer();
    void _clearBuffer();
    uint8_t _receiveResponse();
    uint8_t _readBytes(uint16_t addr, uint8_t len);

    uint16_t read_int(uint8_t addr);
    uint32_t read_long(uint8_t addr);


    void _setAddressPointer(uint16_t addr);
    void _registerRead(uint8_t numBytes);
    void _buf_append(uint8_t b);

  public:
    MCP39F511N(uint8_t pin_rx, uint8_t pin_tx, uint32_t baud_rate);

    void readPower();
    void readRange();
    void readGain();

    void readConfig();

    float getFrequency();
    float getVolts();

    float getAmps1();
    float getWatts1();
    float getVars1();
    float getVA1();

    float getAmps2();
    float getWatts2();
    float getVars2();
    float getVA2();


    void setPrecisionVolts(uint8_t decimals);
    void setPrecisionAmps1(uint8_t decimals);
    void setPrecisionPower1(uint8_t decimals);
    void setPrecisionAmps2(uint8_t decimals);
    void setPrecisionPower2(uint8_t decimals);

    //void read();
    //void loop();
    void debug();


};
