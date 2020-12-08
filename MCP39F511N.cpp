#include <Arduino.h>
#include "MCP39F511N.h"

// Return the Nth byte from a long (0 indexed)
uint8_t extractByte(uint32_t data, uint8_t n){
  uint32_t msk = 0xFF << ((n)*8);
  uint32_t tmp = data & msk;
  tmp = tmp >> ((n)*8);
  return tmp;
}

uint16_t MCP39F511N::read_int(uint8_t addr){
  uint16_t offset = 2;
  return word(_msg_buffer[addr + offset + 1], _msg_buffer[addr + offset + 0]);
}

uint32_t MCP39F511N::read_long(uint8_t addr){
  uint32_t tmp = 0;
  uint32_t ret = 0;
  uint16_t offset = 2;
  tmp += _msg_buffer[addr+3 + offset];
  tmp = tmp << 24;
  ret += tmp;
  tmp = _msg_buffer[addr+2 + offset];
  tmp = tmp << 16;
  ret += tmp;
  tmp = _msg_buffer[addr+1 + offset];
  tmp = tmp << 8;
  ret += tmp;
  ret += _msg_buffer[addr+0 + offset];
  return ret;
}

MCP39F511N::MCP39F511N(Stream* ser){
  this->_ser = ser;
}

void MCP39F511N::_clearBuffer(){
  //DEBUG_PRINT("_clearBuffer");
  _rx_status = 0;
  _buf_count = 0;
}

// Add an extra byte to the end of the buffer
void MCP39F511N::_buf_append(uint8_t b){
  // prevent overflow
  if(_buf_count >= MCP_BUFFER_LEN) return;
  // increase buffer length counter and add this byte to the end
  _buf_count++;
  _msg_buffer[_buf_count-1] = b;
}

void MCP39F511N::_start_tx_frame(){
  //DEBUG_PRINT("_start_tx_frame");
  // clear the buffer
  for(uint8_t i=0; i<MCP_BUFFER_LEN; i++) _msg_buffer[i] = 0;
  // set the first character
  _buf_count = 0;
  _buf_append(MCP_HEADER_BYTE);
  // set the counter to *2* bytes, we need to reserve byte 2 for msg len
  _buf_count = 2;
}

// Append the checksum and send the frame
void MCP39F511N::_complete_tx_frame(){
  //DEBUG_PRINT("_complete_tx_frame");
  // Second byte is the message len
  _msg_buffer[1] = _buf_count +1;
  // Calculate and include our checksum
  _buf_append(_checksum(_msg_buffer, _buf_count));
  // write data to the device
  for(uint8_t i=0; i < _buf_count; i++){
    _ser->write(_msg_buffer[i]);
    //DEBUG_PRINT("TX: "+String(_msg_buffer[i], HEX));
  }
}

void MCP39F511N::debug(){
  switch(_rx_status){
    case MCP_STATUS_ERR_MSG_TOO_SHORT:
      DEBUG_PRINT("Response too short ("+String(_buf_count, DEC)+" chars)");
      break;
    default:
      DEBUG_PRINT("rx_status = "+String(_rx_status, DEC));
      break;
  }

}

uint8_t MCP39F511N::_processBuffer(){
  // check first character is a start char
  if(_msg_buffer[0] != MCP_ACK) return _set_rx_status(MCP_STATUS_ERR_HEADER_BYTE);

  // check the message length
  if(_buf_count < 3){
    return _set_rx_status(MCP_STATUS_ERR_MSG_TOO_SHORT);
  }

  // check if MCP retuend CSFAIL
  if(_msg_buffer[0] == MCP_CSFAIL) return _set_rx_status(MCP_STATUS_ERR_CSFAIL_TX);

  // check the checksum (excluding the received checksum)
  uint8_t my_checksum = _checksum(_msg_buffer, _buf_count - 2);
  if(_msg_buffer[_buf_count-1] != my_checksum) return _set_rx_status(MCP_STATUS_ERR_CSFAIL_RX);

  // if we made it this far, msg received
  return 0;
}

// receive a byte into the buffer
uint8_t MCP39F511N::_rx_byte(uint8_t b){
  //DEBUG_PRINT("RX: "+String(b, HEX));
  if(_buf_count >= MCP_BUFFER_LEN){
    return _processBuffer();
  }
  _buf_append(b);
}

// Calculate the checksum of data in the array, up to len bytes of data
uint8_t MCP39F511N::_checksum(uint8_t * data, uint8_t len){
  uint32_t cs = 0;
  uint32_t tmp = 0;
  //DEBUG_PRINT("len is "+String(len));
  for(uint8_t i=0; i< len; i++){
    tmp = data[i];
    cs = cs + tmp;
    //DEBUG_PRINT("+"+String(tmp, DEC));
  }
  //cs = cs % 256;
  //DEBUG_PRINT("_checksum ("+String(len,DEC)+") = "+String(cs, HEX));
  //DEBUG_PRINT("cs: " + String(cs));
  return (cs % 256) & 0xFF;
}


void MCP39F511N::readConfig(){

  uint8_t result;

  _start_tx_frame();
  result = _readBytes(MCP_REG_RANGE1, 4);
  if(result) return;
  uint32_t buf;
  uint32_t tmp;
  buf = read_long(0);

  /*
  tmp = buf;
  tmp = tmp & 0xFF;
  this->range_volts = tmp;

  tmp = buf;
  tmp = tmp >> 8;
  tmp = tmp & 0xFF;
  this->range_amps1 = tmp ;

  tmp = buf;
  tmp = tmp >> 16;
  tmp = tmp & 0xFF;
  this->range_power1 = tmp ;
  */
  this->range_volts = extractByte(buf,0);
  this->range_amps1 = extractByte(buf,1);
  this->range_power1 = extractByte(buf,2);

  _start_tx_frame();
  result = _readBytes(MCP_REG_RANGE2, 4);
  if(result) return;
  buf = read_long(0);

  /*
  tmp = buf;
  tmp = tmp >> 8;
  tmp = tmp & 0xFF;
  this->range_amps2 = tmp ;

  tmp = buf;
  tmp = tmp >> 16;
  this->range_power2 = tmp & 0xFF;
  */
  this->range_amps2 = extractByte(buf, 1);
  this->range_power2 = extractByte(buf, 2);

  _start_tx_frame();
  result = _readBytes(MCP_REG_DIVISOR_DIGITS1, 4);
  if(result) return;
  this->divisor1 = read_int(0);
  this->divisor2 = read_int(2);


  DEBUG_PRINT("***\nRanges:\nv: " + String(this->range_volts, DEC)
    + "\nI1: " + String(this->range_amps1, DEC)
    + "\nS1: " + String(this->range_power1, DEC)
    + "\nI2: " + String(this->range_amps2, DEC)
    + "\nS2: " + String(this->range_power2, DEC)
    + "\ndiv1: " + String(this->divisor1, DEC)
    + "\ndiv2: " + String(this->divisor2, DEC)
  );

  // Read gain registers
  _start_tx_frame();

  result = _readBytes(MCP_REG_GAIN_AMPS1, 16);
  // gain_amps1     (2b) 0-1
  // gain_amps2     (2b) 2-3
  // gain_volts     (2b) 4-5
  // gain_watts1    (2b) 6-7
  // gain_watts2    (2b) 8-9
  // gain_vars1     (2b) 10-11
  // gain_vars2     (2b) 12-13
  this->gain_vars2      = read_int(12);
  this->gain_frequency  = read_int(14);

  DEBUG_PRINT("***\nGains:\nI1: " + String(this->gain_amps1, DEC)

    + "\nW1: " + String(this->gain_watts1, DEC)
    + "\nVAR1: " + String(this->gain_vars1, DEC)
    + "\nI1: " + String(this->gain_amps1, DEC)
    + "\nW2: " + String(this->gain_watts2, DEC)
    + "\nVAR2: " + String(this->gain_vars2, DEC)
    + "\nV: " + String(this->gain_volts, DEC)
    + "\nF: " + String(this->gain_frequency, DEC)
  );

}



void MCP39F511N::readPower(){
  // annoyingly, we need to read the sign of active/reactive power
  // from the system status Register
  _start_tx_frame();
  uint8_t result;
  result = _readBytes(MCP_REG_SYSTEM_STATUS, 4);
  if(result) return;

  this->ssr = read_int(0);
  int16_t sign_active_1 = (ssr && MCP_SSR_SIGN_PA_CH1) ? 1 : -1;
  int16_t sign_reactive_1 = (ssr && MCP_SSR_SIGN_PR_CH1) ? 1 : -1;
  int16_t sign_active_2 = (ssr && MCP_SSR_SIGN_PA_CH2) ? 1 : -1;
  int16_t sign_reactive_2 = (ssr && MCP_SSR_SIGN_PR_CH2) ? 1 : -1;




  _start_tx_frame();
  result = _readBytes(MCP_REG_VOLTS, 32);
  // volts  (2b) 0-1
  // freq   (2b) 2-3
  // pf1    (2b) 4-5
  // pf2    (2b) 6-7
  // amps1  (4b) 8-11
  // amps2  (4b) 12-15
  // watts1 (4b) 16-19
  // watts2 (4b) 20-23
  // vars1  (4b) 24-27
  // vars2  (4b) 28-31

  // if something went wrong, there will be an error code
  if(result) return;
  // no error code, process the received data
  this->volts = read_int(0);
  this->frequency = read_int(2);
  this->pf1 = read_int(4);
  this->pf2 = read_int(6);
  this->amps1 = read_long(8);
  this->amps2 = read_long(12);
  this->watts1 = read_long(16);
  this->watts2 = read_long(20);
  this->vars1 = read_long(24);
  this->vars2 = read_long(28);
  /*
  DEBUG_PRINT("***");
  DEBUG_PRINT("Reads:");
  DEBUG_PRINT("V: " + String(this->volts, DEC)
  + "\nF: " + String(this->frequency, DEC)
    + "\nI1: " + String(this->amps1, DEC)
    + "\nW1: " + String(this->watts1, DEC)
    + "\nVAR1: " + String(this->vars1, DEC)
  );
  */
}

uint8_t MCP39F511N::_receiveResponse(){
  //DEBUG_PRINT("_receiveResponse...");
  uint32_t t_timeout = millis() + MCP_TIMEOUT_MS;
  uint8_t result = _processBuffer();
  bool accellerated = false;

  while(millis() < t_timeout && result != MCP_STATUS_RX_COMPLETE){
    if(this->_ser->available()){
      this->_rx_byte(this->_ser->read());
      if(_buf_count >= MCP_BUFFER_LEN){
        DEBUG_PRINT("buffer overflow");
        return _set_rx_status(MCP_STATUS_ERR_MSG_BUFFER_OVERFLOW);
      }
    }else{
      // message may have finished. give it 20ms to wrap up.
      if(!accellerated && _buf_count){
        t_timeout = millis() + 20;
        //DEBUG_PRINT("quiet - accellerating timeout");
        accellerated = true;
      }
    }
    result = _processBuffer();
  }
  //DEBUG_PRINT("Result = " + String(result, DEC));

  // check if we're here because of a timeout
  if(millis() >= t_timeout && !_buf_count){
    DEBUG_PRINT("timeout");
    return _set_rx_status(MCP_STATUS_RX_TIMEOUT);
  }
  // All clear
  //DEBUG_PRINT("receive complete");
  return _set_rx_status(0);
}

// Read 'len' bytes from the device, starting at address 'addr'
// These will be read in to the msg_buffer
uint8_t MCP39F511N::_readBytes(uint16_t addr, uint8_t len){
  //DEBUG_PRINT("_readBytes: " + String(addr, HEX) + " - " + String(len) + " bytes");
  _setAddressPointer(addr);
  _registerRead(len);
  _complete_tx_frame();
  _clearBuffer();
  // fill our RX buffer or timeout
  return _receiveResponse();
}

void MCP39F511N::setPrecisionVolts(uint8_t decimals){ this->_precisionVolts = decimals; }
void MCP39F511N::setPrecisionAmps1(uint8_t decimals){ this->_precisionAmps1 = decimals; }
void MCP39F511N::setPrecisionPower1(uint8_t decimals){ this->_precisionPower1 = decimals; }
void MCP39F511N::setPrecisionAmps2(uint8_t decimals){ this->_precisionAmps2 = decimals; }
void MCP39F511N::setPrecisionPower2(uint8_t decimals){ this->_precisionPower2 = decimals; }

void MCP39F511N::_setAddressPointer(uint16_t addr){
  //DEBUG_PRINT("_setAddressPointer: " + String(addr, HEX));

  _buf_append(MCP_CMD_SET_ADDRESS_POINTER);
  _buf_append(highByte(addr));
  _buf_append(lowByte(addr));
}

void MCP39F511N::_registerRead(uint8_t numBytes){
  //DEBUG_PRINT("_registerRead " + String(numBytes, DEC) + " bytes");
  _buf_append(MCP_CMD_REGISTER_READ);
  _buf_append(numBytes);
}

uint8_t MCP39F511N::_set_rx_status(uint8_t status){
  _rx_status = status;
  return _rx_status;
}


float MCP39F511N::getFrequency(){ return ((float)this->frequency) / MCP_FREQUENCY_DIVISOR; }
float MCP39F511N::getVolts()    { return ((float)this->volts)   / ((float)pow(10, this->_precisionVolts)) ; }
float MCP39F511N::getAmps1()    { return ((float)this->amps1)   / ((float)pow(10, this->_precisionAmps2)) * ((ssr && MCP_SSR_SIGN_PA_CH1) ? 1 : -1); }
float MCP39F511N::getWatts1()   { return ((float)this->watts1)  / ((float)pow(10, this->_precisionPower2)) * ((ssr && MCP_SSR_SIGN_PA_CH1) ? 1 : -1); }
float MCP39F511N::getVars1()    { return ((float)this->vars1)   / ((float)pow(10, this->_precisionPower2)) * ((ssr && MCP_SSR_SIGN_PR_CH1) ? 1 : -1); }
float MCP39F511N::getAmps2()    { return ((float)this->amps2)   / ((float)pow(10, this->_precisionAmps2)) * ((ssr && MCP_SSR_SIGN_PA_CH2) ? 1 : -1); }     // 4
float MCP39F511N::getWatts2()   { return ((float)this->watts2)  / ((float)pow(10, this->_precisionPower2)) * ((ssr && MCP_SSR_SIGN_PA_CH2) ? 1 : -1); }    // 2
float MCP39F511N::getVars2()    { return ((float)this->vars2)   / ((float)pow(10, this->_precisionPower2)) * ((ssr && MCP_SSR_SIGN_PR_CH2) ? 1 : -1); }    // 2
