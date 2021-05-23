/*! @file MAX31855.cpp
 @section MAX38155cpp_intro_section Description

Arduino Library for Microchip SRAM access\n\n
See main library header file for details
*/
#include "MAX31855.h"                 // Include the header definition
MAX31855_Class::MAX31855_Class() {}   ///< Empty & unused class constructor
MAX31855_Class::~MAX31855_Class() {}  ///< Empty & unused class destructor

bool MAX31855_Class::begin(const uint8_t chipSelect, const bool reverse) {
  /*!
   @brief     Initialize library with Hardware SPI (overloaded function
   @details   When called with one parameter which represents the chip-select pin then hardware SPI
              is used, otherwise when called with 3 parameters they define the software SPI pins SPI
              to be used. Since the MAX31855 is a 1-way device, there is no practical way to check
              for a device, so a dummy read is done to see if the device is responding and that
              defines whether the function returns a true or false status
   @param[in] chipSelect Chip-Select pin number
   @param[in] reverse Option boolean switch to indicate that the wires are (intentionally) reversed
   @return    true if the device could be read, otherwise false
  */
  _reversed = reverse;      // Set to true if contacts reversed
  _cs       = chipSelect;   // Copy value for later use
  pinMode(_cs, OUTPUT);     // Make the chip select pin output
  digitalWrite(_cs, HIGH);  // High means ignore master
  readRaw();                // Try to read the raw data
  return (bool)_errorCode;  // Return error code as a boolean value
}  // of method begin()

bool MAX31855_Class::begin(const uint8_t chipSelect, const uint8_t miso, const uint8_t sck,
                           const bool reverse) {
  /*!
   @brief     Initialize library with Software SPI (overloaded function)
   @details   When called with one parameter which represents the chip-select pin then hardware SPI
              is used, otherwise when called with 3 parameters they define the software SPI pins
              SPI to be used. Since the MAX31855 is a 1-way device, there is no practical way to
              check for a device, so a dummy read is done to see if the device is responding and
              that defines whether the function returns a true or false status
   @param[in] chipSelect Chip-Select pin number
   @param[in] miso       Master-In-Slave-Out pin number
   @param[in] sck        System clock pin number
   @param[in] reverse Option boolean switch to indicate that the wires are (intentionally) reversed
   @return    true if the device could be read, otherwise false
  */
  _reversed = reverse;      // Set to true if contacts reversed
  _cs       = chipSelect;   // Store SPI Chip-Select pin
  _miso     = miso;         // Store SPI Master-in Slave-Out pin
  _sck      = sck;          // Store SPI System clock pin
  pinMode(_cs, OUTPUT);     // Make the chip select pin output
  digitalWrite(_cs, HIGH);  // High means ignore master
  pinMode(_sck, OUTPUT);    // Make system clock pin output
  pinMode(_miso, INPUT);    // Make master-in slave-out input
  readRaw();                // Read the raw data
  return !(bool)_errorCode; // Return if successful (no errors)
}  // of method begin()

uint8_t MAX31855_Class::fault() const {
  /*!
   @brief   Return the device fault state
   @details The fault state of the device from the last read attempt is stored in a private
            variable, this is returned by this call
   @return  MAX31855 fault state from the last read
  */
  return _errorCode;
}  // of method fault()

int32_t MAX31855_Class::readRaw() {
  /*!
   @brief   returns the 32 bits of raw data from the MAX31855 device
   @details Sometimes invalid readings are returned (0x7FFFFFFF) so this routine will loop several
            times until a valid reading is returned, with a maximum of READING_RETRIES attempts.
   @return  Raw temperature reading
  */
  int32_t dataBuffer = 0;
  digitalWrite(_cs, LOW);                     // Tell MAX31855 that it is active
  if (_sck == 0)                              // Hardware SPI
  {
    SPI.beginTransaction(
        SPISettings(14000000, MSBFIRST, SPI_MODE0));  // Start transaction at 14MHz MSB
    dataBuffer = SPI.transfer(0);                     // Read a byte
    dataBuffer <<= 8;                                 // Shift over left 8 bits
    dataBuffer |= SPI.transfer(0);                    // Read a byte
    dataBuffer <<= 8;                                 // Shift over left 8 bits
    dataBuffer |= SPI.transfer(0);                    // Read a byte
    dataBuffer <<= 8;                                 // Shift over left 8 bits
    dataBuffer |= SPI.transfer(0);                    // Read a byte
    SPI.endTransaction();                             // Terminate SPI transaction
  } else                                              // Software SPI
  {
    digitalWrite(_sck, LOW);                      // Toggle the system clock low
    delayMicroseconds(SPI_DELAY_MICROSECONDS);    // Give device time to respond
    for (uint8_t i = 0; i < 32; i++) {            // Loop for each bit to be read
      digitalWrite(_sck, LOW);                    // Toggle the system clock low
      delayMicroseconds(SPI_DELAY_MICROSECONDS);  // Give device time to respond
      dataBuffer <<= 1;                           // Shift over 1 bit
      if (digitalRead(_miso)) dataBuffer |= 1;    // set rightmost bit if true
      digitalWrite(_sck, HIGH);                   // Toggle the system clock high
      delayMicroseconds(SPI_DELAY_MICROSECONDS);  // Give device time to respond
    }                                             // of read each bit from software SPI bus
  }                                               // of if-then-else we are using HW SPI
  _errorCode = dataBuffer & B111;                 // Mask fault code bits
  digitalWrite(_cs, HIGH);                        // MAX31855 no longer active
  return dataBuffer;
}  // of method readRaw()

int16_t MAX31855_Class::rawToProbe(int32_t rawBuffer) {
  /*!
   @brief   returns the probe temperature given the raw reading
   @details The temperature is returned as a signed 16-bit integer. 
            As the ADC gives a 14-bit signed integer for the probe temperature,
            the 2 least significant bits should be zero. If they are
            non-zero, this is indication of an error.
            Note that putting the probe temperature as a 16-bit integer
            effectively means that the magnitude of the temperature is 4 times
            larger and should be normalized by a quarter of the resolution.
   @return  Probe Temperature as a signed 16-bit integer
  */
  int32_t dataBuffer = rawBuffer;  // Copy raw reading to working variable
  int16_t probeTemp;
  int16_t ambientTemp;
  if (dataBuffer & B111) {
    probeTemp = INT16_MAX;  // if any error bits are set then return error value
    return probeTemp;
  }
  else {
    dataBuffer = dataBuffer >> 18; // remove unused ambient values
    dataBuffer = dataBuffer << 2;  // shift 2 bits left to get align 14 bits on 16
    probeTemp = (int16_t) dataBuffer; // get value as a signed 16-bit integer
  }
  if (_reversed) // If the thermocouple pins are reversed we have to switch readings around
  {
    int32_t ambientBuffer = (rawBuffer & 0xFFFF) >> 4; // remove probe & fault values
    ambientBuffer = ambientBuffer << 4; // shift 4 bits left to get align 12 bits on 16
    ambientTemp = (int16_t) ambientBuffer; // get value as a signed 16-bit integer
    // Invert the delta temperature taking into account relative sensitivity
    probeTemp = (ambientTemp / 4 - probeTemp) + ambientTemp / 4; 
  }
  return probeTemp;
}  // of method rawToProbe()

int16_t MAX31855_Class::rawToAmbient(int32_t rawBuffer) {
  /*!
   @brief   returns the ambient temperature given the raw reading
   @details The temperature is returned as a signed 16-bit integer. 
            As the ADC gives a 12-bit signed integer for the ambient temperature,
            the 4 least significant bits should be zero. If they are
            non-zero, this is indication of an error.
            Note that putting the ambient temperature as a 16-bit integer
            effectively means that the magnitude of the temperature is 16 times
            larger and should be normalized by 1/16 of the resolution.
   @return  Ambient Temperature as a signed 16-bit integer
  */
  int32_t dataBuffer = rawBuffer;  // Copy raw reading to working variable
  if (dataBuffer & B111) {
    dataBuffer = INT16_MAX;  // if error bits set then return error
  }
  else {
    dataBuffer = (dataBuffer & 0xFFFF) >> 4; // remove probe & fault values
    dataBuffer = dataBuffer << 4;  // shift 2 bits left to get align 14 bits on 16
  } // of if we have an error
  return (int16_t) dataBuffer; // return value as a signed 16-bit integer
}  // of method readAmbient()
