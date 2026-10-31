// ModbusRTUSlaveWithDevId.h — Modbus RTU server (slave) built from CMB27's
// ModbusADU, ModbusRTUComm and ModbusSlaveLogic, plus FC 0x2B / MEI 0x0E
// "Read Device Identification". Equivalent to ModbusRTUSlave::poll() with one
// extra dispatch.
//
// This subclasses ModbusSlaveLogic rather than ModbusRTUSlave because
// ModbusRTUSlave::poll() has no hook and all of its members are private, so
// nothing in it could be reused.
//
// Supports the Basic (0x00-0x02) and Regular (0x03-0x06) objects. Extended
// objects (0x80-0xFF) are not supported. Strings live in flash (written with
// F("...")); the server keeps a pointer to each one, indexed by object ID.
// The response is built directly in the ADU buffer, so no other RAM is needed.
//
// Behaviour (Modbus Application Protocol V1.1b3, §6.21):
//   - Read Device ID codes 01/02/03 (stream) are cumulative: Regular includes
//     Basic, and Extended returns the same as Regular. Unknown or
//     out-of-category start object IDs restart at object 0.
//   - Code 04 (individual access) returns one object; unknown ID -> exception 02.
//   - Responses that would exceed one PDU are split with More Follows = 0xFF and
//     Next Object ID set. An object is never split.
//   - Conformity level is 0x82 (regular) if any Regular object is set, else
//     0x81 (basic); both with individual access.
//
// One object must fit in one response, so strings longer than 244 characters are
// truncated to 244.
//
//   modbus.configureDeviceIdentification(F("Vendor"), F("Product"), F("1.0"));
//   modbus.setProductName(F("Product name"));

#pragma once
#include <ModbusADU.h>
#include <ModbusRTUComm.h>
#include <ModbusSlaveLogic.h>

#if defined(__AVR__)
  #include <avr/pgmspace.h>
#else
  #ifndef PROGMEM
    #define PROGMEM
    #define strlen_P strlen
    #define memcpy_P memcpy
  #endif
#endif

class ModbusRTUSlaveWithDevId : public ModbusSlaveLogic {
  public:
    // Object IDs
    enum : uint8_t {
      VENDOR_NAME           = 0x00,   // Basic
      PRODUCT_CODE          = 0x01,   // Basic
      MAJOR_MINOR_REVISION  = 0x02,   // Basic
      VENDOR_URL            = 0x03,   // Regular
      PRODUCT_NAME          = 0x04,   // Regular
      MODEL_NAME            = 0x05,   // Regular
      USER_APPLICATION_NAME = 0x06,   // Regular
    };

    ModbusRTUSlaveWithDevId(Stream &serial, int dePin = -1, int rePin = -1);

    // The three mandatory Basic objects. Until this is called, FC 0x2B is
    // answered with exception 01.
    void configureDeviceIdentification(const __FlashStringHelper *vendorName,
                                       const __FlashStringHelper *productCode,
                                       const __FlashStringHelper *majorMinorRevision);

    // Optional Regular objects.
    void setVendorUrl(const __FlashStringHelper *s) { _objects[VENDOR_URL] = s; }
    void setProductName(const __FlashStringHelper *s) { _objects[PRODUCT_NAME] = s; }
    void setModelName(const __FlashStringHelper *s) { _objects[MODEL_NAME] = s; }
    void setUserApplicationName(const __FlashStringHelper *s) { _objects[USER_APPLICATION_NAME] = s; }

    void setResponseDelay(unsigned long responseDelay) { _responseDelay = responseDelay; }
    void begin(uint8_t unitId, unsigned long baud, uint32_t config = SERIAL_8N1);

    // Returns true when a valid request for this unit (or a broadcast) was handled.
    bool poll();

  private:
    static const uint16_t MAX_DATA_LEN = 252;     // 253-byte PDU minus function code
    static const uint16_t MAX_OBJECT_LEN = 244;   // MAX_DATA_LEN minus 6-byte header, object ID and length
    static const uint8_t NUM_OBJECTS = USER_APPLICATION_NAME + 1;
    ModbusRTUComm _comm;
    const __FlashStringHelper *_objects[NUM_OBJECTS] = {};   // index = object ID, 0 = not set
    uint8_t _unitId = 0;
    unsigned long _responseDelay = 0;

    void _processReadDeviceId(ModbusADU &adu);
    bool _put(uint8_t *d, uint16_t &len, uint8_t id);
};
