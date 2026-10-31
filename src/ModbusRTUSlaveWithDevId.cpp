#include "ModbusRTUSlaveWithDevId.h"
#include <string.h>

ModbusRTUSlaveWithDevId::ModbusRTUSlaveWithDevId(Stream &serial, int dePin, int rePin)
  : _comm(serial, dePin, rePin) {}

void ModbusRTUSlaveWithDevId::configureDeviceIdentification(const __FlashStringHelper *vendorName,
                                                            const __FlashStringHelper *productCode,
                                                            const __FlashStringHelper *majorMinorRevision) {
  _objects[VENDOR_NAME] = vendorName;
  _objects[PRODUCT_CODE] = productCode;
  _objects[MAJOR_MINOR_REVISION] = majorMinorRevision;
}

void ModbusRTUSlaveWithDevId::begin(uint8_t unitId, unsigned long baud, uint32_t config) {
  if (unitId >= 1 && unitId <= 247) _unitId = unitId;
  _comm.begin(baud, config);
}

bool ModbusRTUSlaveWithDevId::poll() {
  ModbusADU adu;
  if (_comm.readAdu(adu)) return false;
  const uint8_t unitId = adu.getUnitId();
  if (unitId != _unitId && unitId != 0) return false;
  if (adu.getFunctionCode() == 0x2B && unitId != 0) _processReadDeviceId(adu);
  else processPdu(adu);
  if (unitId != 0) {
    delay(_responseDelay);
    _comm.writeAdu(adu);
  }
  return true;
}

void ModbusRTUSlaveWithDevId::_processReadDeviceId(ModbusADU &adu) {
  uint8_t *d = adu.data;
  if (!_objects[VENDOR_NAME] || d[0] != 0x0E) return adu.prepareExceptionResponse(1);
  if (adu.getDataLen() != 3) return adu.prepareExceptionResponse(3);
  const uint8_t code = d[1], objectId = d[2];
  if (code < 1 || code > 4) return adu.prepareExceptionResponse(3);

  const bool known = objectId < NUM_OBJECTS && _objects[objectId];
  if (code == 4 && !known) return adu.prepareExceptionResponse(2);

  bool regular = false;
  for (uint8_t id = VENDOR_URL; id < NUM_OBJECTS; id++) if (_objects[id]) regular = true;
  d[2] = regular ? 0x82 : 0x81;   // conformity level
  d[3] = 0;             // More Follows
  d[4] = 0;             // Next Object ID
  d[5] = 0;             // Number of objects
  uint16_t len = 6;     // d[0] (MEI) and d[1] (code) are echoed as-is

  if (code == 4) {
    _put(d, len, objectId);
  } else {
    const uint8_t last = code == 1 ? MAJOR_MINOR_REVISION : USER_APPLICATION_NAME;
    uint8_t id = VENDOR_NAME;   // restart at the beginning unless objectId is valid here
    if (known && objectId <= last) id = objectId;
    for (; id <= last; id++) {
      if (_objects[id] && !_put(d, len, id)) { d[3] = 0xFF; d[4] = id; break; }
    }
  }
  adu.setDataLen(len);
}

bool ModbusRTUSlaveWithDevId::_put(uint8_t *d, uint16_t &len, uint8_t id) {
  const char *s = reinterpret_cast<const char *>(_objects[id]);
  uint16_t n = strlen_P(s);
  if (n > MAX_OBJECT_LEN) n = MAX_OBJECT_LEN;   // so the first object in a response always fits
  if (len + 2 + n > MAX_DATA_LEN) return false;
  d[len++] = id;
  d[len++] = n;
  memcpy_P(d + len, s, n);
  len += n;
  d[5]++;
  return true;
}
