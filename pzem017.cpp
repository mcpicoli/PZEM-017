#include "PZEM017.h"
#include <stdio.h>

#define HOLDING_REGISTER_VOLTAGE                0x0000
#define HOLDING_REGISTER_CURRENT                0x0001
#define HOLDING_REGISTER_POWER_LOW              0x0002
#define HOLDING_REGISTER_POWER_HIGH             0x0003
#define HOLDING_REGISTER_ENERGY_LOW             0x0004
#define HOLDING_REGISTER_ENERGY_HIGH            0x0005
#define HOLDING_REGISTER_ALARM_OVERVOLTAGE      0x0006
#define HOLDING_REGISTER_ALARM_UNDERVOLTAGE     0x0007

#define WRITE_REGISTER_ALARM_OVERVOLTAGE        0x0000
#define WRITE_REGISTER_ALARM_UNDERVOLTAGE       0x0001
#define WRITE_REGISTER_SLAVE_ADDRESS            0x0002
#define WRITE_REGISTER_CURRENT_RANGE            0x0003

#define COMMAND_RESET_ENERGY                    0x42

// Not implemented
#define COMMAND_CALIBRATION                     0x41

PZEM017::PZEM017(){};

void PZEM017::begin(Stream &stream, uint8_t slaveAddress)
{
  this->_node.begin(slaveAddress, stream);
  this->_slaveAddress = slaveAddress;
}

void PZEM017::begin(ModbusMaster &node, uint8_t slaveAddress)
{
  this->_node = node;
  this->_node.setSlaveAddress(slaveAddress);
  this->_slaveAddress = slaveAddress;
}

float PZEM017::voltage(bool forceNewReading)
{
  if (forceNewReading)
  {
    _lastRead+= _updateInterval;
  }

  if (!updateValues())
  {
    return NAN;
  }

  return _currentValues.voltage;
}

float PZEM017::current(bool forceNewReading)
{
  if (forceNewReading)
  {
    _lastRead+= _updateInterval;
  }

  if (!updateValues())
  {
    return NAN;
  }

  return _currentValues.current;
}

float PZEM017::power(bool forceNewReading)
{
  if (forceNewReading)
  {
    _lastRead+= _updateInterval;
  }

  if (!updateValues())
  {
    return NAN;
  }

  return _currentValues.power;
}

float PZEM017::energy(bool forceNewReading)
{
  if (forceNewReading)
  {
    _lastRead+= _updateInterval;
  }

  if (!updateValues())
  {
    return NAN;
  }

  return _currentValues.energy;
}

bool PZEM017::getUndervoltageAlarmStatus(bool * alarm, bool forceNewReading)
{
  if (forceNewReading)
  {
    _lastRead+= _updateInterval;
  }

  bool result = updateValues();
  if (result)
  {
    *alarm = _currentValues.underVoltageAlarm;
  }

  return result;
}

bool PZEM017::getOvervoltageAlarmStatus(bool * alarm, bool forceNewReading)
{
  if (forceNewReading)
  {
    _lastRead+= _updateInterval;
  }

  bool result = updateValues();
  if (result)
  {
    *alarm = _currentValues.overVoltageAlarm;
  }

  return result;
}

uint8_t PZEM017::getLastOperationResult()
{
  return _lastResult;
}

String PZEM017::getLastOperationResultString()
{
	return _node.getOperationResultString(_lastResult);
}

uint8_t PZEM017::getAddress()
{
  return _slaveAddress;
}

bool PZEM017::updateValues()
{
  // If the last measurement is within the update interval, use the previous values
  if (_lastRead + _updateInterval > millis())
  {
    // But don't update the last succesful read time
    return true;
  }
  
  // Read 6 registers from the slave starting at register 0x0000
  uint8_t result = _node.readInputRegisters(0x0000, 7);

  // Even if successful.
  _lastResult = result;

  if (result != _node.ku8MBSuccess)
  {
    // A failure has occurred, nothing was (succesfully) read, so the last read time is NOT updated.
    return false;
  }

  // Measurements
  _currentValues.voltage = _node.getResponseBuffer(HOLDING_REGISTER_VOLTAGE) / 100.0;
  _currentValues.current = _node.getResponseBuffer(HOLDING_REGISTER_CURRENT) / 100.0;
  _currentValues.power = (_node.getResponseBuffer(HOLDING_REGISTER_POWER_LOW) | (_node.getResponseBuffer(HOLDING_REGISTER_POWER_HIGH) << 8)) / 10.0;
  _currentValues.energy = _node.getResponseBuffer(HOLDING_REGISTER_ENERGY_LOW) | (_node.getResponseBuffer(HOLDING_REGISTER_ENERGY_HIGH) << 8);

  // Alarms
  // true if alarm is set
  _currentValues.overVoltageAlarm = (_node.getResponseBuffer(HOLDING_REGISTER_ALARM_OVERVOLTAGE) == 0xFFFF);
  _currentValues.underVoltageAlarm = (_node.getResponseBuffer(HOLDING_REGISTER_ALARM_UNDERVOLTAGE) == 0xFFFF);
  
  _lastRead = millis();
  return true;
}

void PZEM017::setUpdateInterval(uint32_t updateInterval)
{
  _updateInterval = updateInterval;
  _lastRead = millis();
}

bool PZEM017::resetEnergy()
{
  // No data expected in the response.
  // However, an exception will still return an exception code.
  uint8_t result = _node.arbitraryCommandNoParameters(COMMAND_RESET_ENERGY, 4);
  _lastResult= result;

  if (result != _node.ku8MBSuccess)
  {
    return false;
  }

  _currentValues.energy = 0;
  return true;
}

bool PZEM017::setSlaveAddress(uint8_t newAddress)
{
  if (newAddress < 0x01 || newAddress > 0xf7)
  {
    _lastResult = _node.ku8MBInvalidSlaveID;
    return false;
  }
  
  _slaveAddress = newAddress;
  return true;
}

bool PZEM017::setAddress(uint16_t newAddress)
{
  if (newAddress < 0x0001 || newAddress > 0x00f7)
  {
    _lastResult = _node.ku8MBInvalidSlaveID;
    return false;
  }
  
  uint8_t result = _node.writeSingleRegister(WRITE_REGISTER_SLAVE_ADDRESS, newAddress);
  _lastResult= result;

  return (result == _node.ku8MBSuccess);
}

bool PZEM017::setCurrentRange(uint16_t range)
{
  uint8_t result = _node.writeSingleRegister(WRITE_REGISTER_CURRENT_RANGE, range);
  _lastResult= result;

  return (result == _node.ku8MBSuccess);
}

uint8_t PZEM017::getCurrentRange()
{
	uint8_t result = _node.readHoldingRegisters(WRITE_REGISTER_CURRENT_RANGE, 1);
	_lastResult = result;
	
	return _node.getResponseBuffer(0);
}

bool PZEM017::setUndervoltageAlarm(float volts)
{
  // Range from the device's documentation.
  if (volts < 1 || volts > 350)
  {
    _lastResult = _node.ku8MBIllegalDataValue;
    return false;
  }
  
  uint8_t result = _node.writeSingleRegister(WRITE_REGISTER_ALARM_UNDERVOLTAGE, (uint16_t)volts);
  _lastResult= result;

  return (result == _node.ku8MBSuccess);
}

bool PZEM017::setOvervoltageAlarm(float volts)
{
  // Range from the device's documentation.
  if (volts < 5 || volts > 350)
  {
    _lastResult = _node.ku8MBIllegalDataValue;
    return false;
  }
  
  uint8_t result = _node.writeSingleRegister(WRITE_REGISTER_ALARM_OVERVOLTAGE, (uint16_t)volts);
  _lastResult= result;

  return (result == _node.ku8MBSuccess);
}
