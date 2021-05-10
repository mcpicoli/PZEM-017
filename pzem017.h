#ifndef PZEM017_h
#define PZEM017_h

#ifndef ModbusMaster_h
	// Important: Requires a version that has support for dinamically changing the slave address (setSlaveAddress() method)
	// Important: Requires a version that has support for sending arbitrary commands without any parameters - required for the reset command - (arbitraryCommandNoParameters() method)
  #include <ModbusMaster.h>
#endif

#define PZEM_DEFAULT_ADDR   0xF8

#define PZEM017_SHUNT_50A   0x0001
#define PZEM017_SHUNT_100A  0x0000
#define PZEM017_SHUNT_200A  0x0002
#define PZEM017_SHUNT_300A  0x0003

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

/*
 * May be used with PZEM003 sensors as well, but some functions like setting the current range won't work.
 */
class PZEM017
{
  public:
    /*
     * The serial port will be (re)started at 9600bps, 8N2
     * I don't know why 8N2 was chosen since all other sensors from the same company use 8N1. Other companies use 8N1 as well.
     */
    PZEM017();
    void begin(Stream &stream, uint8_t slaveAddress = PZEM_DEFAULT_ADDR);
    void begin(ModbusMaster &node, uint8_t slaveAddress = PZEM_DEFAULT_ADDR);

    // --- Measurement --- //
    /*
     * Unit is Volts (V)
     * Resolution is 0.01V
     * Failure to read data returns NaN
     */
    float voltage(bool forceNewReading = false);
    /*
     * Unit is Amperes (A)
     * Resolution is 0.01A
     * Failure to read data returns NaN
     */
    float current(bool forceNewReading = false);
    /*
     * Unit is Watts
     * Resolution is 0.1 W
     * Failure to read data returns NaN
     */
    float power(bool forceNewReading = false);
    /*
     * Total energy consumption from the last reset.
     * Unit is Watts-hour (W.h)
     * Resolution is 1 W.h
     * Failure to read data returns NaN
     */
    float energy(bool forceNewReading = false);
    /*
     * Resets the current energy measurement.
     * This is a non-standard ModBus command.
     * Returns the operation status
     */
    bool resetEnergy();

    // --- Addressing and configuration --- //
    /*
     * Sets the current operating slave address. Allows for instance reuse in multiple-slave scenarios.
     */
    bool setSlaveAddress(uint8_t address);
    /*
     * Sets (writes) the desireds address at the current slave. If successful, the current operating slave address is updated to the address informed.
     * The address must be in the range 0x0001 to 0x00f7
     */
    bool setAddress(uint16_t address);
    /*
     * Gets the current operating slave address.
     */
    uint8_t getAddress();
    /*
     * Sets the current measurement range (based on the installed shunt) at the current slave address
     * See PZEM017_SHUNT_*
     * Returns the operation status
     */
    bool setCurrentRange(uint16_t range);
	uint8_t getCurrentRange();

    // --- Alarms --- //
    /*
     * Sets the undervoltage alarm
     * Resolution is 0.01V, range is 1 - 350V, default is 7V
     * Returns the operation status
     */
    bool setUndervoltageAlarm(float volts);
    /*
     * Sets the overvoltage alarm
     * Resolution is 0.01V, range is 5 - 350V, default is 300V
     * Returns the operation status
     */
    bool setOvervoltageAlarm(float volts);
    /*
     * Gets the current undervoltage alarm status
     * Returns the operation status
     * Changes the alarm. True indicates alarm is set.
     */
    bool getUndervoltageAlarmStatus(bool * alarm, bool forceNewReading = false);
    /*
     * Gets the current overvoltage alarm status
     * Returns the operation status
     * Changes the alarm. True indicates alarm is set.
     */
    bool getOvervoltageAlarmStatus(bool * alarm, bool forceNewReading = false);

    // --- Misc --- //
    /*
     * Gets the result code for the latest operation issued to the slave.
     * See the ModbusMaster library documentation for the possible values.
     */
    uint8_t getLastOperationResult();
	/*
	 * Gets the text string corresponding to the last operation result (as provided by the ModbusMaster library.
	*/
	String getLastOperationResultString();
    /*
     * Sets/overrides the internal update interval between readings.
     * Multiple requests for data after an update up to UPDATE_TIME milisseconds will use the previously fetched data (if any)
     * @param uint32_t The update interval in milisseconds.
     */
    void setUpdateInterval(uint32_t updateInterval);

  private:
    ModbusMaster _node;
    uint8_t _slaveAddress;

    // Latest velues read from slave
    struct
    {
      float voltage;
      float current;
      float power;
      float energy;
      bool underVoltageAlarm;
      bool overVoltageAlarm;
    } _currentValues;

    uint64_t _lastRead;
    uint64_t _updateInterval;
    uint8_t _lastResult;

    bool updateValues();
};
#endif
