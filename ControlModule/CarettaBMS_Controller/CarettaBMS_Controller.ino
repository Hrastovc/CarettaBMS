#include "CarettaBMS_CMD.h"

class CarettaBMS
{
  private:
    Stream *pStream;
    HardwareSerial *pHWserial;
    uint8_t delayRestart = 0x00;
    
    uint8_t nbDelay(uint16_t timeout)
    {
      static uint16_t delayEndCnt = 0x0000;
      
      if(delayRestart) delayEndCnt = (uint16_t)millis() + timeout;
      delayRestart = (delayEndCnt < millis());
      return delayRestart;
    }

    void readAlarmCodes(uint8_t timeout)
    {
      /* Dummy read command code. */
      pStream->read();
      
      uint8_t alarmSource = pStream->read();

      do
      {
        while(!pStream->available());
        /* Check the response */
        uint8_t ch = pStream->read();
        if()
      }
      while(!nbDelay(1000));
    }

    uint8_t waitResponse(uint16_t timeout)
    {
      do
      {
        if(pStream->available())
        {
          uint8_t ch = pStream->peek();
          if(ch == CMD_ALARM_CODES)
          {
            /* Reset Delay function. */
            delayStarted = 0x00;
            readAlarmCodes(timeout);
            return false;
          }
          else return true;
        }
      }
      while(!nbDelay(timeout));
      /* Nothing received. */
      return false;
    }
    
  public:
    uint16_t cmdRespTimeout = 1000;
    
    CarettaBMS::CarettaBMS(HardwareSerial &p)
    {
      if(p == NULL) while(1);
      pHWserial = &p;
      pStream = (Stream *)&p; 
    }
    
    void begin(void)
    {
      pHWserial->begin(9600, SERIAL_8E2);
    }

    void write(uint8_t data)
    {
      pStream->write(data);
    }
    
    uint8_t sleep(void)
    {
      pStream->write(CMD_GOTO_SLEEP);
      /* Check if response was received. */
      if(pStream->getResponse(cmdRespTimeout))
      {
        /* Check the response */
        if(pStream->read() == RESP_OK) return true;
        else return false;
      }
      else return false;
    }
    
};

/* For reference:
 *  
 *  Using HW serial example:
 *   CarettaBMS myBMS(Serial);
 *   or
 *   CarettaBMS myBMS(Serial2);
 *  
 *  Using SW serial example:
 *   SoftwareSerial softSerial(2, 3);
 *   CarettaBMS newBMS(softSerial);
 */

CarettaBMS myBMS(Serial);

void setup()
{
  myBMS.begin();
  
  /* For now we are using just communication.
   *  
   * TODO in setup:
   *                        Arduino pin number (A0, A1, ...), offset when current zero, scale factor in mA/V
   * myBMS.AddCurrentSensor(                  uint8_t pinNum,          uint16_t offset,        uint16_t gain);
   */
}

void loop()
{
  /* Write exposed just now for debug. */
  myBMS.write(0x40);
  delay(1000);
}
