#include <SPI.h>
#include <MySensor.h>  

 #define NDEBUG                        // enable local debugging information

#define NODE_ID 193 



#define DIGITAL_INPUT_SENSOR 3  // The digital input you attached your light sensor.  (Only 2 and 3 generates interrupt!)
#define PULSE_FACTOR 2000       // Nummber of blinks per KWH of your meeter
#define SLEEP_MODE false        // Watt-value can only be reported when sleep mode is false.
#define MAX_WATT 10000          // Max watt value to report. This filetrs outliers.
#define INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)


#define CHILD_ID 								1              // Id of the sensor child
#define CHILD_ID_WATT 							2              // Id of the sensor child
#define CHILD_ID_KWH 							3              // Id of the sensor child

#define REBOOT_CHILD_ID                       100
#define RECHECK_SENSOR_VALUES                 101 

/*****************************************************************************************************/
/*                               				Common settings									      */
/******************************************************************************************************/
#define RADIO_RESET_DELAY_TIME 50 //Задержка между сообщениями
#define MESSAGE_ACK_RETRY_COUNT 5  //количество попыток отсылки сообщения с запросом подтверждения
#define DATASEND_DELAY  10

boolean gotAck=false; //подтверждение от гейта о получении сообщения 
int iCount = MESSAGE_ACK_RETRY_COUNT;
unsigned long SEND_FREQUENCY = 20000; // Minimum time between send (in milliseconds). We don't wnat to spam the gateway.


MySensor gw;
double ppwh = ((double)PULSE_FACTOR)/1000; // Pulses per watt hour
boolean pcReceived = false;
volatile unsigned long pulseCount = 0;   
volatile unsigned long lastBlink = 0;
volatile unsigned long watt = 0;
unsigned long oldPulseCount = 0;   
unsigned long oldWatt = 0;
double oldKwh;
unsigned long lastSend;
boolean boolRecheckSensorValues = false;

MyMessage wattMsg(CHILD_ID_WATT,V_WATT);
MyMessage kwhMsg(CHILD_ID_KWH,V_KWH);
MyMessage pcMsg(CHILD_ID,V_VAR1);


void setup()  
{  
  pinMode(3, INPUT);

  gw.begin(incomingMessage, NODE_ID, false);
  gw.wait(RADIO_RESET_DELAY_TIME);

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Energy Meter", "1.0");
  gw.wait(RADIO_RESET_DELAY_TIME);

  // Register this device as power sensor
  gw.present(CHILD_ID_WATT, S_POWER);
  gw.wait(RADIO_RESET_DELAY_TIME);

  // Register this device as power sensor
  gw.present(CHILD_ID_KWH, S_POWER);
  gw.wait(RADIO_RESET_DELAY_TIME);

  // Fetch last known pulse count value from gw
  gw.request(CHILD_ID, V_VAR1);
  gw.wait(RADIO_RESET_DELAY_TIME);

  gw.request(REBOOT_CHILD_ID, S_BINARY);
  gw.wait(RADIO_RESET_DELAY_TIME);


  gw.request(RECHECK_SENSOR_VALUES, S_LIGHT);
  gw.wait(RADIO_RESET_DELAY_TIME);

  attachInterrupt(INTERRUPT, onPulse, RISING);
  lastSend=millis();

    //Enable watchdog timer
  	wdt_enable(WDTO_8S);
}


void loop()     
{ 
  gw.process();
  unsigned long now = millis();
  // Only send values at a maximum frequency or woken up from sleep
  bool sendTime = now - lastSend > SEND_FREQUENCY;
  if (pcReceived && (SLEEP_MODE || sendTime)) {
    // New watt value has been calculated  
    if (!SLEEP_MODE && watt != oldWatt) {
      // Check that we dont get unresonable large watt value. 
      // could hapen when long wraps or false interrupt triggered
      if (watt<((unsigned long)MAX_WATT)) {
 
            //Отсылаем состояние сенсора с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
                   // Send in the new temperature                  
                   gw.send(wattMsg.set(watt), true);  // Send watt value to gw 
                    gw.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;
      }  

      #ifdef NDEBUG
      Serial.print("Watt:");
      Serial.println(watt);
      #endif
      oldWatt = watt;
    }
  
    // Pulse cout has changed
    if (pulseCount != oldPulseCount) {
            //Отсылаем состояние сенсора с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
                   // Send in the new temperature                  
                   gw.send(pcMsg.set(pulseCount), true);  // Send pulse count value to gw 
                    gw.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;

      double kwh = ((double)pulseCount/((double)PULSE_FACTOR));     
      oldPulseCount = pulseCount;
      if (kwh != oldKwh) {
            //Отсылаем состояние сенсора с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
                   // Send in the new temperature                  
                   gw.send(kwhMsg.set(kwh, 4), true);  // Send kwh value to gw 
                    gw.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;

        oldKwh = kwh;
      }
    }    
    lastSend = now;
  } else if (sendTime && !pcReceived) {
    // No count received. Try requesting it again
    gw.request(CHILD_ID, V_VAR1);
    lastSend=now;
  }
  
  if (SLEEP_MODE) {
    gw.sleep(SEND_FREQUENCY);
  }

    //reset watchdog timer
    wdt_reset();    

}

void incomingMessage(const MyMessage &message) {
 
 if (message.isAck())
  {
    gotAck = true;
    return;
  }

  if ( message.sensor == REBOOT_CHILD_ID && message.getBool() == true && strlen(message.getString())>0 ) {
           wdt_enable(WDTO_30MS);
            while(1) {};

   }

  if (message.type==V_VAR1) {  
    pulseCount = oldPulseCount = message.getLong();
         #ifdef NDEBUG
    		Serial.print("Received last pulse count from gw:");
    		Serial.println(pulseCount);
    	 #endif	
    pcReceived = true;
    return;
  }

  if ( message.sensor == RECHECK_SENSOR_VALUES && strlen(message.getString())>0 ) {
         
       if (message.getBool() == true)
       {
          boolRecheckSensorValues = true;
		  return;

       }

   }

}

void onPulse()     
{ 
  if (!SLEEP_MODE) {
    unsigned long newBlink = micros();  
    unsigned long interval = newBlink-lastBlink;
    if (interval<10000L) { // Sometimes we get interrupt on RISING
      return;
    }
    watt = (3600000000.0 /interval) / ppwh;
    lastBlink = newBlink;
  } 
#ifdef NDEBUG  
  Serial.println(pulseCount);
#endif 
  pulseCount++;

}