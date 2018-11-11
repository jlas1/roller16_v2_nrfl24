  /*
 * 
 * 8 roller controller
 * 
 */

// define shift register
//#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_RF24_PA_LEVEL RF24_PA_LOW
//#define MY_RADIO_RFM69
//#define MY_IS_RFM69HWw
//#define MY_REPEATER_NODE
#define MY_NODE_ID AUTO
//#define MY_NODE_ID 1

#define MY_RX_MESSAGE_BUFFER_FEATURE
#define MY_RF24_IRQ_PIN 2

#include <Shifty.h>
#include <SPI.h>
#include <MySensors.h> 
#include <avr/wdt.h>
#include <TimeLib.h> 

// Enable debug prints to serial monitor
Shifty interface;

#define LED_POWERUP_COUNT 6
#define LED_DELAY 200

//Sketch information
#define SKETCH_INFO         "8 Rollers controller"
#define SKETCH_VERSION      "0.3"
#define ROLLER_ID_INFO      "Roller Motor"

//Auto-reset
#define MESSAGES_FAILED_REBOOT 20

//Configurable ACK Timeout
#define ACK_TIMEOUT 1000

//BAUD RATE
#define BAUD_RATE 115200

//Interface ralays states
#define RELAY_ON  HIGH
#define RELAY_OFF LOW

//Cycles in between updates
#define SECONDS_PER_CICLE 30
#define CICLES_PER_PRESENT 2880
#define CICLES_PER_TIMEUPDATE 120

//Transmit Retries
#define HIGH_PRIORITY_RETRIES 4
#define LOW_PRIORITY_RETRIES 2

//PINs
#define MR_pin 6

//Roller States
#define UP 100
#define DOWN 0
#define STOP -1

//Roller maxtravel time
#define ROLLER_TIMEOUT 30000
#define ROLLER_DELAY 500
#define ROLLER_PULSE 250

//Analog Interfaces (switches)
#define ANALOG_INTERFACES 16

//Rollers
#define ROLLERS 8
#define TEXT_ID 9

MyMessage message;

char buf[50];
unsigned int messagesFailed=0;
volatile int isACKed = false, NewRollerPosition[ROLLERS], RollerPosition[ROLLERS];
unsigned int i;
unsigned int topresent = CICLES_PER_PRESENT;

boolean ack = true,led = false;
boolean metric, working[ROLLERS], timeReceived = false;

unsigned long sleep_time = (SECONDS_PER_CICLE); // Sleep time between reads (in milliseconds)
unsigned long cicles=0, currentMillis,loopMillis, millisUP[ROLLERS], millisDOWN[ROLLERS], millisSTOP[ROLLERS], millisDISABLE[ROLLERS];

int convert[] = {7, 6, 5, 4, 3, 2, 1, 0, 15, 14, 13, 12, 11, 10, 9, 8};
int rollerup[] = {7, 5, 3, 1, 15, 13, 11, 9};
int rollerdown[] = {6, 4, 2, 0, 14, 12, 10, 8};

void before() {
  //disable watchdog timer
  MCUSR = 0;
  Serial.begin(BAUD_RATE);
  Serial.println(F("begin"));

  //setup interface library
  //disables shift master reset pin
  pinMode(MR_pin,OUTPUT);
  digitalWrite(MR_pin,HIGH);
  interface.setBitCount(ANALOG_INTERFACES);
  interface.setPins(5, 4, 3); 

  //Initializing
  for (i=0;i<ANALOG_INTERFACES;i++) {
    interface.writeBit(i, RELAY_OFF);
  }
  for (i=0;i<ROLLERS;i++) {
    NewRollerPosition[i]=STOP;
    RollerPosition[i]=STOP;
    working[i]=false;
  }
  
  //blink LED on power up
  pinMode(13,OUTPUT);
  for (i = 0 ; i<LED_POWERUP_COUNT ;i++) {
    Serial.print(".");
    digitalWrite (13, HIGH);
    delay(LED_DELAY);
    digitalWrite (13, LOW);
    delay(LED_DELAY);
    delay(LED_DELAY);
  }
}

void setup()  
{
  Serial.println("");
  //activate watchdog timer
  wdt_enable(WDTO_8S);
  while (timeReceived != true) {
    Serial.println(F("Requesting Time"));
    //request time
    requestTime(); 
    wait(1000);
  }  
  PrintTime ();

  metric = getConfig().isMetric;
  Serial.println(F("Listening for Commands"));
  Serial.println(F("Starting Loop"));
} 

void presentation ()
{
  gwPresent ();
}

void loop() 
{
  //check if it should reboot itself
  if ((hour() == 2) && (minute() <= 1 ) && (cicles > 100)) {
    Serial.println(F("Reboot time reached (2:00)"));
    asm volatile ( "jmp 0");
    wait(100);
  }
    
  if ((minute() == 20) && (second() <= 30)) {
    timeReceived = false;
    while (timeReceived != true) {
      //request time
      requestTime(); 
      wait(500);
    }
  }  
  cicles++;
  Serial.print(F("Waiting in cicle "));
  //displays the current cicle count
  Serial.println(cicles);

  PrintTime ();

  //check if it should update time (every 30 minutes)
  if (((minute() == 30) || (minute() == 0)) && (second() <= 30)) {
    UpdateTimeDomoticz ();
  }
  //stays in the state machine until end off current loop time
  loopMillis = hwMillis();
  while (hwMillis() - loopMillis < sleep_time*1000) {
    currentMillis = hwMillis(); // assure consistent time comparitions
    _process();
    wdt_reset();

    //loop trought the rollers controls
    for (i=0;i<ROLLERS;i++) {
      state_machine (i);
    }
  }
} 

void state_machine (int roller) {
  
  if (NewRollerPosition[roller] == RollerPosition[roller]) {
    //control the relays accoring to the timers
    if (working[roller] == true) {
      if (currentMillis > millisDISABLE[roller]) { //stop the pulse
        Serial.print(F("disabling pulse for roller "));
        Serial.println(roller);
        interface.writeBit(rollerdown[roller], RELAY_OFF);
        interface.writeBit(rollerup[roller], RELAY_OFF);    
        millisDISABLE[roller] = currentMillis + ROLLER_TIMEOUT + 1;
            
      } else if (currentMillis > millisSTOP[roller]) { //STOP
        Serial.print(F("Stopping roller "));
        Serial.println(roller);
        working[roller] = false;
        RollerPosition[roller]=NewRollerPosition[roller]=STOP;
        
      } else if (currentMillis > millisUP[roller]) { //Go UP
        Serial.print(F("Going UP roller "));
        Serial.println(roller);
        interface.writeBit(rollerup[roller], RELAY_ON);
        millisDISABLE[roller] = currentMillis + ROLLER_PULSE; //setup disable pulse
        millisUP[roller] = currentMillis + ROLLER_TIMEOUT + 1;
        
      } else if (currentMillis > millisDOWN[roller]) { //Go DOWN
        Serial.print(F("Going DOWN roller "));
        Serial.println(roller);
        interface.writeBit(rollerdown[roller], RELAY_ON);
        millisDISABLE[roller] = currentMillis + ROLLER_PULSE; //setup disable pulse
        millisDOWN[roller] = currentMillis + ROLLER_TIMEOUT + 1;
      }
    }  
  } else {     
    switch (NewRollerPosition[roller]) {
      case UP: // New position is UP
        Serial.print(F("Preparing UP command roller "));
        Serial.println(roller);
        //stop the roller if working
        if ((working[roller] == true) && (RollerPosition[roller] == DOWN)) {
          interface.writeBit(rollerdown[roller], RELAY_OFF);
          interface.writeBit(rollerup[roller], RELAY_ON); //stops the roller
          millisDISABLE[roller] = currentMillis + ROLLER_PULSE; //setup disable pulse
        }
        //reset time variables
        millisUP[roller] = currentMillis+ROLLER_DELAY;     //setup up time
        millisSTOP[roller] = currentMillis+ROLLER_TIMEOUT; //setup stop time
        millisDOWN[roller] = currentMillis + ROLLER_TIMEOUT + 1;              //cancel down time
        working[roller] = true;
        break;
      case DOWN: // New position is DOWN
        Serial.print(F("Preparing DOWN command "));        
        Serial.println(roller);
        //stop the roller if working
        if ((working[roller] == true) && (RollerPosition[roller] == UP)) {
          interface.writeBit(rollerup[roller], RELAY_OFF);
          interface.writeBit(rollerdown[roller], RELAY_ON); //stops the roller
          millisDISABLE[roller] = currentMillis + ROLLER_PULSE; //setup disable pulse
        }
        //reset time variables
        millisDOWN[roller] = currentMillis+ROLLER_DELAY;   //setup down time
        millisSTOP[roller] = currentMillis+ROLLER_TIMEOUT; //setup stop time
        millisDISABLE[roller] = currentMillis+ROLLER_PULSE; //setup disable pulse
        millisUP[roller] = currentMillis + ROLLER_TIMEOUT + 1;                //cancel up time
        working[roller] = true;
        break;
      case STOP: // New position is STOPPED
        Serial.print(F("Executing STOP command "));
        Serial.println(roller);
        //stop the roller if working
        if ((working[roller] == true) && (RollerPosition[roller] == UP)) {
          interface.writeBit(rollerup[roller], RELAY_OFF);
          interface.writeBit(rollerdown[roller], RELAY_ON); //stops the roller
        }
        if ((working[roller] == true) && (RollerPosition[roller] == DOWN)) {
          interface.writeBit(rollerdown[roller], RELAY_OFF);
          interface.writeBit(rollerup[roller], RELAY_ON); //stops the roller
        }
        //stop time variables processing
        working[roller] = false;
        //waits for the controller
        wait(100);
        //clears inverse commands
        interface.writeBit(rollerdown[roller], RELAY_OFF);
        interface.writeBit(rollerup[roller], RELAY_OFF);
        break;
    }
    RollerPosition[roller]=NewRollerPosition[roller];
  }
}  

void UpdateTimeDomoticz () {
  sprintf(buf,"%d:%d:%d %s %d/%d/%d",hour(),minute(),second(),dayStr(weekday()),day(),month(),year());
  Serial.println(F("SendingTime"));
  resend(message.setSensor(TEXT_ID).setType(V_TEXT).set(buf),LOW_PRIORITY_RETRIES,ACK_TIMEOUT);
}

void PrintTime () {
  //display current time
  Serial.print(F("Current date: "));
  Serial.print(dayStr(weekday()));
  Serial.print(F(" "));
  Serial.print(day());
  Serial.print(F("/"));
  Serial.print(monthStr(month()));
  Serial.print(F("/"));
  Serial.println(year()); 
  Serial.print(F("Current time: "));
  Serial.print(hour());
  Serial.print(F(":"));
  Serial.print(minute());
  Serial.print(F(":"));
  Serial.println(second());
}

void gwPresent () {
  //present at beggining and every day
  if (topresent < CICLES_PER_PRESENT) {
    topresent++;
    return;
  }
  Serial.println(F("Presenting"));
  //reset count;
  topresent = 0;

  sendSketchInfo(SKETCH_INFO, SKETCH_VERSION);
  for (i=0;i<ROLLERS;i++) {
    present(i+1, S_COVER, ROLLER_ID_INFO);
    wait (500);
  }
  present(TEXT_ID, S_INFO, "Time feedback");
}

void resend(MyMessage &msg, int repeats, int timeout)
{
  int repeat = 0;
  int repeatdelay = 0;
  boolean sendOK = false;

  while ((sendOK == false) and (repeat < repeats)) {
    send(msg,true);

    if (waitACK(timeout)) {
      sendOK = true;
      messagesFailed = 0;
    } else {
      sendOK = false;
      Serial.print("Retry ");
      Serial.print(repeat);
      Serial.print(" Failed ");
      Serial.println(messagesFailed);
      repeatdelay += 500;
      wdsleep(repeatdelay);
    }
    repeat++; 
  }
  if (sendOK == false) {
    if (messagesFailed > MESSAGES_FAILED_REBOOT) {
      //wdt_enable(WDTO_15MS);
      asm volatile ( "jmp 0");
      wait(100);
    }
    messagesFailed++;
  }
}

boolean waitACK (int timeout) {
  unsigned long startTime = millis();
  
  while ((millis() - startTime) < timeout) {
    wait(1);
    if (isACKed == true) {
      isACKed = false;
      Serial.print(F("Reply "));
      Serial.print(timeout);
      Serial.print(" ");
      Serial.println((millis() - startTime));
      return true;
    }
  }
  return false;
}

void receive (const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.

  if (message.isAck()) {
    Serial.println(F("This is an ack from gateway."));
    isACKed = true;
  } else if (message.type == V_UP) {
    Serial.print(F("UP Command Received for sensor "));
    Serial.println(message.sensor);
    NewRollerPosition[message.sensor-1] = STOP;
    state_machine(message.sensor-1);
    NewRollerPosition[message.sensor-1] = UP;
  } else if (message.type == V_DOWN) {
    Serial.print(F("DOWN Command Received for sensor "));
    Serial.println(message.sensor);
    NewRollerPosition[message.sensor-1] = STOP;
    state_machine(message.sensor-1);
    NewRollerPosition[message.sensor-1] = DOWN;

  } else if (message.type == V_STOP) {
    Serial.print(F("STOP Command Received for sensor "));
    Serial.println(message.sensor);
    NewRollerPosition[message.sensor-1] = STOP;
    state_machine(message.sensor-1);
  } else {
    Serial.print(F("Incoming change for sensor:"));
    Serial.println(message.sensor);
  }
}

void receiveTime(unsigned long time) {
  setTime(time);
  timeReceived = true;
}

void wdsleep(unsigned long ms) {
  unsigned long enter = hwMillis();
  #if defined(MY_REPEATER_FEATURE)
  while (hwMillis() - enter < ms) {
     _process();
    wdt_reset();
  }
    wdt_reset();
  }
  #else
    sleep(ms);
  #endif
}

