#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SparkFun_MCP9600.h>
MCP9600 tempSensor;

//This sketch requires the modified LMIC library found in the Github repo.
//It is designed to work with the Dragino LG01-N.  This gateway only has a single
//channel, so we restrict the node to that channel.
//Update the frequency channel being used to that in your location.  Currently
//this is setup for channel 0 of AS923

//The DHTlib library is for the DHT11 temperature and humidity sensor, and can be found
//through the library manager.


//The network, app and device keys are setup for my applications on The Things Network.
//You will need to update these values if using yourself.

//The data to be sent is defined in Data().  There is a global array 'mydata' that holds
//the values to be sent.

//This is an ABP example, and my advice is to allow messages with a count lower than
//the highest number received, particularly when experimenting.

//#define PIN 4

// LoRaWAN NwkSKey, network session key from TTN - Change this with your Unique Key



// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//ttn
static const PROGMEM u1_t NWKSKEY[16] = { 0x17, 0xCE, 0xC1, 0xC5, 0x57, 0x3A, 0x92, 0x82, 0x56, 0x0B, 0xEE, 0x2D, 0x9E, 0x4C, 0x09, 0x27 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//ttn
static const u1_t PROGMEM APPSKEY[16] = { 0xFD, 0xB4, 0x6D, 0x01, 0x0A, 0x7F, 0x14, 0x6B, 0x6D, 0xDC, 0x35, 0x05, 0x6A, 0x24, 0xB5, 0xB8 };
//
// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// ttn
static const u4_t DEVADDR = 0x260010BD;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[4] = {0x00,0x00,0x00,0x00};
static osjob_t sendjob;
unsigned int message_count = 0;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

void Data()
{
  if(tempSensor.available()){
        Serial.print("Thermocouple: ");
        Serial.print(tempSensor.getThermocoupleTemp());
        mydata[0] = tempSensor.getThermocoupleTemp();
        Serial.print(" °C   Ambient: ");
        Serial.print(tempSensor.getAmbientTemp());
        mydata[1] = tempSensor.getAmbientTemp();
        Serial.print(" °C   Temperature Delta: ");
        Serial.print(tempSensor.getTempDelta());
        mydata[2] = tempSensor.getTempDelta();
        Serial.print(" °C");
        Serial.println();
        //delay(20); //don't hammer too hard on the I2C bus
    }


}

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) 
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else 
    {
        Data();
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    analogReference(INTERNAL);
//    pinMode(PIN,INPUT); // Configuring pin A1 as input
    Serial.begin(9600);
    Serial.println(F("Starting"));
    Wire.begin();
    Wire.setClock(100000);
    tempSensor.begin();       // Uses the default address (0x60) for SparkFun Thermocouple Amplifier

    //check if the sensor is connected
    if(tempSensor.isConnected()){
        Serial.println("Device will acknowledge!");
    }
    else {
        Serial.println("Device did not acknowledge! Freezing.");
        while(1); //hang forever
    }

    //check if the Device ID is correct
    if(tempSensor.checkDeviceID()){
        Serial.println("Device ID is correct!");        
    }
    else {
        Serial.println("Device ID is not correct! Freezing.");
        while(1);
    }

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
