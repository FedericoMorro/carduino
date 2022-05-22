#include <IRremote.hpp>

#define IR_RECEIVE_PIN 9
#define PIN_RBGLED 4

struct IRData {
    decode_type_t protocol;  // UNKNOWN, NEC, SONY, RC5, ...
    uint16_t address;        // Decoded address
    uint16_t command;        // Decoded command
    uint16_t extra;          // Used by MagiQuest and for Kaseikyo unknown vendor ID.  Ticks used for decoding Distance protocol.
    uint16_t numberOfBits;   // Number of bits received for data (address + command + parity) - to determine protocol length if different length are possible.
    uint8_t flags;               // See IRDATA_FLAGS_* definitions above
    uint32_t decodedRawData;     // Up to 32 bit decoded raw data, used for sendRaw functions.
    irparams_struct *rawDataPtr; // Pointer of the raw timing data to be decoded. Mainly the data buffer filled by receiving ISR.
};

void setup() {
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}

void loop() {
    if (IrReceiver.decode()) {
        Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
        IrReceiver.printIRResultShort(&Serial); // optional use new print version
        IrReceiver.resume(); // Enable receiving of the next value
    }
}