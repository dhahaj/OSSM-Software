#include "encoder.h"

// Define the global encoder instance
// 6th arg = areEncoderPinsPulldown_forEsp32. false -> INPUT_PULLUP on A/B,
// matching the remote PCB where the encoder common is wired to GND.
AiEsp32RotaryEncoder encoder(
    Pins::Remote::encoderB,
    Pins::Remote::encoderA,
    Pins::Remote::encoderSwitch,
    Pins::Remote::encoderPower,
    Pins::Remote::encoderStepsPerNotch,
    false
);

void IRAM_ATTR readEncoderISR() {
    encoder.readEncoder_ISR();
}

void initEncoder() {
    encoder.begin();
    encoder.setup(readEncoderISR);
    encoder.setBoundaries(0, 99, false);
    encoder.setAcceleration(0);
    encoder.disableAcceleration();
}
