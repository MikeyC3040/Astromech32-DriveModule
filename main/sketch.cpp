// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2
#include "sdkconfig.h"
#include <Arduino.h>

#define USE_DEBUG
#define USE_MOTOR_DEBUG

#include "ArduinoConsole.h"
#include "HardwareSerial.h"
#include "ReelTwo.h"
#include "core/AnimatedEvent.h"
#include "core/SetupEvent.h"
#include "drive/TankDriveCytron.h"
#include "esp32-hal.h"
#include "hcr.h"

#include <Arduino.h>
#include <Bluepad32.h>

#include "joysticks.h"

JoyController leftStick=JoyController();
JoyController rightStick=JoyController();

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    if (ctl->getModelName() == "Switch JoyCon Left") {
        if (!leftStick.isConnected()){
            Console.println("CALLBACK: Left controller is connected...");
            leftStick.setController(ctl);
            ctl->setPlayerLEDs(1);
            leftStick.onConnect();
            leftStick.setLeft(true);
            Console.printf("Controller at address: %d\n", leftStick.getController());
            foundEmptySlot = true;
        }
    } else if (ctl->getModelName() == "Switch JoyCon Right") {
        if (!rightStick.isConnected()){
            Console.println("CALLBACK: Right controller is connected...");
            rightStick.setController(ctl);
            ctl->setPlayerLEDs(2);
            rightStick.onConnect();
            Console.printf("Controller at address: %d\n", rightStick.getController());
            foundEmptySlot = true;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Controller connected, but could not find empty joystick");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;
    if (leftStick.getController() == ctl) {
        Console.println("CALLBACK: Controller disconnected from left");
        leftStick.setController(nullptr);
        leftStick.onDisconnect();
        foundController = true;
    }
    else if (rightStick.getController() == ctl) {
        Console.println("CALLBACK: Controller disconnected from right");
        rightStick.setController(nullptr);
        rightStick.onDisconnect();
        foundController = true;
    }
    if (!foundController) {
        Console.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void bpSetup(){
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController, true);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    //BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    // Enables the BLE Service in Bluepad32.
    // This service allows clients, like a mobile app, to setup and see the state of Bluepad32.
    // By default, it is disabled.
    BP32.enableBLEService(false);
}


TankDriveCytron tankDrive(uint8_t(0),Serial2,MDDS30,leftStick);


HCRVocalizer HCR(&Serial1,115200); // Serial (Stream Port, baud rate)
// Arduino setup function. Runs in CPU 1
void setup() {
    bpSetup();
    Serial1.begin(115200, SERIAL_8N1, 26, 27);
    HCR.begin();
    HCR.SetMuse(false);
    HCR.OverrideEmotions(false);
    REELTWO_READY();
    Serial2.begin(115200);
    SetupEvent::ready();
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    if (BP32.update()){
        leftStick.notify();
        rightStick.notify();
        if (rightStick.event.button_down.r1){
            HCR.ToggleMuse();
        }
        int emoteLevel = EMOTE_MODERATE;
        if (rightStick.state.button.r2){
            emoteLevel = EMOTE_STRONG;
        }
        if(rightStick.event.button_down.circle){
            HCR.Stimulate(HAPPY, emoteLevel);
        }
        else if(rightStick.event.button_down.cross){
            HCR.Stimulate(SAD, emoteLevel);
        }
        else if(rightStick.event.button_down.square){
            HCR.Stimulate(SCARED, emoteLevel);
        }
        else if(rightStick.event.button_down.triangle){
            HCR.Stimulate(MAD, emoteLevel);
        }
        if(rightStick.event.button_down.start){
            int channel = CH_A;
            int song = 0;
            if (emoteLevel == EMOTE_STRONG){
                channel = CH_B;
                song = 1;
            }
            int currentWav = HCR.GetPlayingWAV(channel);
            if (currentWav != -1){
                HCR.StopWAV(channel);
            } else {
                HCR.PlayWAV(channel,song);
            }
        }
    }
    AnimatedEvent::process();
    HCR.update();
    vTaskDelay(1);
}
