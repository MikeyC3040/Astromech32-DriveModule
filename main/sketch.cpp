// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2
#include "sdkconfig.h"
#include <Arduino.h>
#include "esp32-hal.h"

#define USE_DEBUG
//#define USE_MOTOR_DEBUG
//#define USE_DOME_DEBUG
//#define USE_VERBOSE_DOME_DEBUG

#include "ArduinoConsole.h"
#include "HardwareSerial.h"
#include "Wire.h"
#include "ReelTwoSMQ32.h"
#include "ReelTwo.h"
#include "core/AnimatedEvent.h"
#include "core/SetupEvent.h"
#include "drive/TankDriveCytron.h"
#include "drive/DomeDriveCytron.h"
#include "ServoDispatchDirect.h"
#include "core/PushButton.h"

#define HCR_I2C_RATE 100000
#include "hcr.h"

#include <Bluepad32.h>

#include "joysticks.h"

JoyController leftStick=JoyController();
JoyController rightStick=JoyController();

PushButton utilBtn(14);

bool pairingBT = true;

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    if (ctl->getModelName() == "Switch JoyCon Left") {
        if (!leftStick.isConnected()){
            leftStick.setController(ctl);
            ctl->setPlayerLEDs(1);
            leftStick.onConnect();
            leftStick.setLeft(true);
            foundEmptySlot = true;
            Console.println("CALLBACK: Left controller is connected...");
        }
    } else if (ctl->getModelName() == "Switch JoyCon Right") {
        if (!rightStick.isConnected()){
            rightStick.setController(ctl);
            ctl->setPlayerLEDs(2);
            rightStick.onConnect();
            foundEmptySlot = true;
            Console.println("CALLBACK: Right controller is connected...");
        }
    }
    if (leftStick.getController() && rightStick.getController()){
        pairingBT = false;
        BP32.enableNewBluetoothConnections(pairingBT);
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Controller connected, but could not find empty joystick");
    }
}


void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;
    if (leftStick.getController() == ctl) {
        leftStick.disconnect();
        foundController = true;
        Console.println("CALLBACK: Controller disconnected from left");
    }
    else if (rightStick.getController() == ctl) {
        rightStick.disconnect();
        foundController = true;
        Console.println("CALLBACK: Controller disconnected from right");
    }
    if (!foundController) {
        ctl->disconnect();
        Console.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void bpSetup(){
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController, pairingBT);

    //BP32.forgetBluetoothKeys();

    BP32.enableVirtualDevice(false);

    // Enables the BLE Service in Bluepad32.
    // This service allows clients, like a mobile app, to setup and see the state of Bluepad32.
    // By default, it is disabled.
    BP32.enableBLEService(false);
}

void toggleBTPairing(){
    pairingBT = !pairingBT;
    if (pairingBT){
        Console.println("BT Pairing Mode Enabled");
        BP32.enableNewBluetoothConnections(true);
    } else {
        Console.println("BT Pairing Mode Disabled");
        BP32.enableNewBluetoothConnections(false);
    }
};

TankDriveCytron tankDrive(uint8_t(0),Serial2,MDDS30,leftStick, false);
DomeDriveCytron domeDrive(uint8_t(1),Serial1,MDDS10,rightStick, false);

HCRVocalizer HCR(1,Wire);
// Arduino setup function. Runs in CPU 1
void setup() {
    bpSetup();
    Serial1.begin(9600, SERIAL_8N1, 13, 25); //rx, tx
    Serial2.begin(9600, SERIAL_8N1, 16, 17); //rx, tx
    Wire.setPins(26,27);
    HCR.begin();
    HCR.SetMuse(false);
    HCR.OverrideEmotions(false);
    tankDrive.setMaxSpeed(0.25);
    tankDrive.setAccelerationScale(5);
    tankDrive.setDecelerationScale(5);
    utilBtn.attachClick(toggleBTPairing);
    REELTWO_READY();
    SetupEvent::ready();
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    if (BP32.update()){
        leftStick.notify();
        rightStick.notify();
        if (rightStick.event.button_down.r1){
            HCR.ToggleMuse();
            Console.printf("Musing: %d\n", HCR.GetMuse());
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
            Console.printf("Channel: %d Song: %d, CurrentWave: %d\n", channel, song, currentWav);
            if (currentWav != -1){
                HCR.StopWAV(channel);
            } else {
                HCR.PlayWAV(channel,song);
            }
        }
        //Console.printf("HCR State: %f %f %f %f\n", HCR.GetEmotions());
    }
    AnimatedEvent::process();
    //HCR.update();
    vTaskDelay(1);
}
