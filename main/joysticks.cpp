#include "joysticks.h"
#include <cstdint>
#include "ArduinoConsole.h"
#include "ArduinoController.h"
#include "esp32-hal.h"

void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "idx=%d, dpad: 0x%02x, buttons: a:%d b:%d x:%d y:%d, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->a(),
        ctl->b(),
        ctl->x(),
        ctl->y(),
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void JoyController::setLeft(bool left){
    _left = left;
}

void JoyController::setController(ControllerPtr ctrl){
    _ctrl = ctrl;
    _lastUpdate = millis();
    fConnected = true;
}

ControllerPtr JoyController::getController(){
    return _ctrl;
}

void JoyController::disconnect(){
    _ctrl->disconnect();
    _ctrl = nullptr;
    fConnected = false;
}

void JoyController::notify(){
    {
        uint32_t currentTime = millis();
        uint32_t lagTime = (currentTime > _lastUpdate) ? currentTime - _lastUpdate : 0;
        if(_ctrl){
            if (_ctrl->hasData())
            {
                mapController();
                _lastUpdate = currentTime;
            }
            else if (lagTime > 5000)
            {
                Console.println("More than 5 seconds. Disconnect");
                clearController();
                disconnect();
            }
            else if (lagTime > 300)
            {
                Console.println("It has been 300ms. Clear controller");
                clearController();
            }
        }
    }

}


void JoyController::mapController(){
    Event evt = {};
    State prev = state;
    _mapped = false;
    _mapped = _left ? mapLeft() : mapRight();
    if (_mapped){
        #ifdef CONFIG_DEBUG_JOYSTICK
        dumpGamepad(_ctrl);
        #endif

        #define CHECK_BUTTON_DOWN(b) evt.button_down.b = (!prev.button.b && state.button.b)
        CHECK_BUTTON_DOWN(select);
        CHECK_BUTTON_DOWN(start);
        CHECK_BUTTON_DOWN(l3);
        CHECK_BUTTON_DOWN(r3);

        CHECK_BUTTON_DOWN(l2);
        CHECK_BUTTON_DOWN(r2);
        CHECK_BUTTON_DOWN(l1);
        CHECK_BUTTON_DOWN(r1);

        CHECK_BUTTON_DOWN(triangle);
        CHECK_BUTTON_DOWN(circle);
        CHECK_BUTTON_DOWN(cross);
        CHECK_BUTTON_DOWN(square);

        #define CHECK_BUTTON_UP(b) evt.button_up.b = (prev.button.b && !state.button.b)
        CHECK_BUTTON_UP(select);
        CHECK_BUTTON_UP(start);
        CHECK_BUTTON_UP(l3);
        CHECK_BUTTON_UP(r3);

        CHECK_BUTTON_UP(l2);
        CHECK_BUTTON_UP(r2);
        CHECK_BUTTON_UP(l1);
        CHECK_BUTTON_UP(r1);

        CHECK_BUTTON_UP(triangle);
        CHECK_BUTTON_UP(circle);
        CHECK_BUTTON_UP(cross);
        CHECK_BUTTON_UP(square);
    }
    event = evt;
}

bool JoyController::mapLeft(){
    state.button.select = _ctrl->miscStart();
    state.button.start = _ctrl->miscSelect();
    state.button.l3 = _ctrl->thumbL();
    state.button.circle = _ctrl->y();
    state.button.cross = _ctrl->b();
    state.button.triangle = _ctrl->x();
    state.button.square = _ctrl->a();
    state.button.l1 = _ctrl->l2();
    state.button.l2 = _ctrl->r2();
    state.button.r1 = _ctrl->l1();
    state.button.r2 = _ctrl->r1();
    state.analog.stick.lx = map(_ctrl->axisY()*-1,-512,512,-127,127);
    state.analog.stick.ly = map(_ctrl->axisX()*-1,-512,512,127,-127);
    #ifdef DEBUG_JOYSTICK
    Console.printf("Mapped left X: %d Y: %d\n", state.analog.stick.lx, state.analog.stick.ly);
    #endif
    return true;
}

bool JoyController::mapRight(){
    state.button.select = _ctrl->miscSelect();
    state.button.start = _ctrl->miscStart();
    state.button.r3 = _ctrl->thumbL();
    state.button.circle = _ctrl->a();
    state.button.cross = _ctrl->x();
    state.button.triangle = _ctrl->b();
    state.button.square = _ctrl->y();
    state.button.l1 = _ctrl->l1();
    state.button.l2 = _ctrl->r1();
    state.button.r1 = _ctrl->l2();
    state.button.r2 = _ctrl->r2();

    state.analog.stick.rx = map(_ctrl->axisY(),-512,512,-127,127);
    state.analog.stick.ry = map(_ctrl->axisX(),-512,512,-127,127);
    #ifdef DEBUG_JOYSTICK
    Console.printf("Mapped right X: %d Y: %d\n", state.analog.stick.rx, state.analog.stick.ry);
    #endif
    return true;
}

void JoyController::clearController(){
    state.button.select = 0;
    state.button.start = 0;
    state.button.l3 = 0;
    state.button.r3 = 0;
    state.button.circle = 0;
    state.button.cross = 0;
    state.button.triangle = 0;
    state.button.square = 0;
    state.button.l1 = 0;
    state.button.l2 = 0;
    state.button.r1 = 0;
    state.button.r2 = 0;
    state.analog.stick.lx = 0;
    state.analog.stick.ly = 0;
    state.analog.stick.rx = 0;
    state.analog.stick.ry = 0;
}
