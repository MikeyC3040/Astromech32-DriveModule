#ifndef JOYSTICKS_H
#define JOYSTICKS_H

#include <ArduinoConsole.h>
#include <ArduinoController.h>
#include "ArduinoBluepad32.h"
#include "JoystickController.h"

void dumpGamepad(ControllerPtr ctl);
class JoyController: public JoystickController {
    public:
    JoyController(): JoystickController(){
        _drive = false;
        _lastUpdate = 0;
        _ctrl = nullptr;
    }
    void setDrive(bool drive);
    void setController(ControllerPtr ctrl);
    ControllerPtr getController();
    void mapController();
    void clearController();

    private:
    unsigned long _lastUpdate;
    ControllerPtr _ctrl;
    bool mapLeft();
    bool mapRight();
    bool _mapped;
    bool _drive;
};

typedef JoyController* JoyPtr;

#endif
