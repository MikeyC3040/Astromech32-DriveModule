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
        _left = false;
        _lastUpdate = 0;
        _ctrl = nullptr;
    }
    void setLeft(bool left);
    void setController(ControllerPtr ctrl);
    ControllerPtr getController();
    virtual void disconnect() override;
    virtual void notify() override;
    void clearController();

    private:
    void mapController();
    unsigned long _lastUpdate;
    ControllerPtr _ctrl;
    bool mapLeft();
    bool mapRight();
    bool _mapped;
    bool _left;
};

typedef JoyController* JoyPtr;

#endif
