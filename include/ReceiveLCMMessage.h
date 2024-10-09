#ifndef __recieve_lcm_message_hpp__
#define __recieve_lcm_message_hpp__

#include <lcm/lcm-cpp.hpp>
#include <stdio.h>
#include <iostream>
#include "gamepad_lcmt.hpp"
#include "GamepadCommand.h"
#include "Utilities/utilities.h"

// GamepadCommand* _gamepadCommand;

int RecieveLCMGamepadCommand(GamepadCommand* _gamepadCommand);


#endif