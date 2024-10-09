#include <ReceiveLCMMessage.h>  
#include <assert.h>


// Create a class that handles the recieved messages

class Handler {
  public:
    Handler (GamepadCommand* gamepadCommand_ptr):_gamepadCommand(gamepadCommand_ptr) {}
    ~Handler() {}
    GamepadCommand* _gamepadCommand;
    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const gamepad_lcmt *_gamepad_lcmt) // function that handles recieved lcm message (inputs are specified by LCM)
    {
        
        // printf("Received message on channel \"%s\":\n", chan.c_str());

        _gamepadCommand->set(_gamepad_lcmt); // Set the GamepadCommand object with recieved LCM message

        //PRINT THE JOYSTICK ---------------------------------------------------
        
        // std::cout << "[DESIRED STATE COMMAND] Printing Raw Gamepad Info...\n";
        // std::cout << "---------------------------------------------------------\n";
        // std::cout << "Button Start: " << _gamepadCommand->start
        //           << " | Back: " << _gamepadCommand->back << "\n";
        // std::cout << "Button A: " << _gamepadCommand->a
        //           << " | B: " << _gamepadCommand->b << " | X: " << _gamepadCommand->x
        //           << " | Y: " << _gamepadCommand->y << "\n";
        // std::cout << "Left Stick Button: " << _gamepadCommand->leftStickButton
        //           << " | X: " << _gamepadCommand->leftStickAnalog[0]
        //           << " | Y: " << _gamepadCommand->leftStickAnalog[1] << "\n";
       // std::cout << "Right Analog Button: " << _gamepadCommand->rightStickButton
        //           << " | X: " << _gamepadCommand->rightStickAnalog[0]
        //           << " | Y: " << _gamepadCommand->rightStickAnalog[1] << "\n";
        // std::cout << "Left Bumper: " << _gamepadCommand->leftBumper
        //           << " | Trigger Switch: " << _gamepadCommand->leftTriggerButton
        //           << " | Trigger Value: " << _gamepadCommand->leftTriggerAnalog
        //           << "\n";
        // std::cout << "Right Bumper: " << _gamepadCommand->rightBumper
        //           << " | Trigger Switch: " << _gamepadCommand->rightTriggerButton
        //           << " | Trigger Value: " << _gamepadCommand->rightTriggerAnalog
        //           << "\n\n";
        // std::cout << std::endl;

    }
};


int RecieveLCMGamepadCommand(GamepadCommand* _gamepadCommand)
{
    // set LCM URL for connecting another PC 255 can be anything as soon as communicating pc agrees
    lcm::LCM lcm(getLcmUrl(255)); 

    if (!lcm.good())
        return 1;

    //  Creating Handler object and subscribtion
    //  Every time a msg with specified type comes go to the handleMessage function
    //  (Same as ROS, ROS2 callBackFunction)
    Handler handlerObject(_gamepadCommand); 
    //std::cout<<"not yet"<<std::endl;
    lcm.subscribe("command", &Handler::handleMessage, &handlerObject);

    lcm.handle();


    return 0;
}
