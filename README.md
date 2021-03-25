# STM32_AnalogBrushlessServomotorController_10v_StepDir
Project in development for an old Brushless analog (+10/-10v) trapezoidal magnet servomotor that accepts step dir input 

The idea behind this project in current development is to provide a conversion controller for old CNC machinery where brushless controlers expect analog speed/torque input signal
with values of +10v/-10v. The advantaje with the system is that the position feedback is provided by an encoder, in this case an incremental heds 9140 with two digital channel and
a complete rotation digital signal thus preventing the motor from lose position in comparission to Stepper motors. A problem is that this type of interface is no longer
supported by most common cnc controllers. 


An old solution to this problem was developed in CNC Forums with a board named YAPSC +10V that takes the encoder input and controls the direction output with PWM. In this case the 
development in an effort to reutilize an old motor provided in old high speed Pick-And-Place robots. The available board for testin is the nucleo STM32F334R8 that includes DSP
functions in hardware for speed. CMSIS provides a low level highly optimized PID library that allow the system to keep the sampling rate of the motor.

Soon i will provide more data about the project...


ToDo:

Electrical diagram

Parts data, frequency and evidence of the development

Include a memory for store and modification for pid constant and configuration and a terminal for config mode.

Development of a GUI for tunning.

Port to STM32F411 microcontroller
