# Servomotor Control System (Editing)
Servomotor control using a microcontroller
```
* Hardware: PIC24(16-bit microcontroller), SG90(Servo), LCD, keypad, ADC, PWM, RTCC, UART protocol, Circuit design
* Software: C programming
```

## _1. System Overview_

### (1) Diagram
User can interact with the system through `Keypad` (input), `LCD` and `Teraterm` (output).

`Output Compare` module generates PWM(Pulse Width Modulation) signal to move a servo.

![alt text](https://github.com/lkyungho/Images/blob/master/servomotor-control-diagram.JPG "Diagram")


### (2) System Operation
**`update_state`** function in **`project_servo.c`** determines microcontroller operation for servo control.

FSM(Finite State Machine) is used to define the operation.

![alt text](https://github.com/lkyungho/Images/blob/master/servomotor-control-update-state.JPG "update_state")

## _2. Descriptions of the System_
### (1) System Options
A LCD shows options to users so that users can interact with the system. There are three options; **`SETUP`**, **`RUN`**, and **`DOWNLOAD`**. Users can select an option using a keypad.
### (2) Servomotor
(_For more information about servos: [servocity.com](https://www.servocity.com/servos)_)

The shaft of the servo can be positioned to specific angular positions by sending a coded signal. As long as the coded signal exists on the input line, the servo will maintain the angular position of the shaft. If the coded signal changes, then the angular position of the shaft changes.
### (3) SETUP option
