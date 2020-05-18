# Servomotor Control System (Editing)
Servomotor control using a microcontroller
```
* Hardware: PIC24(16-bit microcontroller), SG90(Servo), LCD, keypad, ADC, PWM, RTCC, UART protocol, Circuit design
* Software: C programming
```

## _1. System Overview_

### (1) Diagram
User can interact with the system through keypad (input), LCD and teraterm (output)

![alt text](https://github.com/lkyungho/Images/blob/master/servomotor-control-diagram.JPG "Diagram")


### (2) System Operation
**`update_state`** function in **`project_servo.c`** is the main function for servo control.

FSM(Finite State Machine) is used to determine system operation.

![alt text](https://github.com/lkyungho/Images/blob/master/servomotor-control-update-state.JPG "update_state")

## _2. Descriptions of the System_
### (1) System Options
A LCD shows options to users so that users can interact with the system. There are three options; **`SETUP`**, **`RUN`**, and **`DOWNLOAD`**. Users can select an option using a keypad.

### (2) SETUP option
