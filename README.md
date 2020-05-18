# Servomotor Control System (Editing)
Servomotor control using a microcontroller
```
* Hardware: PIC24(16-bit microcontroller), SG90(Servo), LCD, keypad, ADC, PWM, RTCC, UART protocol, Circuit design
* Software: C programming
```

## _1. System Diagram_
**`update_state`** function in **`project_servo.c`** is the main function for servo control.

FSM(Finite State Machine) is used to determine system operation.

![alt text](https://github.com/lkyungho/Images/blob/master/servomotor-control-diagram.JPG "System Diagram")

## _2. Description of the System_
### (1) Option State
This state shows menus to users so that users can select a option they want. There are three options; **`SETUP`**, **`RUN`**, and **`DOWNLOAD`**.

### (2) Setup State
