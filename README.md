# Servomotor Control System (Editing)
Servomotor control using a microcontroller
```
* Hardware: PIC24(16-bit microcontroller), SG90(Servo), LCD, keypad, ADC, PWM, RTCC, UART protocol, Circuit design
* Software: C programming
```

## _1. System Diagram_
(_Datasheet: [PIC24HJ128GP502](http://ww1.microchip.com/downloads/en/devicedoc/70293g.pdf)_)

Users can interact with the system through `Keypad` (input), `LCD` and `Teraterm` (output).

`Output Compare` module generates PWM(Pulse Width Modulation) signal to move a servo.

![alt text](https://github.com/lkyungho/Images/blob/master/servomotor-control-diagram.JPG "Diagram")

## _2. Servomotor_
(_For more information about servos: [servocity.com](https://www.servocity.com/servos)_)

The shaft of the servo can be positioned to specific angular positions by sending a coded signal. As long as the coded signal exists on the input line, the servo will maintain the angular position of the shaft. If the coded signal changes, then the angular position of the shaft changes. The maximun pulse width is 12% of period and the minimun pulse width is 3% of period.

![alt text](https://github.com/lkyungho/Images/blob/master/servomotor-control-servo.JPG "Servo")


## _3. System Operation_
**`update_state`** function in **`project_servo.c`** determines microcontroller operation for servo control.

FSM(Finite State Machine) is used to define the operation.

![alt text](https://github.com/lkyungho/Images/blob/master/servomotor-control-update-state.JPG "update_state")

## _4. Descriptions of the Operation_
### (1) System Options
A LCD shows options to users so that users can interact with the system. There are three options; **`SETUP`**, **`RUN`**, and **`DOWNLOAD`**. Users can select an option using a keypad.

### (2) SETUP option
This option allows users to setup the range of servo angles. Users can limit the range of clockwise (CW) direction and the range of counter clockwise (CCW) direction. (The maximum value of CW value is 2.4 ms and the minimun value of CCW is 0.6 ms).

Voltage signal from the potentio-resistor is converted to digital signal by ADC module. The digital signal creates PWM signal at Output Compare module. The servo moves to the corresponded position with the PWM signal and The system saves the digital value.

### (3) RUN option

### (4) DOWNLOAD option

## _5. Result_
(Video link will be added)
