Communication Protocol

Reference standard 485 Modbus RTU protocol CRC16 check

Serial port parameters

Baud rate: 38400

Data bits: 8

Parity: no parity

Stop bit: 1

Slave fixed ID: 01

Control mode

1. T10 wireless remote control mode

2. Modbus speed closed loop control mode

3. Modbus position closed loop control mode

4. 485 joystick control mode

Register address and description

|  |  |  |  |  |
| --- | --- | --- | --- | --- |
| **Register function** | **address code**  **(Decimal)** | **address code**  **(Hexadecimal)** | **Read and write function code** | **Register value** |
| 485 rocker (vertical) | 10 | 0x0A | 0x03,0x06,0x10 | Signed hexadecimal parameter  Range: 800~~-800 |
| 485 joystick (horizontal) | 11 | 0x0B | 0x03,0x06,0x10 | Signed hexadecimal parameter  Range: 800~~-800 |
| Left wheel target speed | 12 | 0x0C | 0x03,0x06,0x10 | Signed hexadecimal parameter |
| Right wheel target speed | 13 | 0x0D | 0x03,0x06,0x10 | Signed hexadecimal parameter |
| Control mode | 80 | 0x50 | 0x03,0x06,0x10 | 0: wireless remote control mode  1: Speed closed loop mode  2. Position closed loop mode |
| Start and stop instructions | 81 | 0x51 | 0x03,0x06,0x10 | 0: stop, parameter configuration state  1: Start |
| Left wheel target speed | 82 | 0x52 | 0x03,0x06,0x10 | Signed hexadecimal parameter |
| Right wheel target speed | 83 | 0x53 | 0x03,0x06,0x10 | Signed hexadecimal parameter |
| Left wheel target mileage H | 84 | 0x54 | 0x03,0x06,0x10 | 32-bit unsigned pulse number |
| Left wheel target mileage L | 85 | 0x55 | 0x03,0x06,0x10 |
| Right wheel target mileage H | 86 | 0x56 | 0x03,0x06,0x10 | 32-bit unsigned pulse number |
| Right wheel target mileage L | 87 | 0x57 | 0x03,0x06,0x10 |
| Actual speed of Left wheel | 54 | 0x36 | 0x03 | Signed hexadecimal parameter |
| Actual speed of right wheel | 55 | 0x37 | 0x03 | Signed hexadecimal parameter |
| Actual mileage of Left wheel H | 56 | 0x38 | 0x03 | 32-bit unsigned pulse number |
| Actual mileage of Left wheel L | 57 | 0x39 | 0x03 |
| Actual mileage of right wheel H | 58 | 0x40 | 0x03 | 32-bit unsigned pulse number |
| Actual mileage of right wheel L | 59 | 0x41 | 0x03 |
| Displacement loop control status | 60 | 0x3C | 0x03 | State when detecting the displacement ring  1. 0x01: running  2. 0x00: Before or after running |
| High and low beam control | 61 | 0x3D | 0x03,0x06 | 1. 0x01 Turn off the lights  2. 0x02 low beam  3. 0x03 high beam |
| Radar 1 | 62 | 0x3E | 0x03 | Range: 25-250cm  0: Sensor failure  255: Accessible or not detected |
| Radar 2 | 63 | 0x3F | 0x03 | Range: 25-250cm  0: Sensor failure  255: Accessible or not detected |
| Radar 3 | 64 | 0x40 | 0x03 | Range: 25-250cm  0: Sensor failure  255: Accessible or not detected |
| Radar 4 | 65 | 0x41 | 0x03 | Range: 25-250cm  0: Sensor failure  255: Accessible or not detected |
| Radar 5 | 66 | 0x42 | 0x03 | Range: 25-250cm  0: Sensor failure  255: Accessible or not detected |
| Radar 6 | 67 | 0x43 | 0x03 | Range: 25-250cm  0: Sensor failure  255: Accessible or not detected |
| Radar 7 | 68 | 0x44 | 0x03 | Range: 25-250cm  0: Sensor failure  255: Accessible or not detected |
| Radar 8 | 69 | 0x45 | 0x03 | Range: 25-250cm  0: Sensor failure  255: Accessible or not detected |
| Radar Control Mode | 70 | 0x46 | 0x03,0x06,0x10 | 1.0: Cancel the radar control mode  1: Start the radar control mode |
| Error code output | 71 | 0x47 | 0x03 | 1: 0x00: normal without abnormalities |
| Device ID | 512 | 0x200 | 0x03,0x06 | 1:0x01--0xFE( 1-255)  0xff is the broadcast address |

1. 485 interface remote control mode

1. Two-dimensional joystick data input (X, Y) corresponds to register address (0x0A, 0X0B)

2. Input range: (800----(-800)) origin (0,0)

3. The refresh rate is not more than 1 second interval (recommended 50MS)

4. If the data interaction interval exceeds 1S, the program executes the communication interruption judgment mechanism

5. Feedback real-time speed linear speed (5600---(-5600)) positive and negative sign represents direction

6. Speed ​​feedback value (left wheel, right wheel) corresponding register (0x36, 0x37)

2. Speed closed-loop control mode

1. Speed data input (L left wheel, R right wheel) corresponds to the register address (0X0C, 0X0D)

2. Input range: (5600---(-5600)) origin (0,0)

3. The refresh rate is not more than 1 second interval (recommended 50MS)

4. If the data interaction interval exceeds 1S, the program executes the communication interruption judgment mechanism

5. Feedback real-time speed line speed 0-5600 line speed, the speed has no positive or negative distinction

6. Speed ​​feedback value (left wheel, right wheel) corresponding register (0x36, 0x37)

Device ID range: 0x01--0xFE(1-254) 0XFF is the broadcast address

When the user forgets the ID, the slave ID of the device can be retrieved through the broadcast address

Speed closed-loop control:

Unit: line/second

Accuracy: 20 lines

Range: 600-5600 (forward is positive input, backward is negative input) minimum speed is 600 lines/second

Configuration parameters: speed loop mode, speed value real-time input update

Write input speed frequency: 1HZ (continuous input) Stop when data is interrupted, and the speed can be adjusted during driving

Position closed-loop control:

Speed parameters are the same as speed closed loop

Displacement parameters:

unit: line

Range: 200-100000 lines

Accuracy: 100 lines

Configuration parameters: displacement loop mode, speed value, displacement value, operation start-stop mode operation after update

After the end of the displacement loop, the real-time displacement value is cleared, and the parameters need to be repeatedly configured before executing the vehicle control. During the driving process, the displacement and speed cannot be modified, and the start-stop function can be controlled in real time.

Read input displacement frequency 1hz

Note: After the parameter is updated, after writing the start command, the start-stop command key needs to be read at a reading frequency of 1hz. It is used to support the heartbeat command of the displacement loop movement. If it exceeds, the stop will be cleared.

The speed of the product is unified according to line/second. According to the configuration of our company, the actual mileage of each encoder revolution is 0.51m (fixed value) Line/second is converted into m/S formula (collection speed/number of encoder lines)\*0.51

The actual speed is related to the number of lines of the encoder

Note: Radar sensor is the default function (customer optional)