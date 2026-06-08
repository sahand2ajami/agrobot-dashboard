microROS Communication Protocol Specification

Document Version: V1.0

Compilation Date: 2026.01.20

Applicable Object: AFD Chassis Robot (ROS2 Module + Modbus RTU Chassis + microROS Host Communication)

Document Description: This document is compiled entirely based on the actual running program, including complete content such as hardware connection, software environment, full ROS2 communication interfaces, test commands, and troubleshooting.

1. **Document Introduction & Scope of Application**
   1. **Document Purpose**

This document is the standard communication protocol for Avatar Chassis Robot's Modbus to microROS (ROS2) conversion. It is used for the docking and debugging between ROS2 developers of the host computer and chassis hardware. It clarifies all ROS2 communication rules, data formats, functional interfaces, hardware connection requirements, host computer environment configuration, and test commands between the chassis and the host computer. All content is completely consistent with the actual running program of the chassis, with no redundant information, and can be directly used as the development basis and acceptance standard.

* 1. **Function Overview**

The core logic of the chassis hardware is "underlying Modbus RTU protocol collection/control" + "upper-layer microROS (ROS2) standard communication forwarding":

The main controller of the chassis ROS2 module communicates with the chassis motor driver, power management module, and sensor module through Modbus RTU (485 bus) to complete hardware functions such as chassis speed control, light control, odometer collection, battery voltage collection, fault status collection, and odometer reset.

The ROS2 module communicates with the ROS2 host computer through the microROS protocol, encapsulates the chassis hardware data collected by Modbus into ROS2 standard topics and publishes them to the host computer; at the same time, it receives ROS2 standard topic commands issued by the host computer, parses them, and issues them to the chassis hardware for execution through Modbus; it also provides ROS2 standard service interfaces to respond to hardware control requests from the host computer.

Core Implementation: The host computer only needs to develop based on ROS2 standard interfaces, without paying attention to the details of the underlying Modbus protocol. All hardware interactions are automatically completed by the chassis ROS2 for protocol conversion.

* 1. **Core Agreements**

All ROS2 communication interfaces (topics/services) adopt ROS2 official standard **std\_msgs** message types and **std\_srvs** service types, without the need to customize msg/srv files. The host computer can directly call them without compilation dependencies.

All topic/service names uniformly use **/avatar\_robot/** as the namespace to avoid naming conflicts in the ROS2 system.

All data interactions are protected by thread safety, with no risk of data read/write confusion. The host computer does not need additional data verification.

All control commands include hardware-level timeout safety protection to prevent chassis loss of co ntrol after the host computer is disconnected.

All content of this protocol corresponds one-to-one with the actual running program of the chassis. All parameters, addresses, and command values can be directly reused.

1. **Hardware Connection Instructions (Reserved)**
2. **Host Computer microROS (ROS2) Operating Environment Installation and Configuration (Complete and Detailed Version)**
   1. **Core Environment Description**

The chassis uses the microROS protocol, which is essentially a lightweight ROS2 running on embedded devices. The host computer must install the ROS2 system + microROS Agent proxy software. The microROS Agent is the only communication bridge between the ROS2 and the host computer ROS2. The microROS data of the ROS2 is transparently transmitted to the Agent through the USB serial port, and then forwarded to the host computer ROS2 system by the Agent. Both are indispensable.

* 1. **Host Computer Hardware Requirements**

System: Ubuntu 20.04 LTS or Ubuntu 22.04 LTS (Officially recommended by ROS2, with the best compatibility)

Memory: ≥8G, Hard Disk: ≥50G available space

Must have a USB port for connecting the ROS2 main control board

* 1. **Host Computer Software Installation Steps (Complete and Copyable, Users Can Execute Directly)**

### Step 1: Install ROS2 Distribution (Recommended: Humble Hawksbill)

ROS2 Humble is compatible with Ubuntu 22.04, is a long-term maintenance version, and has no compatibility issues. Execute the following commands to complete the installation:

#### #5. Verify Successful ROS2 Installation

ros2 --version

### ✅ Verification Success: Print the ROS 2 Humble version information.

### Step 2: Install microROS Agent Proxy Software (Core and Essential)

The microROS Agent is the communication proxy between the ROS2 and the host computer ROS2 and must be installed. Execute the following commands to complete the installation (binary installation, fastest and most stable, no compilation errors):

# 1. Install dependencies sudo apt update && sudo apt install -y ros-humble-micro-ros-agent

# 2. Verify successful installation

ros2 run micro\_ros\_agent micro\_ros\_agent --version

✅ Verification Success: Print the microROS Agent version information.

### Step 3: Install Additional Debugging Dependencies (Optional, Recommended)

# Install auxiliary tools for ROS2 topic viewing and service calling sudo apt install ros-humble-rqt ros-humble-rqt-topic ros-humble-rqt-service-caller -y

microROS Agent Startup Command (Must Be Executed on the Host Computer, Core Command)

After the host computer is connected to the ROS module, the microROS Agent must be started first; otherwise, the ROS2 cannot establish communication with the ROS2 host computer, and the ROS2 will continuously print the "Agent not found" log. The startup command is as follows:

# Core startup command (adapted to ROS2 USB virtual serial port, baud rate 115200, consistent with the program)

ros2 run micro\_ros\_agent micro\_ros\_agent serial --dev /dev/ttyACM0 -b 115200

Key Notes:

Serial Port Device Name /dev/ttyACM0: The default device name of the ROS2's USB virtual serial port. If it is recognized as /dev/ttyACM1, modify it directly; the actual serial port name can be viewed through the ls /dev/ttyACM\* command.

Baud Rate -b 115200: Must be consistent with Serial.begin(115200) in the ROS2 program; otherwise, communication will fail.

Agent Startup Success Mark: The terminal prints "micro-ROS agent running" without errors.

ROS2 Connection Success Mark: The Agent terminal prints "Client connected", and the ROS2 serial port prints "[ROS] Agent connected!".

**3.4 Host Computer Environment Verification**

After the Agent is started, execute the following command. If the /avatar\_robot node can be viewed, it indicates that the environment configuration is successful and the communication link is normal:

ros2 node list

1. **ROS2 Node Basic Information (Fully Matching the Program, Core)**
   1. **Basic Node Attributes**

Node Name: avatar\_robot (Hard-coded in the program, can’t be modified)

Node Namespace: /avatar\_robot

Node Running Carrier: ROS2 Module (microROS Client)

Core Node Roles:

Publisher: Encapsulates the hardware data (battery voltage, odometer, fault code) collected by the chassis ROS2 from the chassis hardware through Modbus RTU into ROS2 standard topics and actively publishes them to the host computer periodically.

Subscriber: Real-time monitors the ROS2 control topics (speed commands, light commands) issued by the host computer, parses them, and issues them to the chassis hardware for execution through Modbus.

Server: Provides ROS2 standard service interfaces to respond to synchronous call requests from the host computer (odometer reset), executes hardware-level commands, and returns execution results.

**4.2 Node Status View Commands (Essential for Host Computer Testing)**

# Check if the node is online

ros2 node list

# View complete node communication information (full list of publications/subscriptions/services)

ros2 node info /avatar\_robot

✅ Normal Return: The node information includes "3 published topics, 2 subscribed topics, 1 service interface", which is completely consistent with this document.

1. **ROS2 Published Topics (Chassis → Host Computer: Hardware Data Reporting)**

Function Description: The chassis ROS2 collects data from the chassis hardware through Modbus RTU, encapsulates it into ROS2 standard topics, and actively publishes them to the host computer periodically. The host computer can obtain all real-time status data of the chassis by subscribing with ros2 topic echo; all publishing logic is executed in the vTaskMicroROS task of the ROS2, and the data source is real hardware collection without false data.

Core Agreement: The frequency, data type, and data range of all published topics are solidified configurations in the program. The host computer can directly subscribe without adaptation.

**5.1 Battery Voltage Reporting Topic**

Topic Full Name: /avatar\_robot/battery

Message Type: std\_msgs/msg/Float32 (ROS2 standard single-precision floating-point type)

Publishing Frequency: 1Hz (1000ms/time, configured by the last\_battery\_time timer in the program)

Data Source: Read the chassis power module register (register address 25+4) through Modbus, and the original value ÷ 100 to get the actual voltage.

Data Meaning: Real-time voltage value of the chassis power battery.

Data Unit: Volts (V)

Data Range: 0.0 V ~ 60.0 V

Data Precision: Keep 2 decimal places

Host Computer Subscription Command:

ros2 topic echo /avatar\_robot/battery std\_msgs/msg/Float32

Data Example (Received by the Host Computer):

data: 24.18

---

Note: If the voltage value is 0.0, it indicates that Modbus communication is abnormal, and the chassis has not collected battery data.

**5.2 Wheel Odometer Reporting Topic**

Topic Full Name: /avatar\_robot/wheel\_odom

Message Type: std\_msgs/msg/Int32MultiArray (ROS2 standard signed 32-bit integer array)

Publishing Frequency: 20Hz (50ms/time, configured by the last\_wheel\_time timer in the program, high-frequency and real-time)

Data Source: Read the odometer register of the chassis motor driver (register address 25+0~25+3) through Modbus, and splice 4 bytes into a 32-bit pulse count.

Data Format: The array has 2 fixed elements, which cannot be increased or decreased, and the order cannot be changed.

data[0]: Left wheel cumulative odometer pulse count (int32\_t)

data[1]: Right wheel cumulative odometer pulse count (int32\_t)

Data Meaning: The cumulative encoder pulse count of the left and right wheels since power-on/reset. The mapping relationship between the pulse count and the actual distance is determined by the chassis hardware.

Data Range: -2147483648 ~ 2147483647 (maximum value of int32\_t, no overflow risk)

Host Computer Subscription Command:

ros2 topic echo /avatar\_robot/wheel\_odom std\_msgs/msg/Int32MultiArray

Data Example (Received by the Host Computer):

layout:

dim: []

data\_offset: 0

data: [12560, 12558]

---

Key Association: After calling the odometer reset service, the two data values of this topic will be set to 0 immediately.

**5.3 Chassis Fault Code Reporting Topic**

Topic Full Name: /avatar\_robot/error

Message Type: std\_msgs/msg/UInt16 (ROS2 standard unsigned 16-bit integer)

Publishing Frequency: 1Hz (1000ms/time, configured by the last\_error\_time timer in the program)

Data Source: Read the chassis fault detection module register (register address 25+5) through Modbus.

Data Meaning: Chassis hardware fault status code. 0 indicates no fault, and non-0 values indicate corresponding hardware faults.

Data Range: 0 ~ 65535 (maximum value of uint16\_t)

Fault Code Definition (Solidified in the program, can be expanded as needed, users can delete):

Refer to the fault codes of the adapted model for fault code reference.

Host Computer Subscription Command:

ros2 topic echo /avatar\_robot/error std\_msgs/msg/UInt16

Data Example (Received by the Host Computer):

data: 0

---

Note: When the fault code is non-0, the chassis will stop moving, and the host computer must handle the fault in a timely manner before issuing new commands.

**6. ROS2 Subscribed Topics (Host Computer → Chassis: Hardware Control Command Issuance)**

Function Description: The ROS2 module acts as a ROS2 subscriber, real-time monitoring the control topics issued by the host computer. After receiving the commands, it synchronizes them to thread-safe shared variables through mutex locks, and finally, the vTaskModbus task of the ROS2 uniformly issues them to the chassis hardware for execution through Modbus RTU; all control commands include a hardware-level 1-second timeout safety protection mechanism, which is the core safety logic of the program and must be followed by the host computer.

Core Agreement: The data type and format of the commands issued by the host computer must be completely consistent with this protocol; otherwise, the chassis will discard invalid commands without any response; all commands are "overwriting type", and new commands will immediately replace old ones.

**6.1 Light Control Command Topic**

Topic Full Name: /avatar\_robot/light\_cmd

Message Type: std\_msgs/msg/UInt8 (ROS2 standard unsigned 8-bit integer)

Receiving Method: Real-time reception, no cache, new commands overwrite old ones.

Data Meaning: Control commands for the chassis light module. Single-byte values correspond to different light modes.

Command Value Definition (Solidified in the program, can be expanded as needed, users can delete):

0x00: Turn off all lights (default value, automatically set to 0 after timeout)

0x01: Turn off all lights (default value, automatically set to 0 after timeout)

0x02: Turn on chassis high beam lights

0x03: Turn on low beam lights

Core Safety Rules **[Must Follow]**: 1-second command timeout, automatically set to 0 → The program counts through light\_callback\_count. If the host computer does not issue any light commands for more than 1 second, the chassis will automatically set the light control variable light\_cmd=0 and turn off all lights; this logic is a hard-coded safety mechanism and cannot be turned off. The purpose is to prevent the lights from staying on after the host computer is disconnected.

Command Issuance Logic: The host computer must issue commands periodically (recommended 10Hz) to avoid triggering the timeout mechanism; a single command can only take effect for 1 second.

Host Computer Test Command Issuance (Complete and Copyable):

# Single send: Turn on constant lights

ros2 topic pub --once /avatar\_robot/light\_cmd std\_msgs/msg/UInt8 "{data: 1}"

# Continuous send (recommended to avoid timeout): Send warning light command at 10Hz, never timeout

ros2 topic pub -r 10 /avatar\_robot/light\_cmd std\_msgs/msg/UInt8 "{data: 2}"

# Single send: Turn off all lights

ros2 topic pub --once /avatar\_robot/light\_cmd std\_msgs/msg/UInt8 "{data: 0}"

**6.2 Left and Right Wheel Speed Control Command Topic (Core Control Command of the Chassis)**

Topic Full Name: /avatar\_robot/speed\_cmd

Message Type: std\_msgs/msg/Int16MultiArray (ROS2 standard signed 16-bit integer array)

Receiving Method: Real-time reception, no cache, new commands overwrite old ones.

Data Format: The array has 2 fixed elements, which cannot be increased or decreased, the order cannot be changed, and neither can be missing.

data[0]: Left wheel speed set value (int16\_t)

data[1]: Right wheel speed set value (int16\_t)

Core Speed Value Rules (Natively adapted in the program, no overflow risk):

Direction Definition: Positive number = chassis forward, negative number = chassis backward, 0 = corresponding wheel stop.

Value Range: -32767 ~ +32767 (standard value range of int16\_t, received by int16\_t variables in the program, no data overflow).

Speed Mapping: The value is proportional to the wheel speed. The larger the value, the faster the speed. The specific mapping relationship is determined by the chassis Modbus motor driver (no need for the host computer to pay attention).

Emergency Stop Command: data: [0, 0] is the highest priority command. After issuance, the chassis will stop all movements immediately.

Core Safety Rules **[Must Follow]**: 1-second command timeout, automatically set to 0 → The program counts through speed\_callback\_count. If the host computer does not issue any speed commands for more than 1 second, the chassis will automatically set the left and right wheel speed variables left\_speed=0 and right\_speed=0, and the chassis will emergency stop immediately; this logic is a hard-coded safety mechanism and cannot be turned off. It is the core guarantee to prevent chassis loss of control after host computer disconnection, network abnormalities, or program crashes.

Command Issuance Logic: The host computer must issue commands periodically (recommended 20Hz) to avoid triggering the timeout mechanism; a single command can only take effect for 1 second, and the emergency stop command can be sent once.

Host Computer Test Command Issuance (Complete and Copyable, High-Frequency and Commonly Used, Users Can Directly Use):

# Continuous send (recommended 20Hz): Chassis forward, left and right wheel speeds are both 500

ros2 topic pub -r 20 /avatar\_robot/speed\_cmd std\_msgs/msg/Int16MultiArray "{data: [500, 500]}"

# Continuous send: Chassis turn left in place, left wheel -300, right wheel +300

ros2 topic pub -r 20 /avatar\_robot/speed\_cmd std\_msgs/msg/Int16MultiArray "{data: [-300, 300]}"

# Continuous send: Chassis turn right in place, left wheel +300, right wheel -300

ros2 topic pub -r 20 /avatar\_robot/speed\_cmd std\_msgs/msg/Int16MultiArray "{data: [300, -300]}"

# Continuous send: Chassis backward, left and right wheel speeds are both -400

ros2 topic pub -r 20 /avatar\_robot/speed\_cmd std\_msgs/msg/Int16MultiArray "{data: [-400, -400]}"

# Single send: Chassis emergency stop (highest priority, takes effect immediately)

ros2 topic pub --once /avatar\_robot/speed\_cmd std\_msgs/msg/Int16MultiArray "{data: [0, 0]}"

Chassis Response: After receiving the command, the ROS2 serial port prints "[Speed] Callback - L:X R:X", and issues it to the motor driver for execution through Modbus. The chassis moves according to the command.

**7. ROS2 Service Interfaces (Host Computer → Chassis: Synchronous Hardware Function Call)**

Function Description: The ROS2 module acts as a ROS2 server, providing 1 core synchronous service interface based on the ROS2 standard std\_srvs/srv/Empty empty service type (no request parameters, no response parameters). After the host computer calls this service through ros2 service call, the chassis will immediately execute the corresponding hardware-level operation and return the execution result (log printing); this service is a synchronous blocking call, and the call is successful only when the chassis completes the operation, with no delay.

Core Optimization: The underlying Modbus commands of this service have been processed with task exclusivity, without any conflict with the chassis Modbus task. The command issuance success rate is 100%, solving the 485 command loss problem of the original program.

**7.1 Odometer Reset Service (Core Function Service of the Chassis)**

Service Full Name: /avatar\_robot/reset\_position

Service Type: std\_srvs/srv/Empty (ROS2 standard empty service, no request parameters, no response parameters)

Service Call Method: Synchronous blocking call. After the host computer initiates the call, it waits for the chassis to complete the execution and returns success.

Core Function: One-click reset of the left and right wheel odometer data of the chassis, including two layers of reset logic (fully implemented in the program, neither can be missing):

Software-Level Reset: The global odometer variables left\_odom=0 and right\_odom=0 of the chassis ROS2 module are atomically set to zero, with no data confusion.

Hardware-Level Reset: The ROS2 module writes the value 0x0001 to register 31 of the chassis motor driver through Modbus RTU, triggering hardware-level odometer reset, and the hardware register value is set to zero.

Call Timing: The host computer calls it during chassis power-on initialization, when reaching a specified position, after task completion, or when needing to reset the odometer reference.

Host Computer Call Command (Complete and Copyable, Core Command):

ros2 service call /avatar\_robot/reset\_position std\_srvs/srv/Empty

Successful Call Return (ROS2 Standard Response, Host Computer Terminal):

requester: making request: std\_srvs.srv.Empty\_Request()

response:

std\_srvs.srv.Empty\_Response()

**8. Complete Test Command Set of the Host Computer (Summary Version, Can Be Copied Directly, Essential for User Debugging)**

**8.1 Basic Status Query Commands**

# View all online ROS2 nodes

ros2 node list

# View complete communication information of the avatar\_robot node

ros2 node info /avatar\_robot

# View all topic lists published/subscribed by the chassis

ros2 topic list

# View all service lists provided by the chassis

ros2 service list

**8.2 Data Reporting Test (Subscribe to All Chassis Statuses)**

# Real-time view battery voltage

ros2 topic echo /avatar\_robot/battery std\_msgs/msg/Float32

# Real-time view left and right wheel odometers

ros2 topic echo /avatar\_robot/wheel\_odom std\_msgs/msg/Int32MultiArray

# Real-time view chassis fault codes

ros2 topic echo /avatar\_robot/error std\_msgs/msg/UInt16

**8.3 Topic Frequency Verification Commands (Verify Whether Chassis Publishing Is Normal)**

# View odometer publishing frequency (should be 20Hz)

ros2 topic hz /avatar\_robot/wheel\_odom

# View battery voltage publishing frequency (should be 1Hz)

ros2 topic hz /avatar\_robot/battery

**8.4 Control Command Test (Complete Combination, Users Can Directly Use)**

# Combination 1: Chassis forward + turn on lights + real-time view odometer

ros2 topic pub -r 20 /avatar\_robot/speed\_cmd std\_msgs/msg/Int16MultiArray "{data: [60, 60]}"

ros2 topic pub -r 10 /avatar\_robot/light\_cmd std\_msgs/msg/UInt8 "{data: 1}"

ros2 topic echo /avatar\_robot/wheel\_odom std\_msgs/msg/UInt32MultiArray

# Combination 2: Chassis emergency stop + turn off lights + odometer reset (Essential after task completion)

ros2 topic pub --once /avatar\_robot/speed\_cmd std\_msgs/msg/Int16MultiArray "{data: [0, 0]}"

ros2 topic pub --once /avatar\_robot/light\_cmd std\_msgs/msg/UInt8 "{data: 0}"

ros2 service call /avatar\_robot/reset\_position std\_srvs/srv/Empty

**9. Common Troubleshooting and Solutions (Complete and Detailed, Users Can Directly Refer to)**

**9.1 Communication Faults (Most Common, Highest Priority)**

Fault 1: The host computer executes ros2 node list but there is no /avatar\_robot node

Phenomenon: The ROS2 serial port prints "[ROS] WARNING: Agent not found after 30 attempts!".

Causes: 1. microROS Agent not started; 2. USB connection between ROS2 and host computer disconnected; 3. Serial port baud rate mismatch; 4. Incorrect serial port device name.

Solutions:

Restart the microROS Agent with the command: ros2 run micro\_ros\_agent micro\_ros\_agent serial --dev /dev/ttyACM0 -b 115200

Re-plug the ROS2 USB cable to ensure good contact.

Confirm that the baud rate is 115200, consistent with the program.

Use ls /dev/ttyACM\* to view the actual serial port name and modify the device name in the Agent command.

Fault 2: After the host computer issues a command, the chassis has no response

Phenomenon: The host computer successfully sends the command, but the ROS2 serial port does not print any logs, and the chassis does not move/the lights do not change.

Causes: 1. Mismatched command data type; 2. Incorrect command format (e.g., insufficient 2 elements in the speed command array); 3. The chassis triggers the timeout mechanism.

Solutions:

Issue commands strictly according to the message types in this protocol; prohibit type mismatch.

The speed command must be in the format data: [X, X], with two elements indispensable.

The host computer must continuously issue commands with -r 10/-r20 to avoid 1-second timeout.

**9.2 Functional Faults**

Fault 1: The odometer reset service call is successful, but the odometer value is not reset

Phenomenon: The host computer returns a successful service call, and the ROS2 serial port prints a Modbus error code.

Causes: 1. Incorrect Modbus register address for odometer reset; 2. Incorrect wiring of the 485 bus; 3. The chassis motor driver does not respond to the reset command.

Solutions:

Verify the odometer reset register address of the chassis motor driver and modify the address in the ROS2 program.

Check if the 485 A/B wires are reversed and rewire them.

Restart the chassis motor driver and call the service again.

Fault 2: The odometer value does not change when the chassis is moving

Phenomenon: The chassis moves normally, but the value of the wheel\_odom topic subscribed by the host computer is always 0.

Causes: 1. Incorrect Modbus register address for reading the odometer; 2. Motor encoder fault; 3. Modbus communication abnormality.

Solutions:

Verify the odometer register address of the chassis motor driver and modify the address in the ROS2 program.

Check if the motor encoder wiring is normal.

Check if the ROS2 serial port prints Modbus read failure logs and troubleshoot 485 communication.

Fault 3: The chassis triggers the timeout mechanism, and the command stops after taking effect for 1 second

Phenomenon: After the host computer issues a single command, the chassis executes it for 1 second and then automatically stops/turns off the lights.

Cause: The host computer does not continuously issue commands, triggering the 1-second timeout safety mechanism in the program.

Solution: When the host computer issues commands, it must add the -r 10 or -r 20 parameter to send commands continuously and periodically.

**9.3 Modbus Communication Faults (Internal Chassis Communication)**

Phenomenon: The ROS2 serial port prints "[Modbus] Read failed X times" or "[Modbus] Write failed: X".

Causes: 1. Incorrect wiring of the 485 bus; 2. Incorrect Modbus slave address; 3. Mismatched baud rate; 4. Bus interference.

Solutions:

Check if the 485 A/B wires are reversed and if the common ground is connected.

Confirm that the Modbus slave address is 1, consistent with the program.

Confirm that the baud rate is 38400, consistent with the program.

Add a 485 terminal resistor to reduce signal reflection.

**10. Appendix (Can Be Deleted as Needed, Supplementary Explanation)**

**10.1 Modbus Core Register Mapping (Solidified in the Program, Users Can Refer to)**

Starting Address of Speed Control Registers: 22, Number of Registers: 3 (Left wheel speed, Right wheel speed, Light command)

Starting Address of Status Collection Registers: 25, Number of Registers: 6 (Left wheel odometer high 16 bits, Left wheel odometer low 16 bits, Right wheel odometer high 16 bits, Right wheel odometer low 16 bits, Battery voltage, Fault code)

Odometer Reset Register Address: 31, Written Value: 0x0001 (Trigger reset)

**10.2 Summary of Core Safety Mechanisms**

All control commands have 1-second timeout protection to prevent chassis loss of control after the host computer is disconnected.

All cross-task shared variables are protected by FreeRTOS critical sections, with no data read/write confusion.

Modbus commands are executed exclusively by independent tasks, with no multi-task conflicts, and the command issuance success rate is 100%.

Chassis fault code reporting is real-time, and the chassis automatically stops moving when a fault occurs to ensure hardware safety.

**10.3 Document Revision History**

V1.0 2026.01.20: Compiled based on the actual running program of the Avatar chassis robot, including hardware connection, environment configuration, full ROS2 communication interfaces, test commands, and troubleshooting. All content is completely consistent with the program, with no redundancy.