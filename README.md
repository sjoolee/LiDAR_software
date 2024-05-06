## LiDAR_software
#Device Overview
Features
The circuitry consists of two external push buttons, two on-board LEDs on the microcontroller, the stepper motor and the ToF sensor to capture 360 degrees distance data using the push buttons to control the circuit functions.
MSP432E401Y microcontroller
Acts as the common communication point between the PC and ToF sensor
Transmits live data from the ToF sensor to the microcontroller using I2C communication (100 kbps for I2C communication)
Transmits live data from the microcontroller to the computer using UART communication (115200 bps baud rate for UART communication)
3.3V operating voltage
32-bit registers
30MHz bus speed
LED measurement status = PF0
LED additional status = PN0
VL53LIX Time of Flight (ToF) Sensor
50 Hz operating frequency
Accurate long distance range
Range up to 360cm
Stepper Motor
4 phases per step
MATLAB
3D plot generation using the saved distance data from the ToF
Reads the data, plots points and reconstructs data in 3D


General Description
The circuit design is an embedded system meant to collect the distance data of a room or hall and reconstruct it in a 3D digital rendering in an easy, and low-cost manner. The circuitry is made up of the MSP432E401Y microcontroller, ToF sensor, stepper motor and two external push buttons. When the ToF sensor captures data on the yz plane every 11.25 degrees, the on-board measurement LED will blink, displaying to the user that the distance measurement is being read. To start or stop the full 360 degree rotation, press the first push button, then the second button should be pressed to initiate the distance measurement capture. By pressing the second push button, it should toggle the on-board additional status LED, visually demonstrating that the measurement mode is on. Only when the second button is pushed, the system would transmit the scanned data to the computer. However, when the second button is pressed while it is scanning, it will cancel the data scan and return the ToF sensor to its original position. To avoid tangling the wires, after the ToF sensor has completed a full scan, it will rotate back to its home position in the opposite direction. 

The ToF sensor measures distances data by emitting a ray of infrared light then measuring the time it takes for the light to hit an object and return back to the sensor. The measured distance can be calculated by the photon’s travel time times the speed of light divided by 2. Once the sensor has the data, using I2C communication it sends that data to the microcontroller and then sends it to the computer/PC via UART. The data is read and then manipulated in MATLAB to create a 3D rendering via the MATLAB plot3() function. Using trigonometry, the data is converted from polar form into cartesian, and extracts the Y and Z data to form an YZ plane with its individual X coordinates, developing the full 3D extraction of the room or hallway. 


Block Diagram

Figure 1. The block diagram of the physical circuitry



Device Characteristics Table
Table 1. Device Characteristics 
Stepper Motor
Microcontroller
Time of Flight Sensor
V+
5V
Measurement LED
PF0
Vin
3.3V
V-
GND
Additional Status LED
PN0
GND
GND
IN1
PH0
Bus Speed
30 MHz
SCL
PB2
IN2
PH1
Baud Rate
115200 bps
SDA
PB3
IN3
PH2
Serial Port
COM6 (but can be changed by user)
Measurement
Takes measurement data every 11.25 degrees
IN4
PH3
GPIO Ports
PB2-3, PL0-1, PH0-3
Captured Distance
360cm (in dark, ideal conditions)

Detailed Description
Distance Measurement
The embedded system used for spatial mapping utilizes the VL53LIX Time of Flight sensor to acquire the required data. It operates by emitting a ray of infrared light at 940nm using its “emitter” and times how long it takes for it to reflect off an object and return back to the sensor’s “receiver”. The mathematical calculation can be described by the photon’s travel time times the speed of light divided by 2. The distance measured is recorded in millimeters, and once the sensor acquires the data, it performs transduction, conditioning and Analog to Digital Conversion to produce a digital value. Then, using I2C communication, it sends that data to the microcontroller and then sends it to the computer/PC via UART. 

The ToF sensor should ideally work in dark environments with a 360 cm measuring distance limit. To initiate the scanning of the ToF sensor, the ToF sensor boot function is invoked within the Kiel programming. The ToF sensor boot activates the sensor and prepares it to take distance measurements, but for it to fully activate it must be triggered by the external push buttons which acts as peripherals. It is the polling method that is used to trigger the sensor, meaning that the microcontroller checks the status of the buttons at constant intervals to determine which button is pressed and when. The first push button controls the stepper motor and starts the rotation process to spin 360 degrees. It will also blink the measurement status LED on the microcontroller whenever the ToF sensor takes a measurement. The second push button starts the measurement process and toggles on the additional status LED to let the user know that the measurement mode is in progress.

The ToF sensor is attached to the stepper motor using a custom 3D printed holder, enabling the sensor to rotate 360 degrees and scan the frontal plane. Once it is done recording the data, the stepper motor will rotate 11.25 degrees. It rotates by this value since a full rotation or 360 degrees is made of 512 steps, which means that for every step, it must rotate approximately 0.703 degrees to make a full rotation. So mathematically, we can calculate it as 11.25 degrees by 0.703 which equals 16 steps for every 11.25 degrees rotated. After each measurement is taken, the motor will move 11.25 degrees, positioning the ToF at a new angle until the stepper motor has rotated a full 360 degrees. If the second external push button is pressed during the measurement process it will cancel the function and the stepper motor will return back to its original position.

Visualization
The data measured by the ToF sensor is read and then manipulated in MATLAB to create a 3D rendering using the MATLAB plot3() function. The ToF sensor uses I2C communication to send the data to the microcontroller, then the microcontroller sends it to the PC using UART communication at a baud rate of 115200 bps. MATLAB communicates with the microcontroller using serial communication via the COM6 port that is connected to the computer. Within the MATLAB code there is a function that monitors a serial port, awaiting a completed string line, which ends with a “\r \n” character. The “Starting to measure” message identifies that the ToF sensor is measuring data and will send the data to the PC indicating the degrees of change per measurement. The incoming measurement data is converted from string format into double data format and is stored in a matrix. This process continues until the system exits measurement mode which is signalled by the “Ending measurement process” message.
The data is read and then manipulated in MATLAB to create a 3D rendering via the MATLAB plot3() function. Using trigonometry equations, the data is converted from polar form into cartesian, and extracts the Y and Z data to form an YZ plane, with the y-axis being the vertical axis and z-axis being the horizontal. The trigonometric equations are identified as: y = distance * sin(angle) and z = distance * cos(angle). If we set the YZ planes to be 30 cm apart from each other, and if it is assumed that each frontal plane has a constant X value, a plane can be developed if each plane is spaced 30 cm apart in the x-axis. Therefore, the 3D plane is developed as long as the user moves 30 cm in the x-axis to capture the frontal plane measurements. Then the data in the matrix can be converted into spatial coordinates to develop the full 3D extraction of the room or hallway. 


Application
LiDAR, also known as light detection and ranging, is a type of technology that is utilized in numerous fields and industries. By utilizing laser light to measure distances, LiDAR can develop detailed 3D renderings of environments, and can be applied in most smart system softwares. A common application of LiDAR is often in the autonomous vehicles and advanced driver-assistance systems. Most smart cars have features such as lane centering, where LiDAR sensors are used to ensure the vehicles stay within a lane by detecting lane markings and monitoring the vehicle’s position relative to them. It can also aid in obstacle detection and avoidance, by generating a small noise when it detects obstacles such as other vehicles, pedestrians or cyclists that are moving too close to the vehicle as a warning mechanism. However, LiDAR technology also helps with natural disaster management, urban planning and infrastructure, archeology mapping, as well as forestry/ agriculture. 


Instructions
Assuming the user has all the necessary programs downloaded and ready:
Connect the microcontroller to your computer and search for Device Manager in the computer’s Control Windows key (for a Windows PC) or a Control Command (for a MAC PC). Once the Device Manger menu is open, click the Ports (COM & LPT) to expand and look for the x value in XDS110 Class Application/ User UART (COM x)
In the MATLAB code, search near the top for the line:  s = serialport("COM#",115200);
Change the COM# to the COM value you found previously in step 1
Ensure circuit is configured with the data found in the Device Characteristics table
Open the Kiel project and click Translate → Build → Load 
Run the MATLAB code
Reset microcontroller using the reset button and the "Please Wait ToF Booting!” message will pop up, followed by a message asking which push button the user would like to press
After pressing the first push button, a  “Starting to measure” message will pop up on the MATLAB terminal and the on-board additional status LED D3 will turn on
Ensure you are at the beginning of the hallway or room with the circuitry oriented that the stepper motor and ToF sensor are aligned with the x-axis
Press the second push button to start the yz plane scan. After rotating 360 degrees, the stepper motor will return the ToF sensor back to its home position and the data recorded should be printed. The on-board measurement status LED D4 will blink for each measurement record at 11.25 degrees. 
If you click on the external button 2 while recording, it will cancel the measurement process and return the ToF sensor back to its initial position
Once you are done recording the 360 degree scan, move 30 cm forward 
Repeat Steps 9 and 10 until the end of the room/ hallway is reached. Then press the external push button 1 to complete the measurement process and graph the 3D rendering of the environment. If this is successful, the user will see a “Ending measurement process” and afterwards a “3D Map Developing” message
The 3D map will pop up as a window and can be further explored using the MATLAB pan, zoom and rotate tools seen at the top left corner


Expected Output
My assigned scan location was in section I, which can be seen in Figure 2, while the 3D rendering of the hallway can be seen in Figure 3. 

Figure 2. Picture of the assigned Section I at the start position

Figure 3. The 3D scan of Section I going along the x-axis

When the measurements were taken, there were bright lights from the overhead lights and a largely lit screen as you moved further into the hall. There were also walls that were outside the 360 cm limit as it led to more hallways/ classroom entrances, as well as multiple window glass in the surrounding environment. The environment was not ideal but the results were able to display the key features of the hallway. 
Figure 3 demonstrates the output of the circuitry for the assigned hallway seen in Figure 2. From the 3D rendering of the image, a darker line running along the x-axis is seen, indicating that plane to be the ceiling. This can be assumed because the ToF sensor started the measurement process pointing upwards in all its scans. The assigned hallway had some irregular contortions like the benches lining on the left side of the wall, pillars that were located on either side at different positions, and a small hallway that extended to the entrance of a lecture room on the right side near the starting position. The pillars can be seen in Figure 3, by the indentations seen on the left and right side of the 3D image. A displacement of the x-axis of around 75, can also be seen on the right side of Figure 3, which was the approximate location to the entrance hallway of a lecture room. Since the walls near this location were more than 360 cm away from the circuitry, the output displayed less accurate lines due to the limitations of the ToF sensor. On the left side of the 3D rendering, there was some inconsistency in specific areas displayed as small spikes, which could have been caused due to the windows near that location. 

Limitations
Floating Point
When the data is being plotted in MATLAB, it uses the trigonometric functions, sine and cosine, to convert the distance measurements to spatial coordinates. This results in large decimal values that may have extensive repetition. To address this, the MSP432E401Y microcontroller integrates 32x32-bit single precision registers within its Arithmetic Logic Unit (ALU), forming the Floating Point Units (FPU) and allowing for internal handling of floating-point operations. The microcontroller’s single-precision floats store a sign bit, 8 bits for the exponent and 23 bits for the significand. However, this limitation does not impact the functionality as the microcontroller can manage angle calculations which typically remain within the 23-bit limit.

Trigonometric Functions
When MATLAB utilizes trigonometric functions it can have errors based on the software’s calculation precision. Slight errors can be found when numbers are rounded close to 0, instead of equaling to an exact 0 value. For example, if sin(deg2rad(180)) is being calculated, MATLAB will calculate it to equal a non-zero value when it should be exactly equivalent to a zero value. While this is not a huge error for this circuitry’s purpose, it may prove to be an issue if this design were to be upgraded to a more large-scale version. 

ToF Sensor Quantization Error
The maximum quantization error of a system can be calculated by using the equation Max Quantization Error = Max Reading ÷ 2 number of ADC bits. Since the maximum reading of the ToF sensor is 360 cm and the ToF sensor uses a built-in 16-bit Analog to Digital converter, we can solve the maximum quantization error to be 3600 mm ÷  216  = 0.055 mm. This value is the quantization error when the analog distance value is converted to a digital representation. 

Circuit Schematic

Figure 4. The Circuit Schematic of the different devices (note: the PB1 and PB2 seen connected to the 10k resistors stand for Push Button 1 and 2, and not GPIO Port B which is not related to the button circuitry)






Programming Logic Flowcharts

Figure 4. The Kiel Logic Flowchart



Figure 5. The MATLAB Logic Flowchart
