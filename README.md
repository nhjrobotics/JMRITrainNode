![image](https://github.com/user-attachments/assets/75c13b00-b55d-4264-bcbb-a3ae0a9e97ee)


# JMRITrainNode
TrainNode is a JMRI compatible I/O controller built on the MQTT protocol. Using the Raspberry Pi Pico, each node can support up to 16 turnouts as standard, or more with user addons.

## Installation
- To use TrainNode, you must have a MQTT server with username and password authentication connected to JMRI ([JMRI + MQTT](https://www.jmri.org/help/en/html/hardware/mqtt/index.shtml)), additionally you must have access to WIFI in close proximity to your layout for optimal performance. It is recommended to use Eclipse Mosquitto as your MQTT broker.
- To install TrainNode on the Rpi Pico, install MicroPython for Pico-W ([MicroPython](https://micropython.org/download/RPI_PICO_W/)).
- Next, configure the TrainNode [config.py](https://github.com/rewind2b4/JMRITrainNode/tree/master/Code/config.py) file (see Docs for more info). 
- Finally, copy the files from the [code](https://github.com/rewind2b4/JMRITrainNode/tree/master/Code) folder to the Pico.
- The Pico will automatically connect to the internet through the installation WIFI connection, download the required dependencies, and then restart, connecting to the layout MQTT broker through the layout WIFI connection (for layout networks that are disconnected from internet).

## Docs
Network / MQTT Settings
- Ssid (The WIFI connection used to access the MQTT server)
- Password (The WIFI connection used to access the MQTT server)
- Installation_Ssid (The WIFI connection used to access the internet for autoconfig)
- Installation_Password (The WIFI connection used to access the internet for autoconfig)
- Server_addr (The MQTT server address)
- Client_name (The MQTT client name)
- MQTT_User (The username to the MQTT server)
- MQTT_Password (The password to the MQTT server)
- MQTT_Port (Usually 1883)
- MQTT_Prefix (The base topic that all JMRI topics are based on, defined in JMRI MQTT config)

Device Settings
- Ramp_Rate (PWM Ramp delay per step in seconds)
- PWM_Freq (PWM Frequency in Hz)
- Debounce_delay (Switch debounce delay in seconds)
- Neopixel_pin (The pin used to shift out bits to any neopixels) (None if not used)

Server Configuration Settings
- The topics used by JMRI to publish certain events to 
- turnout_HIGH = "THROWN"
- turnout_LOW = "CLOSED"
- light_HIGH = "ON"
- light_LOW = "OFF"
- sensor_HIGH = "ACTIVE"
- sensor_LOW = "INACTIVE"

MQTT_Addresses
- Devices to be controlled by TrainNode
- The number of devices is not limited except by the program execution time and physical I/O limitations of the Pico-W
- Available Device Types
   - Servo [PWM Pin(int), Min PWM(int), Max PWM(int), Ramp Enable(bool)] (OUTPUT)
   - Neopixel [Neopixel Index(int), Color ON([int, int, int], Color OFF[int, int, int]] (OUTPUT)
   - LED [LED Pin(int)] (OUTPUT)
   - Pin [Pin(int)] (OUTPUT)
   - General_OUT [] (OUTPUT)
   - Button_Latching [Button Pin(int), Pin State High(bool)] (INPUT)
   - Pin_Scan [Pin(int), Pin State High(bool)] (INPUT)
   - General_IN [] (INPUT)
