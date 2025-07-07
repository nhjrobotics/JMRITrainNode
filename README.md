![image](https://github.com/user-attachments/assets/75c13b00-b55d-4264-bcbb-a3ae0a9e97ee)


# JMRITrainNode
TrainNode is a JMRI compatible I/O controller built on the MQTT protocol. Using the Raspberry Pi Pico, each node can support up to 16 turnouts as standard, or more with user addons.

RAM reduced refers to the ability for TrainNode to run on the RP2040 with lower RAM specs.

## Required Hardware
This project uses the [Wiznet Ethernet Hat](https://docs.wiznet.io/Product/Open-Source-Hardware/wiznet_ethernet_hat) for the Raspberry Pi Pico (RP2040), which is available at major retailers such as Core Electronics in Australia. This is used for networking. The Pico-W is unsupported at this time.

## Installation
- To use TrainNode, you must have a MQTT server with username and password authentication connected to JMRI ([JMRI + MQTT](https://www.jmri.org/help/en/html/hardware/mqtt/index.shtml)). It is recommended to use Eclipse Mosquitto as your MQTT broker.
- To install TrainNode on the Rpi Pico, install MicroPython for Pico compiled from the Wiznet Github ([MicroPython](https://github.com/Wiznet/RP2040-HAT-MicroPython/releases/download/v1.0.5/rp2_w5100s_20220421_v1.0.5.uf2)).
- Next, configure the TrainNode [config_example.json](https://github.com/rewind2b4/JMRITrainNode/tree/master/Code/config_example.json) file and rename it config.json (see Docs for more info). 
- Finally, copy the files from the [code](https://github.com/rewind2b4/JMRITrainNode/tree/master/Code) folder to the Pico.
- The Pico will automatically connect to the network, connecting to the layout MQTT broker. It will now be operational.

## Docs
General Settings
"server_addr": IP address of the MQTT server
"client_name": The client name of the node
"MQTT_user": The username of the MQTT server
"MQTT_password": The password of the MQTT server
"MQTT_port": The port the MQTT server is on
"neopixel_pin": The GPIO pin that any neopixels are connected to (ignored when 0 neopixels)
"neopixel_count": The number of neopixels attached to the node (put 0 for no neopixels)
"device_poll_timeout_ms": The amount of time that the system waits for interupts such as button releases when pressed
"max_ip_connect_attempts": The amount of times the node will attempt to connect before powering off (requiring a power cycle)

Device Settings (see example_json for usage)
- Devices to be controlled by TrainNode
- The number of devices is not limited except by the program execution time and physical limitations of the Pico (I/O, RAM, etc).
- Additional data can be added, this will be ignored by the system unless used in the user program space.

The device settings looks as follows:

"address": "jmri/track/turnout/4",
"current_state": {
      "state": "CLOSED",
      "position": 0
},
"states": {
      "THROWN": [255, 0, 0],
      "CLOSED": [0, 255, 0]
},
"io": "OUTPUT",
"type": "neopixel",
"args": {
}

Where,
"address": "jmri/track/turnout/4",
"current_state": wrapper for "state" and "position" (Do not modify)
"state": The current state of the device (this is updated in real time), eg. "THROWN"
"position": The current position of the device (this is updated in real time), eg. [255, 0, 0]
"states": The messages that JMRI will send and the response position, eg. "THROWN": [255, 0, 0], "CLOSED": [0, 255, 0]
"io": Whether the device is an INPUT or an OUTPUT
"type": The device type (see list below)
"args": Any other arguments required by the device (see arguments in list below)

Available Device Types
   - servo ["freq": Frequency of the servo, "pin": Output GPIO pin, "ramp": true or false for whether the servo will ramp to its final position, "step": The size of the step for the ramp per cycle] (OUTPUT)
   - neopixel ["pixel": the output pixel] (OUTPUT)
   - pin_output ["pin": GPIO pin] (OUTPUT)
   - rfid ["bus": The bus for the SPI interface, "sda": The SDA pin, "scl": The SCL pin, "asw_address": the switch address on the piicodev RFID] (INPUT)
   - button ["pin": GPIO pin, "pullup": true or false for pullup, "debounce": The debounce time in milliseconds, "button_trig": Whether the button is normally closed or normally open] (INPUT)
   - pin_input ["pin": GPIO pin, "pullup": true or false for pullup, "debounce": The debounce time in milliseconds] (INPUT)
