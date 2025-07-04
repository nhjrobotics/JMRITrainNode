import json
from time import sleep, ticks_us, sleep_ms
from lib.umqtt.simple import MQTTClient
import network
from machine import Pin, PWM, SPI
from lib.neopixel.ws2812lib import ws2812_array
from lib.PiicoDev.PiicoDev_RFID import PiicoDev_RFID
import user


class config_manager:
    def __init__(self):
        return
    
    def load_config(self, filename):
        self.filename = filename
        with open(self.filename, 'r') as json_file:
            self.config = json.load(json_file)

        self.devices = self.config["devices"]
        self.settings = self.config["settings"]
        return
    
    def save_config(self):
        with open(self.filename, 'w') as json_file:
            json_file.write(json.dumps(self.config))
        return 



def connect(config):
    spi=SPI(0,2_000_000, mosi=Pin(19),miso=Pin(16),sck=Pin(18))
    nic = network.WIZNET5K(spi,Pin(17),Pin(20)) #spi,cs,reset pin
    nic.active(True)

    if not nic.isconnected():
        nic.connect()
        print("Waiting for connection...")
        while not nic.isconnected():
            sleep(1)

    ip = nic.ifconfig()[0]
    print(f'Connected on {ip}')

    mqtt = MQTT_handler(config)

    return nic, mqtt


class MQTT_handler:
    def __init__(self, config):
        self.config = config
        
        print("Connecting to MQTT Server")
        self.mqtt = MQTTClient(client_id = config.settings["client_name"], server = config.settings["server_addr"], port = config.settings["MQTT_port"], user = config.settings["MQTT_user"], password = config.settings["MQTT_password"])
        self.mqtt.set_callback(self.sub_cb)
        self.mqtt.connect()
        print("Connected to MQTT Server")


    def sub_cb(self, topic, msg):
        device = None
        for potential_device in self.config.devices:
            if self.config.devices[potential_device]["address"] == topic.decode():
                device = self.config.devices[potential_device]
                break
    
        if device == None:
            return
    
        if device["current_state"]["state"] != msg.decode():
            for state in device["states"]:
                if msg.decode() == state:
                    device["current_state"]["state"] = state
                    print(device["type"], device["address"], device["current_state"]["state"], device["states"][device["current_state"]["state"]])
                    return
            print(f"{msg.decode()}: invalid state, no change:", device["type"], device["address"], device["current_state"]["state"], device["states"][device["current_state"]["state"]])
            return


    def pub(self, address, msg):
        self.mqtt.publish(address, msg, qos=1)


    def sub(self, address):
        self.mqtt.subscribe(address)


def servo(config: config_manager, device):
    servo = PWM(Pin(config.devices[device]["args"]["pin"]))
    servo.freq(config.devices[device]["args"]["freq"])
    if config.devices[device]["args"]["ramp"]:
        step = config.devices[device]["args"]["step"]

        if config.devices[device]["current_state"]["position"] > config.devices[device]["states"][config.devices[device]["current_state"]["state"]]:
            config.devices[device]["current_state"]["position"] -= step
        else:
            config.devices[device]["current_state"]["position"] += step
    else:
        config.devices[device]["current_state"]["position"] = config.devices[device]["states"][config.devices[device]["current_state"]["state"]]
    servo.duty_u16(config.devices[device]["current_state"]["position"])


def pin_output(config: config_manager, device):
    pin = Pin(config.devices[device]["args"]["pin"], Pin.OUT)
    pin.value(config.devices[device]["states"][config.devices[device]["current_state"]["state"]])
    config.devices[device]["current_state"]["position"] = config.devices[device]["states"][config.devices[device]["current_state"]["state"]]


def pin_input(mqtt: MQTT_handler, config: config_manager, device):
    if config.devices[device]["args"]["pullup"]:
        pin = Pin(config.devices[device]["args"]["pin"], Pin.IN, Pin.PULL_UP)
    else:
        pin = Pin(config.devices[device]["args"]["pin"], Pin.IN)
    state = pin.value()
    for config_state in config.devices[device]["states"]:
        if config.devices[device]["states"][config_state] == state:
            state_to_set = str(config_state)
            
            break
    if state_to_set != config.devices[device]["current_state"]["state"]:
        sleep_ms(config.devices[device]["args"]["debounce"])
        pin_check = pin.value()
        if pin_check == state:
            config.devices[device]["current_state"]["state"] = state_to_set
            mqtt.pub(config.devices[device]["address"], state_to_set)
            print(config.devices[device]["type"], config.devices[device]["address"], config.devices[device]["current_state"]["state"])


class neopixel:
    def __init__(self, config):
        self.config = config
        self.neo = None
        if config.settings["neopixel_count"] != 0:
            self.neo = ws2812_array(config.settings["neopixel_count"], config.settings["neopixel_pin"])

    def process(self, device):
        if self.neo != None:
            self.neo.pixels_set(config.devices[device]["args"]["pixel"], config.devices[device]["states"][config.devices[device]["current_state"]["state"]])
            self.neo.pixels_show()
            config.devices[device]["current_state"]["position"] = config.devices[device]["states"][config.devices[device]["current_state"]["state"]]


def rfid_process(mqtt: MQTT_handler, config: config_manager, device):
    try:
        rfid = PiicoDev_RFID(sda=Pin(config.devices[device]["args"]["sda"]), 
                         scl=Pin(config.devices[device]["args"]["scl"]), 
                         asw=config.devices[device]["args"]["asw_address"],
                         bus=config.devices[device]["args"]["bus"])
    except Exception as e:
        print(f"Error: RFID Processing Failed: {e}")
        return

    if rfid.tagPresent():
        id = rfid.readID()
        if id != '':
            config.devices[device]["current_state"]["state"] = id
            print(config.devices[device]["type"], config.devices[device]["address"], config.devices[device]["current_state"]["state"])
            mqtt.pub(config.devices[device]["address"], config.devices[device]["current_state"]["state"])           
            timeout_start = ticks_us()
            while(rfid.readID() != ''):
                if ticks_us() - timeout_start > (1000 * config.settings["device_poll_timeout_ms"]):
                    return



def button(mqtt: MQTT_handler, config: config_manager, device):
    if config.devices[device]["args"]["pullup"]:
        pin = Pin(config.devices[device]["args"]["pin"], Pin.IN, Pin.PULL_UP)
    else:
        pin = Pin(config.devices[device]["args"]["pin"], Pin.IN)
    state = pin.value()
    if config.devices[device]["args"]["button_trig"] == state and state != config.devices[device]["current_state"]["position"]:
        sleep_ms(config.devices[device]["args"]["debounce"])
        pin_check = pin.value()
        if pin_check == config.devices[device]["args"]["button_trig"]:
            current_state = config.devices[device]["current_state"]["state"]
            if len(config.devices[device]["states"]) == 2:
                for button_state in config.devices[device]["states"]:
                    if current_state != button_state:
                        state_to_set = button_state

                        config.devices[device]["current_state"]["state"] = state_to_set
                        mqtt.pub(config.devices[device]["address"], state_to_set)
                        print(config.devices[device]["type"], config.devices[device]["address"], config.devices[device]["current_state"]["state"])
                        timeout_start = ticks_us()
                        while pin.value() == config.devices[device]["args"]["button_trig"]:
                            if ticks_us() - timeout_start > (1000 * config.settings["device_poll_timeout_ms"]):
                                return

            else:
                print(f"Error: too many states in device {device} for button object")


def process_inputs(mqtt: MQTT_handler, config: config_manager):
    for device in config.devices:
        if config.devices[device]["io"] == "INPUT":
            if config.devices[device]["type"] == "rfid":
                rfid_process(mqtt, config, device)
            elif config.devices[device]["type"] == "button":
                button(mqtt, config, device)
            elif config.devices[device]["type"] == "pin_input":
                pin_input(mqtt, config, device)


def process_outputs(config: config_manager, neo:neopixel):
    for device in config.devices:
        if config.devices[device]["io"] == "OUTPUT":
            position = config.devices[device]["current_state"]["position"]
            if position != config.devices[device]["states"][config.devices[device]["current_state"]["state"]]:
                print(config.devices[device]["address"], config.devices[device]["current_state"]["state"], "position", config.devices[device]["current_state"]["position"], "to", config.devices[device]["states"][config.devices[device]["current_state"]["state"]])

                if config.devices[device]["type"] == "servo":
                    servo(config, device)
                elif config.devices[device]["type"] == "pin_output":
                    pin_output(config, device)
                elif config.devices[device]["type"] == "neopixel":
                    neo.process(device)



if __name__ == "__main__":
    config = config_manager()
    config.load_config('config.json')
    neo = neopixel(config)
    attempt = 0

    while True:
        print("TrainNode by NHJRobotics... Initialising")
        print(f"Client name: {config.settings["client_name"]}")
        try:
            net, mqtt = connect(config)
            for device in config.devices:
                mqtt.sub(config.devices[device]["address"])

                print("Subscribed to", config.devices[device]["address"])
        except Exception as e:
                print(f"Attempt {attempt + 1}/{config.settings["max_ip_connect_attempts"]} - Error: Connection Lost: {e}")
                attempt += 1
                if attempt == config.settings["max_ip_connect_attempts"]:
                    break
                continue

                
        while net.isconnected():
            try: 
                mqtt.pub(config.settings["client_name"], "HEARTBEAT")
                print(config.settings["client_name"], "HEARTBEAT")
                user.custom_node_functions(config.devices)
                process_inputs(mqtt, config)
                process_outputs(config, neo)
                config.save_config()
            except Exception as e:
                print(f"Error: Input / Output Processing Failed: {e}")
                break
                