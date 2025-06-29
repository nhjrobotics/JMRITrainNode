import json
from time import sleep, ticks_us, sleep_ms
#import neopixel
from lib.umqtt.simple import MQTTClient
import network
from machine import Pin, PWM, SPI
#from lib.PiicoDev.PiicoDev_RFID import PiicoDev_RFID
#from lib.PiicoDev.PiicoDev_Unified import sleep_ms
import re
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


global config
config = config_manager()
config.load_config('config.json')

global neo
if config.settings["neopixel_count"] != 0:
    pass
        #neo = neopixel.NeoPixel(Pin(config.settings["neopixel_pin"]), config.settings["neopixel_count"])


def sub_cb(topic, msg):
    if topic.decode() == config.settings["client_name"] + "/client_name":
        print("Changing client name to", msg.decode())
        config.settings["client_name"] = msg.decode()
        config.save_config()
        return 
    
    elif config.settings["client_name"] in topic.decode():
        decoded_topic = re.sub(config.settings["client_name"] + "/", "", topic.decode())
        device_num = decoded_topic.split("/")[0]
        setting = decoded_topic.split("/")[1]
        print("device_num", device_num, "setting", setting, "to", msg.decode())

        try:
            json_object = json.loads(msg.decode())
            config.devices[device_num][setting] = json_object
        except:
            config.devices[device_num][setting] = msg.decode()
        config.save_config()
        return

    device = None
    for potential_device in config.devices:
        if config.devices[potential_device]["address"] == topic.decode():
            device = config.devices[potential_device]
            break
    
    if device == None:
        return
    
    if device["current_state"]["state"] != msg.decode():
        for state in device["states"]:
            if msg.decode() == state:
                device["current_state"]["state"] = state
                config.save_config()
                print(device["type"], device["address"], device["current_state"]["state"], device["states"][device["current_state"]["state"]])
                return
        print(f"{msg.decode()}: invalid state, no change:", device["type"], device["address"], device["current_state"]["state"], device["states"][device["current_state"]["state"]])
        return
    

def check_mqtt_msg(mqtt):
    mqtt.check_msg()


def publish_mqtt(mqtt, address, msg):
    mqtt.publish(address, msg, qos=1)


def sub_mqtt(mqtt, address):
    mqtt.subscribe(address)


def connect():
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

    if nic.isconnected():
        ##try:
            print("Connecting to MQTT Server")
            mqtt = MQTTClient(client_id = config.settings["client_name"], server = config.settings["server_addr"], port = config.settings["MQTT_port"], user = config.settings["MQTT_user"], password = config.settings["MQTT_password"])
            mqtt.set_callback(sub_cb)
            mqtt.connect()
            print("Connected to MQTT Server")

            return nic, mqtt
        ##except Exception as e:
        ##   print(f"Error: MQTT Connection Failed: {e}")
    return None, None


def servo(device):
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
    config.save_config()


def pin_output(device):
    pin = Pin(config.devices[device]["args"]["pin"], Pin.OUT)
    pin.value(config.devices[device]["states"][config.devices[device]["current_state"]["state"]])
    config.devices[device]["current_state"]["position"] = config.devices[device]["states"][config.devices[device]["current_state"]["state"]]
    config.save_config()


def pin_input(device):
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
            publish_mqtt(mqtt, config.devices[device]["address"], state_to_set)
            print(config.devices[device]["type"], config.devices[device]["address"], config.devices[device]["current_state"]["state"])
            config.save_config()


def neopixel_process(device):
    if config.settings["neopixel_count"] != 0:
        #neo[config.devices[device]["args"]["pixel"]] = config.devices[device]["states"][config.devices[device]["current_state"]["state"]]
        #neo.write()
        config.devices[device]["current_state"]["position"] = config.devices[device]["states"][config.devices[device]["current_state"]["state"]]
        config.save_config()


def rfid_process(device):
    pass
    '''
    rfid = PiicoDev_RFID(sda=Pin(config.devices[device]["args"]["sda"]), 
                         scl=Pin(config.devices[device]["args"]["scl"]), 
                         asw=config.devices[device]["args"]["asw_address"],
                         bus=config.devices[device]["args"]["bus"])
    if rfid.tagPresent():
        id = rfid.readID()
        if id != '':
            config.devices[device]["current_state"]["state"] = id
            config.save_config()
            print(config.devices[device]["type"], config.devices[device]["address"], config.devices[device]["current_state"]["state"])
            publish_mqtt(mqtt, config.devices[device]["address"], config.devices[device]["current_state"]["state"])           
            timeout_start = ticks_us()
            while(rfid.readID() != ''):
                if ticks_us() - timeout_start > (1000 * config.settings["device_poll_timeout_ms"]):
                    return
    '''


def button(device):
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
                        publish_mqtt(mqtt, config.devices[device]["address"], state_to_set)
                        print(config.devices[device]["type"], config.devices[device]["address"], config.devices[device]["current_state"]["state"])
                        config.save_config()
                        timeout_start = ticks_us()
                        while pin.value() == config.devices[device]["args"]["button_trig"]:
                            if ticks_us() - timeout_start > (1000 * config.settings["device_poll_timeout_ms"]):
                                return

            else:
                print(f"Error: too many states in device {device} for button object")


def process_inputs():
    for device in config.devices:
        if config.devices[device]["io"] == "INPUT":
            if config.devices[device]["type"] == "rfid":
                rfid_process(device)
            elif config.devices[device]["type"] == "button":
                button(device)
            elif config.devices[device]["type"] == "pin_input":
                pin_input(device)


def process_outputs():
    for device in config.devices:
        if config.devices[device]["io"] == "OUTPUT":
            position = config.devices[device]["current_state"]["position"]
            if position != config.devices[device]["states"][config.devices[device]["current_state"]["state"]]:
                print(config.devices[device]["address"], config.devices[device]["current_state"]["state"], "position", config.devices[device]["current_state"]["position"], "to", config.devices[device]["states"][config.devices[device]["current_state"]["state"]])

                if config.devices[device]["type"] == "servo":
                    servo(device)
                elif config.devices[device]["type"] == "pin_output":
                    pin_output(device)
                elif config.devices[device]["type"] == "neopixel":
                    neopixel_process(device)


if __name__ == "__main__":
    while True:
        net, mqtt = connect()
        if net == None:
            print("Connection Failed, retrying...")
            continue
        for device in config.devices:
            device_num = str(device)
            sub_mqtt(mqtt, config.devices[device]["address"])
            print("Subscribed to", config.devices[device]["address"])

        counter = 0
        while net.isconnected():
            try:
                start_time = ticks_us()
                #check_mqtt_msg(mqtt)
                user.custom_node_functions(config.devices)
                process_inputs()
                process_outputs()
                finish_time = ticks_us()
                if counter == 10000:
                    config.settings["cycle_time"] = finish_time - start_time
                    mqtt_address = str(config.settings["client_name"]) + "/" + "cycle_time"
                    print("publishing ", mqtt_address)
                    publish_mqtt(mqtt, mqtt_address, str(config.settings["cycle_time"]))
                    counter = 0
                counter += 1 
            except Exception as e:
                print(f"Error: Connection Lost: {e}")
                break