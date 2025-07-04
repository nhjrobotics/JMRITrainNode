from lib.umqtt.simple import MQTTClient
import network
from machine import Pin, SPI
import time


def connect(config):
    spi=SPI(0,2_000_000, mosi=Pin(19),miso=Pin(16),sck=Pin(18))
    nic = network.WIZNET5K(spi,Pin(17),Pin(20)) #spi,cs,reset pin
    nic.active(True)

    if not nic.isconnected():
        nic.connect()
        print("Waiting for connection...")
        while not nic.isconnected():
            time.sleep(1)

    ip = nic.ifconfig()[0]
    print(f'Connected on {ip}')

    if nic.isconnected():
        mqtt = MQTT_handler(config)

        return nic, mqtt
    return None, None


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
                    self.config.save_config()
                    print(device["type"], device["address"], device["current_state"]["state"], device["states"][device["current_state"]["state"]])
                    return
            print(f"{msg.decode()}: invalid state, no change:", device["type"], device["address"], device["current_state"]["state"], device["states"][device["current_state"]["state"]])
            return


    def pub(self, address, msg):
        self.mqtt.publish(address, msg, qos=1)


    def sub(self, address):
        self.mqtt.subscribe(address)

