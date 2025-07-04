from lib.neopixel.ws2812lib import ws2812_array
from machine import Pin, PWM


class neopixel:
    def __init__(self, config):
        self.neo = ws2812_array(config.settings["neopixel_count"], config.settings["neopixel_pin"])

    def process(self, device, config):
        self.neo.pixels_set(config.devices[device]["args"]["pixel"], config.devices[device]["states"][config.devices[device]["current_state"]["state"]])
        self.neo.pixels_show()
        config.devices[device]["current_state"]["position"] = config.devices[device]["states"][config.devices[device]["current_state"]["state"]]
        config.save_config()


def pin_output(device, config):
    pin = Pin(config.devices[device]["args"]["pin"], Pin.OUT)
    pin.value(config.devices[device]["states"][config.devices[device]["current_state"]["state"]])
    config.devices[device]["current_state"]["position"] = config.devices[device]["states"][config.devices[device]["current_state"]["state"]]
    config.save_config()


def servo(device, config):
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


def process_outputs(config, neo: neopixel):
    for device in config.devices:
        if config.devices[device]["io"] == "OUTPUT":
            position = config.devices[device]["current_state"]["position"]
            if position != config.devices[device]["states"][config.devices[device]["current_state"]["state"]]:
                print(config.devices[device]["address"], config.devices[device]["current_state"]["state"], "position", config.devices[device]["current_state"]["position"], "to", config.devices[device]["states"][config.devices[device]["current_state"]["state"]])

                if config.devices[device]["type"] == "servo":
                    servo(device, config)
                elif config.devices[device]["type"] == "pin_output":
                    pin_output(device, config)
                elif config.devices[device]["type"] == "neopixel":
                    if neo != None:
                        neo.process(device, config)
