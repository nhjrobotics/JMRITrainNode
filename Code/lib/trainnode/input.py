from lib.PiicoDev.PiicoDev_RFID import PiicoDev_RFID
from machine import Pin
from time import ticks_us, sleep_ms


def process_inputs(mqtt, config):
    for device in config.devices:
        if config.devices[device]["io"] == "INPUT":
            if config.devices[device]["type"] == "rfid":
                rfid_process(device, config, mqtt)
            elif config.devices[device]["type"] == "button":
                button(device, config, mqtt)
            elif config.devices[device]["type"] == "pin_input":
                pin_input(device, config, mqtt)


def rfid_process(device, config, mqtt):
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
            config.save_config()
            print(config.devices[device]["type"], config.devices[device]["address"], config.devices[device]["current_state"]["state"])
            mqtt.pub(config.devices[device]["address"], config.devices[device]["current_state"]["state"])           
            timeout_start = ticks_us()
            while(rfid.readID() != ''):
                if ticks_us() - timeout_start > (1000 * config.settings["device_poll_timeout_ms"]):
                    return


def button(device, config, mqtt):
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
                        config.save_config()
                        timeout_start = ticks_us()
                        while pin.value() == config.devices[device]["args"]["button_trig"]:
                            if ticks_us() - timeout_start > (1000 * config.settings["device_poll_timeout_ms"]):
                                return

            else:
                print(f"Error: too many states in device {device} for button object")


def pin_input(device, config, mqtt):
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
            mqtt.pub(mqtt, config.devices[device]["address"], state_to_set)
            print(config.devices[device]["type"], config.devices[device]["address"], config.devices[device]["current_state"]["state"])
            config.save_config()
