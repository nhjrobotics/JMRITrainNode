import user_functions
from lib.trainnode.utils import config_manager
import lib.trainnode.input
import lib.trainnode.output
import lib.trainnode.mqtt


if __name__ == "__main__":
    config = config_manager()
    config.load_config('config.json')
    if config.settings["neopixel_count"] != 0:
        neo = lib.trainnode.output.neopixel(config)
    attempt = 0
    while True:
        print("TrainNode by NHJRobotics... Initialising")
        print(f"Client name: {config.settings["client_name"]}")
        try:
            net, mqtt = lib.trainnode.mqtt.connect(config)
            for device in config.devices:
                if mqtt != None:
                    mqtt.sub(config.devices[device]["address"])
                print("Subscribed to", config.devices[device]["address"])
        except Exception as e:
                print(f"Attempt {attempt + 1}/{config.settings["max_ip_connect_attempts"]} - Error: Connection Lost: {e}")
                attempt += 1
                if attempt == config.settings["max_ip_connect_attempts"]:
                    break
                continue

        while net != None and net.isconnected() and mqtt != None:
            #try: 
                mqtt.pub(config.settings["client_name"], "HEARTBEAT")
                print(config.settings["client_name"], "HEARTBEAT")
                user_functions.custom_node_functions(config.devices)
                lib.trainnode.input.process_inputs(mqtt, config)
                lib.trainnode.output.process_outputs(config, neo)
            #except Exception as e:
                #print(f"Error: Input / Output Processing Failed: {e}")
                #break
