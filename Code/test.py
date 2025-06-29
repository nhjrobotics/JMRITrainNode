from lib.umqtt.simple import MQTTClient
from usocket import socket
from machine import Pin,SPI
import network
import time

#mqtt config
mqtt_server = '192.168.168.240'
client_id = 'wiz'
topic_sub = b'hello'

last_message = 0
message_interval = 5
counter = 0

#W5x00 chip init
def w5x00_init():
    spi=SPI(0,2_000_000, mosi=Pin(19),miso=Pin(16),sck=Pin(18))
    nic = network.WIZNET5K(spi,Pin(17),Pin(20)) #spi,cs,reset pin
    nic.active(True)
    
    #DHCP
    #nic.ifconfig('dhcp')
    print('IP address :', nic.ifconfig())
    
    while not nic.isconnected():
        time.sleep(1)
        print(nic.regs())
    
def sub_cb(topic, msg):
    print((topic.decode('utf-8'), msg.decode('utf-8')))

#MQTT connect
def mqtt_connect():
    client = MQTTClient(client_id, mqtt_server, 1883, "mqtt-user", "9te%BM3u$ps77dMoR$B7@DJyBZ", keepalive=60)
    client.set_callback(sub_cb)
    client.connect()
    print('Connected to %s MQTT Broker'%(mqtt_server))
    return client

#reconnect & reset
def reconnect():
    print('Failed to connected to Broker. Reconnecting...')
    time.sleep(5)
    
def main():
    w5x00_init()
    
    try: 
        client = mqtt_connect()
    except OSError as e:
        reconnect()

    while True:
        client.subscribe(topic_sub, 0)
        publish = "publish test"
        client.publish("arstarst", publish)
        print(f"published: {publish}")
        time.sleep(1)

    client.disconnect()

if __name__ == "__main__":
    main()


