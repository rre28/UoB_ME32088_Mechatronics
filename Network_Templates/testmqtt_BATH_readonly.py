import paho.mqtt.client as mqtt
import time
import logging

logging.basicConfig(level=logging.INFO)

# The callback for when the client receives a CONNACK response from the server.
# def on_connect(client, userdata, rc):
#     print("Connected with result code "+str(rc))
#     # Subscribing in on_connect() means that if we lose the connection and
#     # reconnect then subscriptions will be renewed.
#     client.subscribe("$SYS/#")

# This is a different function call based on this: http://www.steves-internet-guide.com/mqtt-python-callbacks/
def on_connect(client,userdata,flags,rc,properties):
    logging.info("Connected with result code %s", rc)
    #client.subscribe("$SYS/#")
    client.subscribe("TeamTopic")
    logging.info("Subscribed to TeamTopic")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    logging.info("%s %s %s", time.time(), msg.topic, msg.payload)

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2) # VERSION2 is the callback format of paho-mqtt 2.x
client.on_connect = on_connect
client.on_message = on_message

client.username_pw_set(***USERNAME***,password=***PASSWORD***)
client.connect(***SERVER***,***PORT***,60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface
client.loop_forever()
