import paho.mqtt.client as mqtt # This is the library to do the MQTT communications
import time # This is the library that will allow us to use the sleep function
import logging

logging.basicConfig(level=logging.INFO)

# The callback for when we connect to the server. The meaning of the return code (rc) can be found here:
# https://docs.oasis-open.org/mqtt/mqtt/v5.0/os/mqtt-v5.0-os.html#_Toc3901079
# Success (code 0) is good.
# After we connect we subscribe to one (or more) topics in this case the topic TeamTopic/1
def on_connect(client,userdata,flags,rc,properties):
    logging.info("Connected with result code %s", rc)
    client.subscribe(MainTopic+"1")


# The callback for when a PUBLISH message is received from the server. I.e. when a new value for the topic we subscribed to above updates
# the expression str(int(msg.payload.rstrip(b'\x00'))) converts the message from the server to an integer and then a string for printing.
# Specifically:
#   msg.payload.rstrip(b'\x00') This is getting all the bytes from the server (should be 11 in our case) and removes bytes with the value \x00 from the right
#   int() converts the remaining bytes into an integer, it treats them as a string
#   str() converts the integer into a string for the purpose of printing it. See below (l.56) for an alternative way to do this
def on_message(client, userdata, msg):
    logging.info(
        "%s In topic: %s the value was %s",
        time.time(),
        msg.topic,
        int(msg.payload.rstrip(b'\x00')),
    )

# Create the mqtt client object
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2) # VERSION2 is the callback format of paho-mqtt 2.x
# Assign the function for the connection event
client.on_connect = on_connect
# Assign the function for the new message event
client.on_message = on_message

# Set the username and password
client.username_pw_set(***USERNAME***,password=***PASSWORD***)

# Connect to the server using a specific port with a timeout delay (in seconds)
client.connect(***SERVER***,***PORT***,60)

# Create your main topic string. The subtopics are numbers, e.g. TeamTopic/1, TeamTopic/5
MainTopic = "TeamTopic/"

# Start the client to enable the above events to happen
client.loop_start()

# Pick a value
val =254

# Send (Publish) the value continuously
while(1):

    # Start a try in case we have an error
    try:
        # Every one second
        time.sleep(1)
        
        # Publish the value (integer) as a string. All messages are strings
        client.publish(MainTopic+"5",str(val))

        # Plot in the terminal what we just did
        logging.info("%s5 %d", MainTopic, val)
        val=val+1
    
    # Capture a KeyboardInterrupt, i.e. Ctrl-C in the terminal window
    except (KeyboardInterrupt):
        # Stop the Client
        client.loop_stop()
        # Disconnect
        client.disconnect()
        # Leave the while which will stop the code
        break

    # Capture any other error from the three lines (most likely will be a publish error)
    except:
        logging.error("There was an error while publishing the data.")
