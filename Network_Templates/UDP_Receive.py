#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

This is an example to setup a UDP Listener.

@author: ioannisgeorgilas
"""

# First we import our libraries
import socket   # This library will allow you to communicate over the network
import sys      # This library will give us some information about your system
import logging

logging.basicConfig(level=logging.INFO)

# First we need to set the IP and PORT we are going to listen to
# "0.0.0.0" means listen on all network interfaces of this machine (USB, WiFi, etc.)
UDP_IP = "0.0.0.0"
 
# This is the LOCAL port I am expecting data (on the sending machine this is the REMOTE port)
UDP_PORT = 50002

# Create the socket for the UDP communication
sock = socket.socket(socket.AF_INET,    # Family of addresses, in this case IP (Internet Protocol) family 
                     socket.SOCK_DGRAM) # What protocol to use, in this case UDP (datagram)

# Bind to the socket and wait for data on this port
sock.bind((UDP_IP, UDP_PORT))
logging.info("Listening on IP: %s Port: %s", UDP_IP, UDP_PORT)

# Wait indefinitely (you will need to use Ctrl+C to stop the program)
while True:
    #Read data
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    # Print data
    logging.info("received message: %s", data.decode('utf-8')) # As a string (check the ASCII table)
    logging.info("received bytes: %s", list(data)) # As byte values
    logging.info("from address: %s", addr) # Print the address of the sender