# Network Communication Templates

This folder contains Python templates for various network communication protocols and architectures.

## Files

### TCP (Transmission Control Protocol)
- **TCP_Client_RepeatSend.py** - TCP client that repeatedly sends messages to a server

### TCP/IP Loopback Testing
- **TCP_Server_LocalLoopback.py** - TCP server for local network testing

### UDP (User Datagram Protocol)
- **UDP_Send.py** - UDP client for sending messages
- **UDP_Receive.py** - UDP server for receiving messages

### MQTT (Message Queuing Telemetry Transport)
- **testmqtt_BATH_readonly.py** - MQTT client with read-only access (University of Bath MQTT broker)
- **testmqtt_BATH_readwrite.py** - MQTT client with read/write access (University of Bath MQTT broker)

## Protocols

### TCP
- Connection-oriented, reliable delivery
- Suitable for critical data transmission
- Higher overhead but guarantees message order and delivery

### UDP
- Connectionless, best-effort delivery
- Low latency, minimal overhead
- Suitable for real-time applications where occasional packet loss is acceptable

### MQTT
- Publish-subscribe messaging protocol
- Designed for IoT and lightweight messaging
- Includes Quality of Service (QoS) levels

## Usage

These templates demonstrate:
- Client-server architectures
- Bidirectional communication
- Connection management
- Message handling

## Logging

All templates use Python's built-in [`logging`](https://docs.python.org/3/library/logging.html) module at the INFO level for consistent status and message output across protocols.

## Requirements

- Python 3.x
- `paho-mqtt` library (for MQTT examples)
- Network connectivity to broker/server
