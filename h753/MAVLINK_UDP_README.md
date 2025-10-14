# MAVLink over UDP Implementation for STM32H753

## Overview

This firmware enables MAVLink communication over UDP/Ethernet on the STM32H753XX microcontroller. It replaces the simple UDP echo example with a full MAVLink protocol handler.

## Features

- **MAVLink v2 Protocol**: Full MAVLink communication support
- **UDP Transport**: MAVLink messages transported over UDP/Ethernet
- **Standard Port**: Uses port 14550 (standard MAVLink port)
- **Heartbeat**: Automatically sends heartbeat messages at 1 Hz
- **Message Handler**: Extensible message handling for various MAVLink message types
- **LWIP Integration**: Fully integrated with LWIP TCP/IP stack
- **FreeRTOS**: Runs as a FreeRTOS task

## Hardware Configuration

- **MCU**: STM32H753XX
- **Ethernet**: Configured with LWIP stack
- **IP Address**: 192.168.11.4 (configurable in LWIP/App/lwip.c)
- **MAVLink UDP Port**: 14550 (local and remote)
- **Remote PC IP**: 192.168.11.2 (configurable in freertos.c)

## Network Configuration

### Default Settings

```c
// Board IP Configuration (in LWIP/App/lwip.c)
IP_ADDRESS:      192.168.11.4
NETMASK:         255.255.255.0
GATEWAY:         192.168.11.1

// MAVLink Configuration (in Core/Src/freertos.c)
System ID:       1
Component ID:    MAV_COMP_ID_ONBOARD_COMPUTER
Local Port:      14550
Remote Port:     14550
Remote IP:       192.168.11.2
```

### Changing Network Settings

1. **Board IP Address**: Edit `LWIP/App/lwip.c`, lines 61-72
2. **Remote PC IP**: Edit `Core/Src/freertos.c`, line 196
3. **MAVLink Ports**: Edit `Core/Src/freertos.c`, lines 194-195

## Build Instructions

```bash
cd h753
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .
```

The output firmware will be in `build/H753UDP.elf`

## Flashing

```bash
STM32_Programmer_CLI --connect port=swd --download build/H753UDP.elf --hardRst --rst --start
```

## Testing with QGroundControl

1. **Configure Network**:
   - Set your PC IP to 192.168.11.2 (or update firmware to match your IP)
   - Connect PC and STM32H753 board via Ethernet

2. **Launch QGroundControl**:
   - QGroundControl should automatically detect the MAVLink device on UDP port 14550
   - You should see heartbeat messages from the STM32

3. **Verify Connection**:
   - LED on PB0 will toggle at 1 Hz when sending heartbeats
   - QGroundControl should show "Connected" status

## Testing with Python (mavutil)

```python
from pymavlink import mavutil

# Connect to STM32H753
master = mavutil.mavlink_connection('udp:192.168.11.4:14550')

# Wait for heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received from system %u component %u" %
      (master.target_system, master.target_component))

# Request parameters (example)
master.mav.param_request_list_send(
    master.target_system,
    master.target_component
)

# Receive messages
while True:
    msg = master.recv_match(blocking=True, timeout=1.0)
    if msg:
        print(msg)
```

## Architecture

### File Structure

```
h753/
├── App/
│   └── comm/
│       ├── mavlink_udp.h        # MAVLink UDP handler interface
│       └── mavlink_udp.c        # MAVLink UDP handler implementation
├── Core/
│   └── Src/
│       └── freertos.c           # FreeRTOS task with MAVLink integration
├── Lib/
│   └── mavlink/
│       └── c_library_v2/        # MAVLink C library
│           └── common/          # Common MAVLink message definitions
└── CMakeLists.txt               # Build configuration
```

### Key Components

1. **mavlink_udp.h/c**: Core MAVLink UDP handler
   - Initializes UDP socket for MAVLink
   - Parses incoming MAVLink messages
   - Sends MAVLink messages over UDP
   - Handles D-Cache operations for H7 series

2. **freertos.c**: Main application task
   - Initializes LWIP and MAVLink
   - Sends periodic heartbeat messages
   - Dispatches received messages to handlers

3. **MAVLink Library**: Standard MAVLink C library v2
   - Common message definitions
   - Message parsing and serialization

## Supported MAVLink Messages

### Outgoing (STM32 → PC)

- **HEARTBEAT**: Sent at 1 Hz
  - Type: MAV_TYPE_ONBOARD_CONTROLLER
  - Autopilot: MAV_AUTOPILOT_INVALID
  - State: MAV_STATE_ACTIVE

### Incoming (PC → STM32)

Currently implemented message handlers in `freertos.c`:

- **HEARTBEAT**: Received and decoded
- **PARAM_REQUEST_LIST**: Parameter request (stub)
- **PARAM_REQUEST_READ**: Parameter read request (stub)
- **PARAM_SET**: Parameter set command (stub)

## Adding Custom Message Handlers

To handle additional MAVLink messages, modify the `mavlink_message_handler()` function in `Core/Src/freertos.c`:

```c
void mavlink_message_handler(const mavlink_message_t *msg, mavlink_udp_t *handler)
{
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_YOUR_MESSAGE:
    {
      // Decode message
      mavlink_your_message_t your_msg;
      mavlink_msg_your_message_decode(msg, &your_msg);

      // Process message
      // ...

      // Send response if needed
      mavlink_message_t response;
      mavlink_msg_your_response_pack(handler->config.system_id,
                                      handler->config.component_id,
                                      &response,
                                      /* parameters */);
      mavlink_udp_send_message(handler, &response);
      break;
    }

    default:
      // Unknown message
      break;
  }
}
```

## Sending Custom Messages

Example of sending a custom MAVLink message:

```c
// Create message
mavlink_message_t msg;
mavlink_msg_statustext_pack(mavlink_handler.config.system_id,
                             mavlink_handler.config.component_id,
                             &msg,
                             MAV_SEVERITY_INFO,
                             "Hello from STM32!",
                             0, 0);

// Send message
mavlink_udp_send_message(&mavlink_handler, &msg);
```

## Performance Characteristics

- **Heartbeat Rate**: 1 Hz
- **Message Processing**: Real-time in LWIP callback
- **Memory Usage**:
  - DTCMRAM: ~34 KB (25.82%)
  - RAM_D2: ~281 KB (95.44%)
  - Flash: ~113 KB (86.20%)

## Troubleshooting

### No MAVLink Connection

1. **Check Network**:
   - Verify Ethernet cable is connected
   - Ping 192.168.11.4 from your PC
   - Check PC firewall settings for UDP port 14550

2. **Check LED**:
   - LED on PB0 should blink at 1 Hz when sending heartbeats
   - If LED blinks rapidly (10 Hz), MAVLink init failed

3. **Check IP Configuration**:
   - Verify board IP (192.168.11.4) matches your network
   - Verify remote IP (192.168.11.2) matches your PC

### Build Errors

1. **MAVLink include errors**: Make sure `Lib/mavlink/` directory exists
2. **Missing symbols**: Rebuild from clean state (`rm -rf build`)

## Differences from f446re Implementation

| Feature | f446re (UART) | h753 (UDP) |
|---------|---------------|------------|
| Transport | UART2 @ 115200 | UDP/Ethernet |
| Protocol | MAVLink v2 | MAVLink v2 |
| Language | C++20 | C |
| Architecture | Complex (SystemContext) | Simplified |
| Port | UART pins | Ethernet |
| RTOS | Optional | FreeRTOS required |

## References

- [MAVLink Protocol](https://mavlink.io/)
- [QGroundControl](http://qgroundcontrol.com/)
- [pymavlink Documentation](https://mavlink.io/en/mavgen_python/)
- [LWIP Documentation](https://www.nongnu.org/lwip/)
