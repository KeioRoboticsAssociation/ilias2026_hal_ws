/**
 * @file mavlink_udp.h
 * @brief MAVLink over UDP communication handler for STM32H753
 */

#ifndef MAVLINK_UDP_H
#define MAVLINK_UDP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "robomaster_motor/mavlink.h"  // Use robomaster_motor dialect (includes common + custom messages)
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief MAVLink UDP configuration
 */
typedef struct {
    uint8_t system_id;         // MAVLink system ID
    uint8_t component_id;      // MAVLink component ID
    uint16_t local_port;       // Local UDP port to bind
    uint16_t remote_port;      // Remote UDP port to send to
    ip_addr_t remote_addr;     // Remote IP address
} mavlink_udp_config_t;

/**
 * @brief MAVLink UDP handler context
 */
typedef struct {
    struct udp_pcb *pcb;              // UDP protocol control block
    mavlink_udp_config_t config;      // Configuration
    mavlink_status_t rx_status;       // MAVLink RX parser status
    mavlink_message_t rx_message;     // Received MAVLink message
    uint32_t messages_received;       // Statistics: messages received
    uint32_t messages_sent;           // Statistics: messages sent
    uint32_t parse_errors;            // Statistics: parse errors
    bool initialized;                 // Initialization flag
} mavlink_udp_t;

/**
 * @brief Message handler callback type
 *
 * @param msg Pointer to received MAVLink message
 * @param handler Pointer to the mavlink_udp handler
 */
typedef void (*mavlink_message_handler_t)(const mavlink_message_t *msg, mavlink_udp_t *handler);

/**
 * @brief Initialize MAVLink UDP communication
 *
 * @param handler Pointer to mavlink_udp_t handler structure
 * @param config Pointer to configuration structure
 * @param msg_handler Callback function for received messages
 * @return err_t LWIP error code (ERR_OK on success)
 */
err_t mavlink_udp_init(mavlink_udp_t *handler,
                       const mavlink_udp_config_t *config,
                       mavlink_message_handler_t msg_handler);

/**
 * @brief Deinitialize MAVLink UDP communication
 *
 * @param handler Pointer to mavlink_udp_t handler structure
 */
void mavlink_udp_deinit(mavlink_udp_t *handler);

/**
 * @brief Send a MAVLink message via UDP
 *
 * @param handler Pointer to mavlink_udp_t handler structure
 * @param msg Pointer to MAVLink message to send
 * @return err_t LWIP error code (ERR_OK on success)
 */
err_t mavlink_udp_send_message(mavlink_udp_t *handler, const mavlink_message_t *msg);

/**
 * @brief Send a MAVLink message to a specific address
 *
 * @param handler Pointer to mavlink_udp_t handler structure
 * @param msg Pointer to MAVLink message to send
 * @param addr Destination IP address
 * @param port Destination port
 * @return err_t LWIP error code (ERR_OK on success)
 */
err_t mavlink_udp_send_message_to(mavlink_udp_t *handler,
                                   const mavlink_message_t *msg,
                                   const ip_addr_t *addr,
                                   uint16_t port);

/**
 * @brief Send a MAVLink HEARTBEAT message
 *
 * @param handler Pointer to mavlink_udp_t handler structure
 * @param type MAV_TYPE enum value
 * @param autopilot MAV_AUTOPILOT enum value
 * @param base_mode MAV_MODE_FLAG bitmask
 * @param system_status MAV_STATE enum value
 * @return err_t LWIP error code (ERR_OK on success)
 */
err_t mavlink_udp_send_heartbeat(mavlink_udp_t *handler,
                                  uint8_t type,
                                  uint8_t autopilot,
                                  uint8_t base_mode,
                                  uint8_t system_status);

/**
 * @brief Get statistics
 *
 * @param handler Pointer to mavlink_udp_t handler structure
 * @param rx_count Output: number of messages received
 * @param tx_count Output: number of messages sent
 * @param errors Output: number of parse errors
 */
void mavlink_udp_get_stats(const mavlink_udp_t *handler,
                            uint32_t *rx_count,
                            uint32_t *tx_count,
                            uint32_t *errors);

#ifdef __cplusplus
}
#endif

#endif // MAVLINK_UDP_H
