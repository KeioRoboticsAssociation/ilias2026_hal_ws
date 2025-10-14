/**
 * @file mavlink_udp.c
 * @brief MAVLink over UDP communication handler implementation
 */

#include "mavlink_udp.h"
#include "main.h"
#include <string.h>

// Forward declaration of internal callback
static void mavlink_udp_recv_callback(void *arg, struct udp_pcb *pcb,
                                      struct pbuf *p, const ip_addr_t *addr,
                                      u16_t port);

// Global message handler callback
static mavlink_message_handler_t g_message_handler = NULL;

/**
 * @brief Initialize MAVLink UDP communication
 */
err_t mavlink_udp_init(mavlink_udp_t *handler,
                       const mavlink_udp_config_t *config,
                       mavlink_message_handler_t msg_handler) {
    if (!handler || !config) {
        return ERR_ARG;
    }

    // Initialize handler structure
    memset(handler, 0, sizeof(mavlink_udp_t));
    memcpy(&handler->config, config, sizeof(mavlink_udp_config_t));

    // Store message handler
    g_message_handler = msg_handler;

    // Create new UDP PCB
    handler->pcb = udp_new();
    if (handler->pcb == NULL) {
        return ERR_MEM;
    }

    // Bind to local port
    err_t err = udp_bind(handler->pcb, IP_ADDR_ANY, config->local_port);
    if (err != ERR_OK) {
        udp_remove(handler->pcb);
        handler->pcb = NULL;
        return err;
    }

    // Set receive callback
    udp_recv(handler->pcb, mavlink_udp_recv_callback, handler);

    handler->initialized = true;
    return ERR_OK;
}

/**
 * @brief Deinitialize MAVLink UDP communication
 */
void mavlink_udp_deinit(mavlink_udp_t *handler) {
    if (!handler || !handler->initialized) {
        return;
    }

    if (handler->pcb) {
        udp_remove(handler->pcb);
        handler->pcb = NULL;
    }

    handler->initialized = false;
}

/**
 * @brief UDP receive callback - parses MAVLink messages from UDP packets
 */
static void mavlink_udp_recv_callback(void *arg, struct udp_pcb *pcb,
                                      struct pbuf *p, const ip_addr_t *addr,
                                      u16_t port) {
    LWIP_UNUSED_ARG(pcb);

    mavlink_udp_t *handler = (mavlink_udp_t *)arg;

    if (p == NULL || handler == NULL) {
        if (p) pbuf_free(p);
        return;
    }

    // Invalidate D-Cache for received data (H7 series requirement)
    SCB_InvalidateDCache_by_Addr((uint32_t*)p->payload, p->len);

    // Parse MAVLink messages from the UDP packet
    uint8_t *data = (uint8_t *)p->payload;
    for (uint16_t i = 0; i < p->len; i++) {
        uint8_t msg_received = mavlink_parse_char(MAVLINK_COMM_0,
                                                   data[i],
                                                   &handler->rx_message,
                                                   &handler->rx_status);

        if (msg_received) {
            handler->messages_received++;

            // Call user message handler if registered
            if (g_message_handler) {
                g_message_handler(&handler->rx_message, handler);
            }
        }
    }

    // Free the pbuf
    pbuf_free(p);
}

/**
 * @brief Send a MAVLink message via UDP to configured remote address
 */
err_t mavlink_udp_send_message(mavlink_udp_t *handler, const mavlink_message_t *msg) {
    if (!handler || !handler->initialized || !msg) {
        return ERR_ARG;
    }

    return mavlink_udp_send_message_to(handler, msg,
                                       &handler->config.remote_addr,
                                       handler->config.remote_port);
}

/**
 * @brief Send a MAVLink message to a specific address
 */
err_t mavlink_udp_send_message_to(mavlink_udp_t *handler,
                                   const mavlink_message_t *msg,
                                   const ip_addr_t *addr,
                                   uint16_t port) {
    if (!handler || !handler->initialized || !msg || !addr) {
        return ERR_ARG;
    }

    // Serialize MAVLink message to buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);

    // Allocate pbuf for sending
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p == NULL) {
        return ERR_MEM;
    }

    // Copy MAVLink data to pbuf
    pbuf_take(p, buffer, len);

    // Clean D-Cache for transmitted data (H7 series requirement)
    SCB_CleanDCache_by_Addr((uint32_t*)p->payload, p->len);

    // Send UDP packet
    err_t err = udp_sendto(handler->pcb, p, addr, port);

    // Free pbuf
    pbuf_free(p);

    if (err == ERR_OK) {
        handler->messages_sent++;
    }

    return err;
}

/**
 * @brief Send a MAVLink HEARTBEAT message
 */
err_t mavlink_udp_send_heartbeat(mavlink_udp_t *handler,
                                  uint8_t type,
                                  uint8_t autopilot,
                                  uint8_t base_mode,
                                  uint8_t system_status) {
    if (!handler || !handler->initialized) {
        return ERR_ARG;
    }

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(handler->config.system_id,
                               handler->config.component_id,
                               &msg,
                               type,
                               autopilot,
                               base_mode,
                               0,  // custom_mode
                               system_status);

    return mavlink_udp_send_message(handler, &msg);
}

/**
 * @brief Get statistics
 */
void mavlink_udp_get_stats(const mavlink_udp_t *handler,
                            uint32_t *rx_count,
                            uint32_t *tx_count,
                            uint32_t *errors) {
    if (!handler) {
        return;
    }

    if (rx_count) *rx_count = handler->messages_received;
    if (tx_count) *tx_count = handler->messages_sent;
    if (errors) *errors = handler->parse_errors;
}
