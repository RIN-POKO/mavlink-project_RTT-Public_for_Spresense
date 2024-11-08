#pragma once
// MESSAGE RTT_ACK PACKING

#define MAVLINK_MSG_ID_RTT_ACK 12922


typedef struct __mavlink_rtt_ack_t {
 uint64_t syn_send_time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
} mavlink_rtt_ack_t;

#define MAVLINK_MSG_ID_RTT_ACK_LEN 8
#define MAVLINK_MSG_ID_RTT_ACK_MIN_LEN 8
#define MAVLINK_MSG_ID_12922_LEN 8
#define MAVLINK_MSG_ID_12922_MIN_LEN 8

#define MAVLINK_MSG_ID_RTT_ACK_CRC 137
#define MAVLINK_MSG_ID_12922_CRC 137



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RTT_ACK { \
    12922, \
    "RTT_ACK", \
    1, \
    {  { "syn_send_time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rtt_ack_t, syn_send_time_usec) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RTT_ACK { \
    "RTT_ACK", \
    1, \
    {  { "syn_send_time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rtt_ack_t, syn_send_time_usec) }, \
         } \
}
#endif

/**
 * @brief Pack a rtt_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param syn_send_time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rtt_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t syn_send_time_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RTT_ACK_LEN];
    _mav_put_uint64_t(buf, 0, syn_send_time_usec);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RTT_ACK_LEN);
#else
    mavlink_rtt_ack_t packet;
    packet.syn_send_time_usec = syn_send_time_usec;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RTT_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RTT_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RTT_ACK_MIN_LEN, MAVLINK_MSG_ID_RTT_ACK_LEN, MAVLINK_MSG_ID_RTT_ACK_CRC);
}

/**
 * @brief Pack a rtt_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param syn_send_time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rtt_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t syn_send_time_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RTT_ACK_LEN];
    _mav_put_uint64_t(buf, 0, syn_send_time_usec);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RTT_ACK_LEN);
#else
    mavlink_rtt_ack_t packet;
    packet.syn_send_time_usec = syn_send_time_usec;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RTT_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RTT_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RTT_ACK_MIN_LEN, MAVLINK_MSG_ID_RTT_ACK_LEN, MAVLINK_MSG_ID_RTT_ACK_CRC);
}

/**
 * @brief Encode a rtt_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rtt_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rtt_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rtt_ack_t* rtt_ack)
{
    return mavlink_msg_rtt_ack_pack(system_id, component_id, msg, rtt_ack->syn_send_time_usec);
}

/**
 * @brief Encode a rtt_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rtt_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rtt_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rtt_ack_t* rtt_ack)
{
    return mavlink_msg_rtt_ack_pack_chan(system_id, component_id, chan, msg, rtt_ack->syn_send_time_usec);
}

/**
 * @brief Send a rtt_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param syn_send_time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rtt_ack_send(mavlink_channel_t chan, uint64_t syn_send_time_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RTT_ACK_LEN];
    _mav_put_uint64_t(buf, 0, syn_send_time_usec);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RTT_ACK, buf, MAVLINK_MSG_ID_RTT_ACK_MIN_LEN, MAVLINK_MSG_ID_RTT_ACK_LEN, MAVLINK_MSG_ID_RTT_ACK_CRC);
#else
    mavlink_rtt_ack_t packet;
    packet.syn_send_time_usec = syn_send_time_usec;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RTT_ACK, (const char *)&packet, MAVLINK_MSG_ID_RTT_ACK_MIN_LEN, MAVLINK_MSG_ID_RTT_ACK_LEN, MAVLINK_MSG_ID_RTT_ACK_CRC);
#endif
}

/**
 * @brief Send a rtt_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rtt_ack_send_struct(mavlink_channel_t chan, const mavlink_rtt_ack_t* rtt_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rtt_ack_send(chan, rtt_ack->syn_send_time_usec);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RTT_ACK, (const char *)rtt_ack, MAVLINK_MSG_ID_RTT_ACK_MIN_LEN, MAVLINK_MSG_ID_RTT_ACK_LEN, MAVLINK_MSG_ID_RTT_ACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_RTT_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rtt_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t syn_send_time_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, syn_send_time_usec);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RTT_ACK, buf, MAVLINK_MSG_ID_RTT_ACK_MIN_LEN, MAVLINK_MSG_ID_RTT_ACK_LEN, MAVLINK_MSG_ID_RTT_ACK_CRC);
#else
    mavlink_rtt_ack_t *packet = (mavlink_rtt_ack_t *)msgbuf;
    packet->syn_send_time_usec = syn_send_time_usec;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RTT_ACK, (const char *)packet, MAVLINK_MSG_ID_RTT_ACK_MIN_LEN, MAVLINK_MSG_ID_RTT_ACK_LEN, MAVLINK_MSG_ID_RTT_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE RTT_ACK UNPACKING


/**
 * @brief Get field syn_send_time_usec from rtt_ack message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_rtt_ack_get_syn_send_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Decode a rtt_ack message into a struct
 *
 * @param msg The message to decode
 * @param rtt_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_rtt_ack_decode(const mavlink_message_t* msg, mavlink_rtt_ack_t* rtt_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rtt_ack->syn_send_time_usec = mavlink_msg_rtt_ack_get_syn_send_time_usec(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RTT_ACK_LEN? msg->len : MAVLINK_MSG_ID_RTT_ACK_LEN;
        memset(rtt_ack, 0, MAVLINK_MSG_ID_RTT_ACK_LEN);
    memcpy(rtt_ack, _MAV_PAYLOAD(msg), len);
#endif
}
