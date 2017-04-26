#ifndef OS_COMMUNICATION_H
#define OS_COMMUNICATION_H

#define TOULOUSE_AMPQ_HOSTNAME "localhost"
#define TOULOUSE_AMPQ_PORT 5672
#define TOULOUSE_AMPQ_EXCHANGE "toulouse"
#define TOULOUSE_AMPQ_ROUTING_KEY "moulinrouge"

const char *form_update_os_payload(int frame_number, CPFrameVersion02 *frame);
const char *form_message_payload(const char *title, const char *type, const char *footnote);
void die_on_amqp_error(amqp_rpc_reply_t x, char const *context);
int open_amqp_conn(amqp_connection_state_t *connection);
int close_amqp_conn(amqp_connection_state_t *connection);
int send_amqp_message(amqp_connection_state_t *connection, const char *messagebody);

#endif // OS_COMMUNICATION_H