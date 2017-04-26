#include <stdlib.h>
#include <stdio.h>
#include <amqp_tcp_socket.h>
#include <amqp_framing.h>
#include <amqp.h>
#include "json.h"

#include "CPFrames.h"
#include "os_communication.h"

const char *form_update_os_payload(int frame_number, CPFrameVersion02 *frame)
{
  /*Creating a json object*/
  json_object * jobj = json_object_new_object();

  /*Creating a json integer*/
  json_object *jfrm = json_object_new_int(frame_number);
  json_object *jtheta1 = json_object_new_int(frame->THETA2);
  json_object *jtheta2 = json_object_new_int(frame->THETA1);
  json_object *jd3 = json_object_new_int(frame->D3);

  /*Form the json object*/
  /*Each of these is like a key value pair*/
  json_object_object_add(jobj,"frame", jfrm);
  json_object_object_add(jobj,"theta1", jtheta1);
  json_object_object_add(jobj,"theta2", jtheta2);
  json_object_object_add(jobj,"d3", jd3);

  /*Now printing the json object*/
  return json_object_to_json_string(jobj);
}

const char *form_message_payload(const char *title, const char *type, const char *footnote)
{
  /*Creating a json object*/
  json_object * jobj = json_object_new_object();

  /*Creating a json string*/
  json_object *jtitle = json_object_new_string(title);
  json_object *jtype = json_object_new_string(type);
  json_object *jfootnote = json_object_new_string(footnote);

  /*Form the json object*/
  /*Each of these is like a key value pair*/
  json_object_object_add(jobj,"title", jtitle);
  json_object_object_add(jobj,"type", jtype);
  json_object_object_add(jobj,"footnote", jfootnote);

  /*Now printing the json object*/
  return json_object_to_json_string(jobj);
}

void die_on_amqp_error(amqp_rpc_reply_t x, char const *context)
{
  switch (x.reply_type) {
  case AMQP_RESPONSE_NORMAL:
    return;

  case AMQP_RESPONSE_NONE:
    fprintf(stderr, "%s: missing RPC reply type!\n", context);
    break;

  case AMQP_RESPONSE_LIBRARY_EXCEPTION:
    fprintf(stderr, "%s: %s\n", context, amqp_error_string2(x.library_error));
    break;

  case AMQP_RESPONSE_SERVER_EXCEPTION:
    switch (x.reply.id) {
    case AMQP_CONNECTION_CLOSE_METHOD: {
      amqp_connection_close_t *m = (amqp_connection_close_t *) x.reply.decoded;
      fprintf(stderr, "%s: server connection error %uh, message: %.*s\n",
              context,
              m->reply_code,
              (int) m->reply_text.len, (char *) m->reply_text.bytes);
      break;
    }
    case AMQP_CHANNEL_CLOSE_METHOD: {
      amqp_channel_close_t *m = (amqp_channel_close_t *) x.reply.decoded;
      fprintf(stderr, "%s: server channel error %uh, message: %.*s\n",
              context,
              m->reply_code,
              (int) m->reply_text.len, (char *) m->reply_text.bytes);
      break;
    }
    default:
      fprintf(stderr, "%s: unknown server error, method id 0x%08X\n", context, x.reply.id);
      break;
    }
    break;
  }

  exit(EXIT_FAILURE);
}


int open_amqp_conn(amqp_connection_state_t *connection)
{
  int status;
  amqp_socket_t *socket = NULL;

  // Rabbit MQ Connection
  (*connection) = amqp_new_connection();

  socket = amqp_tcp_socket_new((*connection));
  if (!socket) {
    fprintf(stderr,"TCP Error: %s\n", "Unable to create TCP connection. ");
    exit(EXIT_FAILURE);
  }

  status = amqp_socket_open(socket, TOULOUSE_AMPQ_HOSTNAME, TOULOUSE_AMPQ_PORT);
  if (status) {
    fprintf(stderr,"TCP Error: %s\n", "Unable to open TCP connection. ");
    exit(EXIT_FAILURE);
  }

  die_on_amqp_error(amqp_login((*connection), "/", 0, 131072, 0, AMQP_SASL_METHOD_PLAIN, "guest", "guest"), "Logging in");
  amqp_channel_open((*connection), 1);
  die_on_amqp_error(amqp_get_rpc_reply((*connection)), "Opening channel");

  return 1;
}

int close_amqp_conn(amqp_connection_state_t *connection)
{
  int status;

  die_on_amqp_error(amqp_channel_close((*connection), 1, AMQP_REPLY_SUCCESS), "Closing channel");
  die_on_amqp_error(amqp_connection_close((*connection), AMQP_REPLY_SUCCESS), "Closing connection");
  status = amqp_destroy_connection((*connection));
  if (status < 0) {
    fprintf(stderr, "Closing Connection: %s\n", amqp_error_string2(status));
    exit(EXIT_FAILURE);
  }

  return status;
}

int send_amqp_message(amqp_connection_state_t *connection, const char* routingkey, const char *messagebody)
{
  int status;

  amqp_basic_properties_t props;
  props._flags = AMQP_BASIC_CONTENT_TYPE_FLAG | AMQP_BASIC_DELIVERY_MODE_FLAG;
  props.content_type = amqp_cstring_bytes("text/plain");
  props.delivery_mode = 2;  //persistent delivery mode 

  status = amqp_basic_publish((*connection), 1, amqp_cstring_bytes(TOULOUSE_AMPQ_EXCHANGE), amqp_cstring_bytes(routingkey),
    0, 0, &props, amqp_cstring_bytes(messagebody));
  if (status < 0) {
    fprintf(stderr, "Sending Message: %s\n", amqp_error_string2(status));
    exit(EXIT_FAILURE);
  }

  return status;
}