#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

struct RelayChannelConfig {
  const char *name;
  uint8_t gpio;
  bool configured;
  bool active_low;
  float nominal_voltage;
};

static const char *kWifiSsid = "Ahmedmegahed";
static const char *kWifiPassword = "6475497740";
static const char *kAgentIp = "10.248.172.21";
static const uint16_t kAgentPort = 8888;

static const size_t kRelayCount = 8;
static RelayChannelConfig kRelayConfig[kRelayCount] = {
  {"front_left_wheel_joint", 15, true, false, 24.0f},
  {"rear_left_wheel_joint", 2, true, false, 24.0f},
  {"front_right_wheel_joint", 4, true, false, 24.0f},
  {"rear_right_wheel_joint", 18, true, false, 24.0f},
  {"Relay 5", 21, false, false, 24.0f},
  {"Relay 6", 22, false, false, 24.0f},
  {"Relay 7", 23, false, false, 24.0f},
  {"Relay 8", 25, false, false, 24.0f},
};

struct RelayRuntimeState {
  bool enabled;
  unsigned long last_command_ms;
  char last_command[24];
};

static RelayRuntimeState g_relay_state[kRelayCount];

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t state_timer;
rcl_publisher_t state_pub;
rcl_publisher_t heartbeat_pub;
rcl_subscription_t command_sub;
rclc_executor_t executor;

std_msgs__msg__String command_msg;
std_msgs__msg__String state_msg;
std_msgs__msg__String heartbeat_msg;

char state_buffer[2048];
char heartbeat_buffer[64];
char command_buffer[256];

bool extract_string_field(const char *json, const char *field, char *out, size_t out_size) {
  char needle[32];
  snprintf(needle, sizeof(needle), "\"%s\"", field);
  const char *start = strstr(json, needle);
  if (start == nullptr) {
    return false;
  }
  start += strlen(needle);
  start = strchr(start, ':');
  if (start == nullptr) {
    return false;
  }
  start++;
  while (*start == ' ') {
    start++;
  }
  if (*start != '"') {
    return false;
  }
  start++;
  const char *end = strchr(start, '"');
  if (end == nullptr) {
    return false;
  }
  size_t len = static_cast<size_t>(end - start);
  if (len >= out_size) {
    len = out_size - 1;
  }
  memcpy(out, start, len);
  out[len] = '\0';
  return true;
}

bool extract_bool_field(const char *json, const char *field, bool *value) {
  char needle[32];
  snprintf(needle, sizeof(needle), "\"%s\":", field);
  const char *start = strstr(json, needle);
  if (start == nullptr) {
    return false;
  }
  start += strlen(needle);
  while (*start == ' ') {
    start++;
  }
  if (strncmp(start, "true", 4) == 0) {
    *value = true;
    return true;
  }
  if (strncmp(start, "false", 5) == 0) {
    *value = false;
    return true;
  }
  return false;
}

size_t find_relay_index_by_name(const char *name) {
  for (size_t i = 0; i < kRelayCount; ++i) {
    if (strcmp(kRelayConfig[i].name, name) == 0) {
      return i;
    }
  }
  return kRelayCount;
}

void apply_relay_state(size_t index, bool enabled, const char *command_name) {
  if (index >= kRelayCount || !kRelayConfig[index].configured) {
    return;
  }

  const bool pin_level = kRelayConfig[index].active_low ? !enabled : enabled;
  digitalWrite(kRelayConfig[index].gpio, pin_level ? HIGH : LOW);
  g_relay_state[index].enabled = enabled;
  g_relay_state[index].last_command_ms = millis();
  snprintf(g_relay_state[index].last_command, sizeof(g_relay_state[index].last_command), "%s", command_name);
}

void apply_software_kill() {
  for (size_t i = 0; i < kRelayCount; ++i) {
    if (!kRelayConfig[i].configured) {
      continue;
    }
    apply_relay_state(i, false, "KILLED");
  }
}

void publish_heartbeat() {
  snprintf(heartbeat_buffer, sizeof(heartbeat_buffer), "%lu", millis());
  heartbeat_msg.data.data = heartbeat_buffer;
  heartbeat_msg.data.size = strlen(heartbeat_buffer);
  heartbeat_msg.data.capacity = sizeof(heartbeat_buffer);
  rcl_publish(&heartbeat_pub, &heartbeat_msg, nullptr);
}

void publish_state() {
  size_t offset = 0;
  offset += snprintf(state_buffer + offset, sizeof(state_buffer) - offset, "{\"bus_voltage\":12.0,\"relays\":[");

  for (size_t i = 0; i < kRelayCount; ++i) {
    const float current = g_relay_state[i].enabled ? 0.02f : 0.0f;
    const float voltage = g_relay_state[i].enabled ? kRelayConfig[i].nominal_voltage : 0.0f;
    const float temperature = 24.0f + static_cast<float>(i);

    offset += snprintf(
      state_buffer + offset,
      sizeof(state_buffer) - offset,
      "%s{\"name\":\"%s\",\"channel\":%u,\"configured\":%s,\"enabled\":%s,"
      "\"voltage\":%.2f,\"current\":%.2f,\"temperature\":%.1f,"
      "\"health\":\"%s\",\"fault\":\"%s\",\"last_command\":\"%s\"}",
      (i == 0) ? "" : ",",
      kRelayConfig[i].name,
      static_cast<unsigned>(i),
      kRelayConfig[i].configured ? "true" : "false",
      g_relay_state[i].enabled ? "true" : "false",
      voltage,
      current,
      temperature,
      kRelayConfig[i].configured ? (g_relay_state[i].enabled ? "healthy" : "offline") : "offline",
      kRelayConfig[i].configured ? (g_relay_state[i].enabled ? "None" : "Not energized") : "Unused channel",
      g_relay_state[i].last_command
    );

    if (offset >= sizeof(state_buffer) - 8) {
      break;
    }
  }

  snprintf(state_buffer + offset, sizeof(state_buffer) - offset, "]}");
  state_msg.data.data = state_buffer;
  state_msg.data.size = strlen(state_buffer);
  state_msg.data.capacity = sizeof(state_buffer);
  rcl_publish(&state_pub, &state_msg, nullptr);
}

void state_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer == nullptr) {
    return;
  }
  publish_state();
  publish_heartbeat();
}

void command_callback(const void *msg_in) {
  const auto *msg = static_cast<const std_msgs__msg__String *>(msg_in);
  if (msg == nullptr || msg->data.data == nullptr) {
    return;
  }

  char command_type[32] = {0};
  if (!extract_string_field(msg->data.data, "type", command_type, sizeof(command_type))) {
    return;
  }

  if (strcmp(command_type, "software_kill") == 0) {
    apply_software_kill();
    return;
  }

  if (strcmp(command_type, "set_power") != 0) {
    return;
  }

  char target[32] = {0};
  bool enable = false;
  if (!extract_string_field(msg->data.data, "target", target, sizeof(target))) {
    return;
  }
  if (!extract_bool_field(msg->data.data, "enable", &enable)) {
    return;
  }

  const size_t relay_index = find_relay_index_by_name(target);
  if (relay_index >= kRelayCount) {
    return;
  }
  apply_relay_state(relay_index, enable, enable ? "ENABLE" : "DISABLE");
}

void init_relay_outputs() {
  for (size_t i = 0; i < kRelayCount; ++i) {
    pinMode(kRelayConfig[i].gpio, OUTPUT);
    snprintf(g_relay_state[i].last_command, sizeof(g_relay_state[i].last_command), "INIT");
    apply_relay_state(i, false, "INIT");
  }
}

void setup() {
  set_microros_wifi_transports(
    const_cast<char *>(kWifiSsid),
    const_cast<char *>(kWifiPassword),
    const_cast<char *>(kAgentIp),
    kAgentPort
  );

  init_relay_outputs();
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, nullptr, &allocator);
  rclc_node_init_default(&node, "esp32_relay_controller", "", &support);

  rclc_publisher_init_default(
    &state_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/rover/relay_board/state"
  );

  rclc_publisher_init_default(
    &heartbeat_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/rover/relay_board/heartbeat"
  );

  rclc_subscription_init_default(
    &command_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/rover/relay_board/command"
  );

  rclc_timer_init_default(&state_timer, &support, RCL_MS_TO_NS(500), state_timer_callback);

  command_msg.data.data = command_buffer;
  command_msg.data.size = 0;
  command_msg.data.capacity = sizeof(command_buffer);

  state_msg.data.data = state_buffer;
  state_msg.data.size = 0;
  state_msg.data.capacity = sizeof(state_buffer);

  heartbeat_msg.data.data = heartbeat_buffer;
  heartbeat_msg.data.size = 0;
  heartbeat_msg.data.capacity = sizeof(heartbeat_buffer);

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &command_sub, &command_msg, &command_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &state_timer);
}

void loop() {
  delay(10);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
