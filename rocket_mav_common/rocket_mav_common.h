#ifndef MAV_COMMON_H
#define MAV_COMMON_H

#include <ctype.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MAV_DEFAULT_IP "127.0.0.1"
#define MAV_DEFAULT_PORT 14550
#define MAV_DEFAULT_SYSID 255
#define MAV_DEFAULT_COMPID 190
#define MAV_DEFAULT_TARGET_SYS 1
#define MAV_DEFAULT_TARGET_COMP 1

#define MAV_DEFAULT_TMP_PARAMS "/tmp/config.params"
#define MAV_DEFAULT_PERSIST_PARAMS "/home/rocket/mavlink_projects/scan/config.params"

#define MAV_SYSTEM_PORTS_ENV_PATH "/etc/rocket-mav/ports.env"
#define MAV_SYSTEM_RATE_ENV_PATH "/etc/rocket-mav/rate.env"
#define MAV_USER_PORTS_ENV_REL ".config/rocket-mav/ports.env"
#define MAV_USER_RATE_ENV_REL ".config/rocket-mav/rate.env"

#define MAV_CFG_MAX_ENTRIES 256
#define MAV_CFG_KEY_LEN 64
#define MAV_CFG_VAL_LEN 192

#define MAV_CFG_KEY_SCAN_LISTEN_IP "ROCKET_MAV_SCAN_LISTEN_IP"
#define MAV_CFG_KEY_SCAN_LISTEN_PORT "ROCKET_MAV_SCAN_LISTEN_PORT"
#define MAV_CFG_KEY_SCAN_TARGET_IP "ROCKET_MAV_SCAN_TARGET_IP"
#define MAV_CFG_KEY_SCAN_TARGET_PORT "ROCKET_MAV_SCAN_TARGET_PORT"
#define MAV_CFG_KEY_SCAN_SYSID "ROCKET_MAV_SCAN_SYSID"
#define MAV_CFG_KEY_SCAN_COMPID "ROCKET_MAV_SCAN_COMPID"
#define MAV_CFG_KEY_SCAN_TARGET_SYS "ROCKET_MAV_SCAN_TARGET_SYS"
#define MAV_CFG_KEY_SCAN_TARGET_COMP "ROCKET_MAV_SCAN_TARGET_COMP"
#define MAV_CFG_KEY_SCAN_SERIAL_BAUD "ROCKET_MAV_SCAN_SERIAL_BAUD"
#define MAV_CFG_KEY_SCAN_PARAM_TIMEOUT_SEC "ROCKET_MAV_SCAN_PARAM_TIMEOUT_SEC"
#define MAV_CFG_KEY_SCAN_DURATION_SEC "ROCKET_MAV_SCAN_DURATION_SEC"

#define MAV_CFG_KEY_TOOLS_TARGET_IP "ROCKET_MAV_TOOLS_TARGET_IP"
#define MAV_CFG_KEY_TOOLS_TARGET_PORT "ROCKET_MAV_TOOLS_TARGET_PORT"
#define MAV_CFG_KEY_TOOLS_SYSID "ROCKET_MAV_TOOLS_SYSID"
#define MAV_CFG_KEY_TOOLS_COMPID "ROCKET_MAV_TOOLS_COMPID"
#define MAV_CFG_KEY_TOOLS_TARGET_SYS "ROCKET_MAV_TOOLS_TARGET_SYS"
#define MAV_CFG_KEY_TOOLS_TARGET_COMP "ROCKET_MAV_TOOLS_TARGET_COMP"
#define MAV_CFG_KEY_TOOLS_LISTEN_PORT "ROCKET_MAV_TOOLS_LISTEN_PORT"

#define MAV_CFG_KEY_SERVO_COMPID "ROCKET_MAV_SERVO_COMPID"
#define MAV_CFG_KEY_MOTOR_COMPID "ROCKET_MAV_MOTOR_COMPID"
#define MAV_CFG_KEY_STREAM_CMD_COMPID "ROCKET_MAV_STREAM_CMD_COMPID"
#define MAV_CFG_KEY_TOOLS_SERIAL_BAUD "ROCKET_MAV_TOOLS_SERIAL_BAUD"
#define MAV_CFG_KEY_SERVO_LOOP_HZ "ROCKET_MAV_SERVO_LOOP_HZ"
#define MAV_CFG_KEY_MOTOR_LOOP_HZ "ROCKET_MAV_MOTOR_LOOP_HZ"

#define MAV_CFG_KEY_STREAM_ODOM_LISTEN_IP "ROCKET_MAV_STREAM_ODOM_LISTEN_IP"
#define MAV_CFG_KEY_STREAM_ODOM_LISTEN_PORT "ROCKET_MAV_STREAM_ODOM_LISTEN_PORT"
#define MAV_CFG_KEY_STREAM_ODOM_OUT_IP "ROCKET_MAV_STREAM_ODOM_OUT_IP"
#define MAV_CFG_KEY_STREAM_ODOM_OUT_PORT "ROCKET_MAV_STREAM_ODOM_OUT_PORT"
#define MAV_CFG_KEY_STREAM_ODOM_RATE_HZ "ROCKET_MAV_STREAM_ODOM_RATE_HZ"
#define MAV_CFG_KEY_STREAM_ODOM_STATE_PATH "ROCKET_MAV_STREAM_ODOM_STATE_PATH"

#define MAV_CFG_KEY_STREAM_CMD_LISTEN_IP "ROCKET_MAV_STREAM_CMD_LISTEN_IP"
#define MAV_CFG_KEY_STREAM_CMD_LISTEN_PORT "ROCKET_MAV_STREAM_CMD_LISTEN_PORT"
#define MAV_CFG_KEY_STREAM_CMD_TARGET_IP "ROCKET_MAV_STREAM_CMD_TARGET_IP"
#define MAV_CFG_KEY_STREAM_CMD_TARGET_PORT "ROCKET_MAV_STREAM_CMD_TARGET_PORT"
#define MAV_CFG_KEY_STREAM_CMD_RATE_HZ "ROCKET_MAV_STREAM_CMD_RATE_HZ"
#define MAV_CFG_KEY_STREAM_CMD_STATE_PATH "ROCKET_MAV_STREAM_CMD_STATE_PATH"
#define MAV_CFG_KEY_STREAM_CMD_TEST_LISTEN_IP "ROCKET_MAV_STREAM_CMD_TEST_LISTEN_IP"
#define MAV_CFG_KEY_STREAM_CMD_TEST_LISTEN_PORT "ROCKET_MAV_STREAM_CMD_TEST_LISTEN_PORT"
#define MAV_CFG_KEY_STREAM_CMD_TEST_CONSOLE_BPS "ROCKET_MAV_STREAM_CMD_TEST_CONSOLE_BPS"

#define MAV_CFG_KEY_STREAM_ODOM_TEST_LISTEN_IP "ROCKET_MAV_STREAM_ODOM_TEST_LISTEN_IP"
#define MAV_CFG_KEY_STREAM_ODOM_TEST_LISTEN_PORT "ROCKET_MAV_STREAM_ODOM_TEST_LISTEN_PORT"
#define MAV_CFG_KEY_STREAM_ODOM_TEST_CONSOLE_BPS "ROCKET_MAV_STREAM_ODOM_TEST_CONSOLE_BPS"
#define MAV_CFG_KEY_STREAM_ODOM_TEST_PRINT_HZ "ROCKET_MAV_STREAM_ODOM_TEST_PRINT_HZ"

#define MAV_CFG_KEY_PARAM_FETCH_TIMEOUT_SEC "ROCKET_MAV_PARAM_FETCH_TIMEOUT_SEC"
#define MAV_CFG_KEY_PARAM_FETCH_RETRIES "ROCKET_MAV_PARAM_FETCH_RETRIES"

#define MAV_CFG_KEY_HEARTBEAT_WATCH_LISTEN_IP "ROCKET_MAV_HEARTBEAT_WATCH_LISTEN_IP"
#define MAV_CFG_KEY_HEARTBEAT_WATCH_LISTEN_PORT "ROCKET_MAV_HEARTBEAT_WATCH_LISTEN_PORT"

#define MAV_CFG_KEY_MONITORING_LISTEN_IP "ROCKET_MAV_MONITORING_LISTEN_IP"
#define MAV_CFG_KEY_MONITORING_LISTEN_PORT "ROCKET_MAV_MONITORING_LISTEN_PORT"
#define MAV_CFG_KEY_MONITORING_OUTPUT_FILE "ROCKET_MAV_MONITORING_OUTPUT_FILE"
#define MAV_CFG_KEY_MONITORING_QUERY_MAX_WAIT_SEC "ROCKET_MAV_MONITORING_QUERY_MAX_WAIT_SEC"
#define MAV_CFG_KEY_MONITORING_QUERY_SELECT_TIMEOUT_SEC "ROCKET_MAV_MONITORING_QUERY_SELECT_TIMEOUT_SEC"
#define MAV_CFG_KEY_MONITORINGD_SELECT_TIMEOUT_SEC "ROCKET_MAV_MONITORINGD_SELECT_TIMEOUT_SEC"
#define MAV_CFG_KEY_MONITORINGD_WRITE_INTERVAL_SEC "ROCKET_MAV_MONITORINGD_WRITE_INTERVAL_SEC"
#define MAV_CFG_KEY_MONITORINGD_TIME_SYNC_THRESHOLD_SEC "ROCKET_MAV_MONITORINGD_TIME_SYNC_THRESHOLD_SEC"
#define MAV_CFG_KEY_MONITORINGD_TIME_SYNC_RETRY_INTERVAL_SEC "ROCKET_MAV_MONITORINGD_TIME_SYNC_RETRY_INTERVAL_SEC"

#define MAV_CFG_KEY_ARM_REASON_LISTEN_IP "ROCKET_MAV_ARM_REASON_LISTEN_IP"
#define MAV_CFG_KEY_ARM_REASON_LISTEN_PORT "ROCKET_MAV_ARM_REASON_LISTEN_PORT"

#define MAV_CFG_KEY_PARAMS_TMP_PATH "ROCKET_MAV_PARAMS_TMP_PATH"
#define MAV_CFG_KEY_PARAMS_PERSIST_PATH "ROCKET_MAV_PARAMS_PERSIST_PATH"
#define MAV_CFG_KEY_LEGACY_PARAMS_TMP_PATH "MAV_PARAMS_TMP_PATH"
#define MAV_CFG_KEY_LEGACY_PARAMS_PERSIST_PATH "MAV_PARAMS_PERSIST_PATH"

typedef struct {
    char key[MAV_CFG_KEY_LEN];
    char value[MAV_CFG_VAL_LEN];
} mav_cfg_kv_t;

typedef struct {
    int loaded;
    mav_cfg_kv_t sys_table[MAV_CFG_MAX_ENTRIES];
    mav_cfg_kv_t user_table[MAV_CFG_MAX_ENTRIES];
    size_t sys_count;
    size_t user_count;
} mav_cfg_state_t;

static inline void mav_cfg_trim(char *s) {
    char *p = s;
    while (*p && isspace((unsigned char)*p)) p++;
    if (p != s) memmove(s, p, strlen(p) + 1);
    size_t len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1])) {
        s[len - 1] = '\0';
        len--;
    }
}

static inline void mav_cfg_store_kv(mav_cfg_kv_t *table, size_t *count, const char *key, const char *value) {
    for (size_t i = 0; i < *count; ++i) {
        if (strcmp(table[i].key, key) == 0) {
            snprintf(table[i].value, sizeof(table[i].value), "%s", value);
            return;
        }
    }
    if (*count >= MAV_CFG_MAX_ENTRIES) return;
    snprintf(table[*count].key, sizeof(table[*count].key), "%s", key);
    snprintf(table[*count].value, sizeof(table[*count].value), "%s", value);
    (*count)++;
}

static inline void mav_cfg_load_file(const char *path, mav_cfg_kv_t *table, size_t *count) {
    FILE *fp = fopen(path, "r");
    if (!fp) return;

    char line[512];
    while (fgets(line, sizeof(line), fp)) {
        mav_cfg_trim(line);
        if (line[0] == '\0' || line[0] == '#') continue;

        if (strncmp(line, "export ", 7) == 0) {
            memmove(line, line + 7, strlen(line + 7) + 1);
            mav_cfg_trim(line);
        }

        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        char *key = line;
        char *value = eq + 1;
        mav_cfg_trim(key);
        mav_cfg_trim(value);
        if (key[0] == '\0' || value[0] == '\0') continue;

        size_t vlen = strlen(value);
        if (vlen >= 2 && ((value[0] == '"' && value[vlen - 1] == '"') || (value[0] == '\'' && value[vlen - 1] == '\''))) {
            value[vlen - 1] = '\0';
            value++;
            mav_cfg_trim(value);
        }
        mav_cfg_store_kv(table, count, key, value);
    }
    fclose(fp);
}

static inline const char *mav_cfg_lookup(const mav_cfg_kv_t *table, size_t count, const char *key) {
    for (size_t i = 0; i < count; ++i) {
        if (strcmp(table[i].key, key) == 0) return table[i].value;
    }
    return NULL;
}

static inline mav_cfg_state_t *mav_cfg_state(void) {
    static mav_cfg_state_t state;
    return &state;
}

static inline void mav_cfg_load_once(void) {
    mav_cfg_state_t *state = mav_cfg_state();
    if (state->loaded) return;
    state->loaded = 1;

    mav_cfg_load_file(MAV_SYSTEM_PORTS_ENV_PATH, state->sys_table, &state->sys_count);
    mav_cfg_load_file(MAV_SYSTEM_RATE_ENV_PATH, state->sys_table, &state->sys_count);

    const char *home = getenv("HOME");
    if (home && home[0]) {
        char user_path[PATH_MAX];
        snprintf(user_path, sizeof(user_path), "%s/%s", home, MAV_USER_PORTS_ENV_REL);
        mav_cfg_load_file(user_path, state->user_table, &state->user_count);
        snprintf(user_path, sizeof(user_path), "%s/%s", home, MAV_USER_RATE_ENV_REL);
        mav_cfg_load_file(user_path, state->user_table, &state->user_count);
    }
}

static inline const char *mav_cfg_get_str(const char *key, const char *fallback) {
    mav_cfg_load_once();
    mav_cfg_state_t *state = mav_cfg_state();
    const char *v = getenv(key); // explicit process env has highest priority
    if (v && v[0] != '\0') return v;

    v = mav_cfg_lookup(state->user_table, state->user_count, key);
    if (v && v[0] != '\0') return v;

    v = mav_cfg_lookup(state->sys_table, state->sys_count, key);
    if (v && v[0] != '\0') return v;

    return fallback;
}

static inline int mav_cfg_get_int(const char *key, int fallback) {
    const char *v = mav_cfg_get_str(key, NULL);
    if (!v || !v[0]) return fallback;

    char *end = NULL;
    long n = strtol(v, &end, 10);
    if (end == v || (end && *end != '\0')) return fallback;
    if (n < INT_MIN || n > INT_MAX) return fallback;
    return (int)n;
}

static inline double mav_cfg_get_double(const char *key, double fallback) {
    const char *v = mav_cfg_get_str(key, NULL);
    if (!v || !v[0]) return fallback;

    char *end = NULL;
    double d = strtod(v, &end);
    if (end == v || (end && *end != '\0')) return fallback;
    return d;
}

static inline const char *mav_select_params_path(void) {
    const char *tmp_path = mav_cfg_get_str(
        MAV_CFG_KEY_PARAMS_TMP_PATH,
        mav_cfg_get_str(MAV_CFG_KEY_LEGACY_PARAMS_TMP_PATH, MAV_DEFAULT_TMP_PARAMS));
    const char *persist_path = mav_cfg_get_str(
        MAV_CFG_KEY_PARAMS_PERSIST_PATH,
        mav_cfg_get_str(MAV_CFG_KEY_LEGACY_PARAMS_PERSIST_PATH, MAV_DEFAULT_PERSIST_PARAMS));
    if (tmp_path && access(tmp_path, R_OK) == 0) return tmp_path;
    if (persist_path && access(persist_path, R_OK) == 0) return persist_path;
    return NULL;
}

#endif
