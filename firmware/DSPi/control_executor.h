/*
 * control_executor.h — transport-neutral DSPi control command executor
 *
 * The executor uses USB vendor request field names intentionally.  USB is the
 * compatibility contract, and I2C/UART frames carry the same logical request
 * shape so every transport reaches the same command semantics.
 */

#ifndef CONTROL_EXECUTOR_H
#define CONTROL_EXECUTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CONTROL_DIR_OUT = 0,
    CONTROL_DIR_IN  = 1,
} ControlDirection;

typedef struct {
    ControlDirection direction;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} ControlRequest;

typedef struct {
    uint8_t *data;
    uint16_t length;
    bool immediate_status;
} ControlOutBuffer;

typedef struct {
    const uint8_t *data;
    uint16_t length;
    bool enter_bootloader;
} ControlResponse;

bool control_executor_prepare_out(const ControlRequest *request,
                                  ControlOutBuffer *out);
void control_executor_execute_out(const ControlRequest *request,
                                  const uint8_t *payload,
                                  uint16_t payload_len);
void control_executor_commit_out(const ControlRequest *request);
bool control_executor_execute_in(const ControlRequest *request,
                                 ControlResponse *response);
void control_executor_enter_bootloader(void);

#endif // CONTROL_EXECUTOR_H
