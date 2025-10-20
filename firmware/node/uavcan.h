#ifndef UAVCAN_H
#define UAVCAN_H

#include <ch.h>
#include <hal.h>
#include <canard.h>

// UAVCAN messages
#include <uavcan.protocol.GetNodeInfo.h>
#include <uavcan.protocol.NodeStatus.h>
#include <uavcan.protocol.RestartNode.h>
#include <uavcan.protocol.dynamic_node_id.Allocation.h>
#include <uavcan.protocol.debug.LogMessage.h>
#include <uavcan.protocol.file.BeginFirmwareUpdate.h>
#include <uavcan.protocol.file.Read.h>
#include <uavcan.equipment.esc.RawCommand.h>
#include <uavcan.equipment.esc.Status.h>
#include <uavcan.equipment.device.Temperature.h>
#include <uavcan.equipment.actuator.Status.h>
#include <uavcan.equipment.actuator.ArrayCommand.h>
#include <uavcan.equipment.range_sensor.Measurement.h>
#include <uavcan.equipment.power.BatteryInfo.h>
#include <uavcan.equipment.power.CircuitStatus.h>
#include <uavcan.equipment.fuelcell.Status.h>
#include <uavcan.protocol.param.GetSet.h>
#include <uavcan.protocol.param.ExecuteOpcode.h>
#include <uavcan.protocol.param.ExecuteOpcode.h>
#include <uavcan.tunnel.Call.h>


struct uavcan_iface_t {
  CANDriver *can_driver;
  uint32_t can_baudrate;
  CANConfig can_cfg;

  event_source_t tx_request;
  mutex_t mutex;
  thread_t *thread_rx;
  thread_t *thread_tx;
  thread_t *thread_uavcan;

  uint8_t node_id;
  CanardInstance canard;
  uint8_t canard_memory_pool[1024];

  // Dynamic node id allocation
  uint32_t send_next_node_id_allocation_request_at_ms;
  uint8_t node_id_allocation_unique_id_offset;

  // Errors
  uint32_t transmit_err_cnt;
  uint32_t transmit_err_flush_cnt;
};

void uavcanInit(void);

int uavcanRequestOrRespond(struct uavcan_iface_t *iface, uint8_t destination_node_id, uint64_t data_type_signature,
    uint8_t data_type_id, uint8_t* inout_transfer_id, uint8_t priority, CanardRequestResponse kind,
    const void* payload, uint16_t payload_len);

int uavcanBroadcast(struct uavcan_iface_t *iface, uint64_t data_type_signature, uint16_t data_type_id,
    uint8_t* inout_transfer_id, uint8_t priority, const void* payload, uint16_t payload_len);

int uavcanBroadcastAll(uint64_t data_type_signature, uint16_t data_type_id,
    uint8_t* inout_transfer_id, uint8_t priority, const void* payload, uint16_t payload_len);

void uavcanDebug(uint8_t level, char *source, char *msg);
void uavcanDebugIface(struct uavcan_iface_t *iface, uint8_t level, char *source, char *msg);

// // Thread management functions
// bool uavcanThreadsRunning(struct uavcan_iface_t *iface);
// void uavcanTerminateThreads(struct uavcan_iface_t *iface);

#endif /* UAVCAN_H */