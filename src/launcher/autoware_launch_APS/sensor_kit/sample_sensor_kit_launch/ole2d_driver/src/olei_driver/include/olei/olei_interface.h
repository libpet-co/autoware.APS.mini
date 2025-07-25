#pragma once

#include <string>
#include <memory>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include"olei/olei_packet_reader.h"
#include"olei/olei_writer.h"
#include "communication/transport.h"
#include "handle_info.h"
#include "olei/olei_parser.h"
#include "pipeline.h"
#include <iostream>

class OleiInterface
{
    public:
  OleiInterface(std::shared_ptr<rclcpp::Node> node);

  bool init(std::shared_ptr<HandleInfo> info, std::shared_ptr<ScanConfig> config,
            std::shared_ptr<ScanParameters> params, const std::string& topic, const std::string& frame_id,
            const uint16_t num_layers,const int version);

  bool start_transmission(std::shared_ptr<std::mutex> net_mtx, std::shared_ptr<std::condition_variable> net_cv,
                          bool& net_fail);
  void stop_transmission();
  void terminate();

private:
  using PipelinePtr = std::unique_ptr<Pipeline>;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::unique_ptr<Transport> transport_;
  std::shared_ptr<Reader<OleiPacket>> reader_;
  std::string topic_;
  std::string frame_id_;
  uint16_t num_layers_;
  int version_;
  PipelinePtr pipeline_;
  std::shared_ptr<HandleInfo> info_;
  std::shared_ptr<ScanConfig> config_;
  std::shared_ptr<ScanParameters> params_;
  std::string prev_handle_;

  enum class OleiState
  {
    UNINIT,
    INIT,
    RUNNING,
    SHUTDOWN,
    ERROR
  };
  OleiState state_;

  bool init();

  void change_state(OleiState state);
  bool can_change_state(OleiState state);

  void start_watchdog_timer(float duration);
  void feed_watchdog();  // timer based
  void on_shutdown();

  // factory functions
  bool handle_version(int major_version, int minor_version, int device_family, const std::string& topic,
                      const std::string& frame_id, const uint16_t num_layers);
  void connection_failure_cb();
  PipelinePtr get_pipeline(const std::string& packet_type, std::shared_ptr<std::mutex> net_mtx,
                           std::shared_ptr<std::condition_variable> net_cv, bool& net_fail);
};