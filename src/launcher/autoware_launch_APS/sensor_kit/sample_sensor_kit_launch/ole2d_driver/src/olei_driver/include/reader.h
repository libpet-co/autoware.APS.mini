#pragma once

#include<string>
#include <memory>

struct ScanConfig
{
  bool watchdog = false;
  uint watchdogtimeout = 0;
  std::string packet_type = "";
  int start_angle = -180;
  int end_angle = 180;
  int version = 2;
  float range_min = 0.5;
  float range_max = 20;
  uint max_num_points_scan = 0;
  uint skip_scans = 0;
};

#pragma pack(push, sp, 1)
struct ScanParameters
{
  double angular_fov = 0.0;
  double radial_range_min = 0.0;
  double radial_range_max = 0.0;
  double angle_min = 0.0;
  double angle_max = 0.0;
  uint16_t layers_enabled = 0;
  double scan_freq = 0.0;        
  uint16_t h_enabled_layer = 0;  
  bool apply_correction = true;
};
#pragma pack(pop, sp)

template <typename T>
class Reader
{
public:
  virtual void read(std::shared_ptr<T> packet) = 0;
  virtual void set_scanoutput_config(ScanConfig config)
  {
  }
  virtual void set_scan_params(ScanParameters params)
  {
  }
  virtual bool start()
  {
    return false;
  }
  virtual bool stop()
  {
    return false;
  }
};