#include "olei/olei_interface.h"
#include "communication/tcp_transport.h"
#include "communication/udp_transport.h"
#include "ros/lidar_publish.h"
OleiInterface::OleiInterface(std::shared_ptr<rclcpp::Node> node):node_(node)
{
}

bool OleiInterface::init(std::shared_ptr<HandleInfo> info, 
std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params, 
const std::string &topic, const std::string &frame_id, const uint16_t num_layers,const int version)
{

    info_ = info;
    config_ = config;
    params_ = params;
    topic_ = topic;
    frame_id_ = frame_id;
    num_layers_ = num_layers;
    version_  = version;
    if (info->handle_type == HandleInfo::HANDLE_TYPE_UDP)
    {
      transport_ = std::make_unique<UDPTransport>(info->hostname,info->port);
      if (!transport_->connect())
      {
        RCLCPP_ERROR(node_->get_logger(), "Unable to establish UDP connection");
        return false;
      }
      info->endpoint = transport_->get_host_ip();
      info->port = transport_->get_port();
    }
    else if (info->handle_type == HandleInfo::HANDLE_TYPE_TCP)
    {
      transport_ = std::make_unique<TCPTransport>(info->hostname);
      // if initially port was not set, request_handle sets it
      // set the updated port in transport
      transport_->set_port(info_->port);
      if (!transport_->connect())
      {
        RCLCPP_ERROR(node_->get_logger(), "Unable to establish TCP connection");
        return false;
      }
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Incorrect transport option");
      return false;
    }   
    return true;
}

bool OleiInterface::start_transmission(std::shared_ptr<std::mutex> net_mtx, 
std::shared_ptr<std::condition_variable> net_cv, bool &net_fail)
{
  if (pipeline_ && pipeline_->is_running())
    return true;
  pipeline_ = get_pipeline(config_->packet_type, net_mtx, net_cv, net_fail);
  if (!pipeline_ || !pipeline_->start())
    return false;
  return true;
}

void OleiInterface::terminate()
{
  if (!pipeline_)
    return;

  if (config_->watchdog)
  {
    watchdog_timer_->cancel();
  }
  pipeline_->terminate();
  pipeline_.reset();
  transport_.reset();
}

void OleiInterface::connection_failure_cb()
{
  std::cout << "handling connection failure" << std::endl;
  terminate();
  std::cout << "terminated" << std::endl;
  while (!init())
  {
    std::cout << "trying to reconnect..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

std::unique_ptr<Pipeline> OleiInterface::get_pipeline(const std::string &packet_type, std::shared_ptr<std::mutex> net_mtx, std::shared_ptr<std::condition_variable> net_cv, bool &net_fail)
{
  std::shared_ptr<Parser<OleiPacket>> parser;
  std::shared_ptr<Writer<OleiPacket>> writer;
  if (version_ == 3)
  {
    RCLCPP_DEBUG(node_->get_logger(), "PacketType is: %s", packet_type.c_str());
    if (packet_type == "A")
    {
      parser = std::unique_ptr<Parser<OleiPacket>>(new OleiVFPacket_A_Parser);
    }
    else if (packet_type == "B")
    {
      parser = std::unique_ptr<Parser<OleiPacket>>(new OleiVFPacket_B_Parser);
    }
    else if (packet_type == "C")
    {
      parser = std::unique_ptr<Parser<OleiPacket>>(new OleiVFPacket_C_Parser);
    }
  }
  else if (version_ == 2)
  {
    
    parser = std::unique_ptr<Parser<OleiPacket>>(new Olei2Packet_A_Parser);
  }
  if (!parser)
  {
    return nullptr;
  }
  reader_ = std::shared_ptr<OleiPacketReader>(new LidarPublisher(node_,config_,params_,
  topic_.c_str(),frame_id_.c_str()));
  writer =
      std::shared_ptr<Writer<OleiPacket>>(new OleiWriter<OleiPacket>(std::move(transport_), parser, node_->get_logger()));
  return std::make_unique<Pipeline>(writer, reader_, std::bind(&OleiInterface::connection_failure_cb, this), net_mtx,
                                    net_cv, net_fail);
}

bool OleiInterface::init()
{
    return init(info_, config_, params_, topic_, frame_id_, num_layers_ ,version_);;
}
