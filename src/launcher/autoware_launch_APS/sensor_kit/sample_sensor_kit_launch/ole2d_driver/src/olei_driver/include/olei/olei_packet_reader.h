#pragma once

#include "reader.h"
#include "olei/olei_packet/olei_packet.h"

class OleiVFPacket_A;
class OleiVFPacket_B;
class OleiVFPacket_C;
class Olei2Packet_A;

class OleiPacketReader : public Reader<OleiPacket>, public std::enable_shared_from_this<OleiPacketReader>
{
public:
  virtual void read(std::shared_ptr<OleiPacket> packet);

  virtual void read(OleiVFPacket_A& packet) = 0;
  virtual void read(OleiVFPacket_B& packet) = 0;
  virtual void read(OleiVFPacket_C& packet) = 0;
  virtual void read(Olei2Packet_A& packet) = 0;

  virtual bool start();
  virtual bool stop();
};
