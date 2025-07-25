#pragma once

#include "olei/olei_packet/olei_vf_packet.h"

class OleiVFPacket_A : public OleiVFPacket
{
protected:
#pragma pack(push, pfA, 1)
  struct Data
  {
    uint16_t distance;
  };
#pragma pack(pop, pfA)

  virtual void read_data(uint8_t* buf, size_t num);
  virtual void read_with(OleiPacketReader& reader);
};
