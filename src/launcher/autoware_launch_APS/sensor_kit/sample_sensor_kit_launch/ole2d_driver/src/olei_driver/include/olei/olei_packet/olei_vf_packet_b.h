#pragma once

#include "olei/olei_packet/olei_vf_packet.h"

class OleiVFPacket_B : public OleiVFPacket
{
protected:
#pragma pack(push, pfB, 1)
  struct Data
  {
    uint16_t distance;
    uint16_t intensity;
  };
#pragma pack(pop, pfB)


  virtual void read_data(uint8_t* buf, size_t num);
  virtual void read_with(OleiPacketReader& reader);
};
