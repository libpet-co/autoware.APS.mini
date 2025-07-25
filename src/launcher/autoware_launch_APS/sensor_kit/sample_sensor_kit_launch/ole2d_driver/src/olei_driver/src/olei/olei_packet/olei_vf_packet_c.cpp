#include "olei/olei_packet/olei_vf_packet_c.h"
#include "olei/olei_packet_reader.h"
void OleiVFPacket_C::read_data(uint8_t *buf, size_t num)
{
    Data* data = reinterpret_cast<Data*>(buf);
    distance.resize(num);
    for(int i = 0; i < num; i++)
    {
        distance[i] = data[i].distance;
        angle[i] = data[i].angle;
    }
}

void OleiVFPacket_C::read_with(OleiPacketReader& reader)
{
    reader.read(*this);
}
