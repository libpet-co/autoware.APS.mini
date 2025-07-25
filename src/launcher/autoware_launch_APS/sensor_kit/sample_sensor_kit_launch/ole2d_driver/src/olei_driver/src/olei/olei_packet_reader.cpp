#include "olei/olei_packet_reader.h"

void OleiPacketReader::read(std::shared_ptr<OleiPacket> packet)
{
    packet->read_with(*this);
}

bool OleiPacketReader::start()
{
    return false;
}

bool OleiPacketReader::stop()
{
    return false;
}
