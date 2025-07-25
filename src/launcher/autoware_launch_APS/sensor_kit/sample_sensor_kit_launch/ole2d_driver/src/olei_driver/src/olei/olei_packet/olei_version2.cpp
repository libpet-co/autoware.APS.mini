#include "olei/olei_packet/olei_version2.h"

size_t Olei2Packet::get_size()
{
    return header_size;
}

std::tuple<uint16_t, uint32_t, uint16_t> Olei2Packet::read_header(boost::shared_array<uint8_t> buffer)
{
    uint8_t* buf = buffer.get();
    memcpy(&header.magic, buf, sizeof(uint32_t));
    buf += sizeof(uint32_t);
    memcpy(&header.version, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.distance_ratio, buf, sizeof(uint8_t));
    buf += sizeof(uint8_t);
    memcpy(header.brand.data(), buf, 3*sizeof(uint8_t));
    buf += 3*sizeof(uint8_t);
    memcpy(header.vommercial.data(), buf, 12*sizeof(uint8_t));
    buf += 12*sizeof(uint8_t); 
    memcpy(&header.internal, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.hardware, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.software, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.timestamp, buf, sizeof(uint32_t));
    buf += sizeof(uint32_t);
    memcpy(&header.scan_frequency, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.safe_status, buf, sizeof(uint8_t));
    buf += sizeof(uint8_t);
    memcpy(&header.error_status, buf, sizeof(uint8_t));
    buf += sizeof(uint8_t);
    memcpy(&header.status_flags, buf, sizeof(uint32_t));
    buf += sizeof(uint32_t);
    return std::tuple<uint16_t, uint32_t, uint16_t>(40,0,0);
}
