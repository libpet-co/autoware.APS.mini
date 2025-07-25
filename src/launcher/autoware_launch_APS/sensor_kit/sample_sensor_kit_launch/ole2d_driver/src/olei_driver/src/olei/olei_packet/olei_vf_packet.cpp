#include "olei/olei_packet/olei_vf_packet.h"

size_t OleiVFPacket::get_size()
{
    return header_size;
}

std::tuple<uint16_t, uint32_t, uint16_t> OleiVFPacket::read_header(boost::shared_array<uint8_t> buffer)
{
    uint8_t* buf = buffer.get();
    memcpy(&header.magic, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.version, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.packet_size, buf, sizeof(uint32_t));
    buf += sizeof(uint32_t);
    memcpy(&header.header_size, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.distance_ratio, buf, sizeof(uint8_t));
    buf += sizeof(uint8_t);     
    memcpy(&header.types, buf, sizeof(uint8_t));
    buf += sizeof(uint8_t); 
    memcpy(&header.scan_number, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.packet_number, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.timestamp_dceimal, buf, sizeof(uint32_t));
    buf += sizeof(uint32_t);
    memcpy(&header.timestamp_integer, buf, sizeof(uint32_t));
    buf += sizeof(uint32_t);
    memcpy(&header.scan_frequency, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.num_points_scan, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.iutput_status, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.output_status, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.field_status, buf, sizeof(uint32_t));
    buf += sizeof(uint32_t); 
    memcpy(&header.start_index, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.end_index, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.first_index, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);
    memcpy(&header.num_points_packet, buf, sizeof(uint16_t));
    buf += sizeof(uint16_t);   
    memcpy(&header.status_flags, buf, sizeof(uint32_t));
    buf += sizeof(uint32_t); 
    return std::tuple<uint16_t, uint32_t, uint16_t>(header.header_size,header.packet_size,
                               header.num_points_packet);
}
