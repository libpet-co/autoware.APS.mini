#include "olei/olei_packet/olei_packet.h"
#include <iostream>
int OleiPacket::find_packet_start(uint8_t *buf, size_t buf_len)
{
  return 0;
}

bool OleiPacket::parse_buf(uint8_t *buf, size_t buf_len, size_t &remainder, size_t &packet_size)
{
   
    const size_t SIZE = get_size(); //获取包头数据大小
    boost::shared_array<uint8_t> buffer(new uint8_t[SIZE]);
    std::copy(buf, buf + SIZE, buffer.get());

    uint16_t h_size;
    uint32_t p_size;
    uint16_t num;
    std::tie(h_size, p_size, num) = read_header(buffer);
    if(p_size == 0&&num == 0)
    {
      p_size = buf_len;
      num = (buf_len - SIZE) / 8;
    }
    auto data_size = p_size - h_size;
    if (buf_len < p_size)
      return false;
    
    read_data(&buf[h_size], num);
    remainder = buf_len - p_size;
    packet_size = p_size;       
    return true;
}