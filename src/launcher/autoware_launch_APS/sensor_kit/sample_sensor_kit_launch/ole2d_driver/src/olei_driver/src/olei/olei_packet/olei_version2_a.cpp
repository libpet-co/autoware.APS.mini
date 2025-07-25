#include "olei/olei_packet/olei_version2_a.h"
#include "olei/olei_packet_reader.h"
#include<iostream>
void Olei2Packet_A::read_data(uint8_t *buf, size_t num)
{
    Data* data = reinterpret_cast<Data*>(buf);
    distance.resize(num);
    angle.resize(num);
    intensity.resize(num);
    for(int i = 0; i < num; i++)
    {
        angle[i] = data[i].angle;     
        distance[i] = data[i].distance;
        intensity[i] = data[i].intensity;
    }
}

void Olei2Packet_A::read_with(OleiPacketReader &reader)
{
    reader.read(*this);
}