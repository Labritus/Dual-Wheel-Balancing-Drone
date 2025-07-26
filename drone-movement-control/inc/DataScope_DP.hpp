#ifndef __DATASCOPE_DP_HPP
#define __DATASCOPE_DP_HPP

#include <stdint.h>
#include <cstddef>

class DataScope_DP {
private:
    static uint8_t outputBuffer[42];
    
public:
    // Generate data frame for transmission
    static uint8_t generateDataFrame(uint8_t channel_count);
    
    // Set data for specific channel
    static void setChannelData(float data, uint8_t channel);
    
    // Convert float to byte array
    static void float2Byte(float *target, uint8_t *buf, uint8_t beg);
    
    // Get output buffer
    static uint8_t* getOutputBuffer() { return outputBuffer; }
    
    // Get buffer size
    static constexpr size_t getBufferSize() { return sizeof(outputBuffer); }
};

// C-compatible interface
extern "C" {
    extern uint8_t DataScope_OutPut_Buffer[42];
}

#endif // __DATASCOPE_DP_HPP