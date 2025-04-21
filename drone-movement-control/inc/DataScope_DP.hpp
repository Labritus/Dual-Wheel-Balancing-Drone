#ifndef __DATA_SCOPE_DP_HPP
#define __DATA_SCOPE_DP_HPP

#include "System.hpp"

/**
 * @brief DataScope class
 * Used for sending collected data to an upper computer for visualization
 */
class DataScope_DP {
public:
    /**
     * @brief Convert a float to a byte array
     * @param target Pointer to the float value
     * @param buf Destination buffer
     * @param beg Starting index in the buffer to write to
     */
    static void float2Byte(float *target, uint8_t *buf, uint8_t beg);
    
    /**
     * @brief Set channel data into the send buffer
     * @param data Channel data
     * @param channel Channel number (1–10)
     */
    static void setChannelData(float data, uint8_t channel);
    
    /**
     * @brief Generate the data frame to be sent
     * @param channelNumber Number of channels to send
     * @return Number of bytes in the output buffer
     */
    static uint8_t generateDataFrame(uint8_t channelNumber);
    
    /**
     * @brief Get pointer to output buffer
     * @return Pointer to output buffer
     */
    static uint8_t* getOutputBuffer();

private:
    static uint8_t outputBuffer[42]; // Data transmission buffer
};

// C-compatible interface declarations (if needed)


#endif // __DATA_SCOPE_DP_HPP
