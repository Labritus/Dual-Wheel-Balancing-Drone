#include "DataScope_DP.hpp"
#include "USART3.hpp"

// Define static member variable
uint8_t DataScope_DP::outputBuffer[42];

// C-compatible interface global variable
uint8_t DataScope_OutPut_Buffer[42];

/**
 * Convert float to byte array
 * @param target Pointer to target float
 * @param buf Target buffer
 * @param beg Start writing at index `beg` of the buffer
 */
void DataScope_DP::float2Byte(float *target, uint8_t *buf, uint8_t beg)
{
    uint8_t *point;
    point = (uint8_t*)target;  // Get float address
    buf[beg] = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}

/**
 * Write channel data into the send buffer
 * @param data Channel data
 * @param channel Select channel (1–10)
 */
void DataScope_DP::setChannelData(float data, uint8_t channel)
{
    if ((channel > 10) || (channel == 0)) 
        return;
    else
    {
        switch (channel)
        {
            case 1:  float2Byte(&data, outputBuffer, 1); break;
            case 2:  float2Byte(&data, outputBuffer, 5); break;
            case 3:  float2Byte(&data, outputBuffer, 9); break;
            case 4:  float2Byte(&data, outputBuffer, 13); break;
            case 5:  float2Byte(&data, outputBuffer, 17); break;
            case 6:  float2Byte(&data, outputBuffer, 21); break;
            case 7:  float2Byte(&data, outputBuffer, 25); break;
            case 8:  float2Byte(&data, outputBuffer, 29); break;
            case 9:  float2Byte(&data, outputBuffer, 33); break;
            case 10: float2Byte(&data, outputBuffer, 37); break;
        }
    }
}

/**
 * Generate data frame
 * @param channelNumber Number of channels to send
 * @return Number of bytes in the send buffer
 */
uint8_t DataScope_DP::generateDataFrame(uint8_t channelNumber)
{
    if (channelNumber > 10) 
        return 0;
    else
    {
        outputBuffer[0] = '$';  // Frame header
        
        // Determine frame length based on channel number
        switch(channelNumber)
        {
            case 1:  outputBuffer[5] = 5; return 6;
            case 2:  outputBuffer[9] = 9; return 10;
            case 3:  outputBuffer[13] = 13; return 14;
            case 4:  outputBuffer[17] = 17; return 18;
            case 5:  outputBuffer[21] = 21; return 22;
            case 6:  outputBuffer[25] = 25; return 26;
            case 7:  outputBuffer[29] = 29; return 30;
            case 8:  outputBuffer[33] = 33; return 34;
            case 9:  outputBuffer[37] = 37; return 38;
            case 10: outputBuffer[41] = 41; return 42;
        }
    }
    return 0;
}

/**
 * Get the output buffer
 * @return Pointer to output buffer
 */
uint8_t* DataScope_DP::getOutputBuffer()
{
    return outputBuffer;
}
