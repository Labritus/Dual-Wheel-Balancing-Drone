#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <cstdint>
#include <string>

class I2CInterface {
public:
    I2CInterface();
    ~I2CInterface();
    
    // 初始化I2C总线
    bool initialize(const std::string& device, uint8_t address);
    
    // 关闭I2C总线
    void close();
    
    // 读取寄存器值
    bool readByte(uint8_t reg, uint8_t* data);
    bool readBytes(uint8_t reg, uint8_t* data, size_t length);
    
    // 写入寄存器值
    bool writeByte(uint8_t reg, uint8_t data);
    bool writeBytes(uint8_t reg, uint8_t* data, size_t length);
    
private:
    int fd_; // 文件描述符
    uint8_t address_; // 设备地址
    bool initialized_; // 初始化状态
};

#endif // I2C_INTERFACE_H 