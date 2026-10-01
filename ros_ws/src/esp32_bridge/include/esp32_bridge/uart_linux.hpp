// Prevent multiple inclusion of this header file
#ifndef UART_LINUX_HPP
#define UART_LINUX_HPP

#include <cstdint>
#include <string>
#include <vector>

class UartLinux
{
public:
    UartLinux();
    ~UartLinux();

    bool open(const std::string& device, int baudrate);
    void close();

    bool isOpen() const;

    bool write(const std::vector<uint8_t>& data);

    ssize_t read(uint8_t* buffer, size_t length);

    int available() const;

private:
    int fd_;
};

#endif  // UART_LINUX_HPP
