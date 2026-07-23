#include "esp32_bridge/uart_linux.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <sys/ioctl.h>

UartLinux::UartLinux()
: fd_(-1)
{
}


UartLinux::~UartLinux()
{
    close();
}


bool UartLinux::open(
    const std::string& device,
    int baudrate)
{
    fd_ = ::open(
        device.c_str(),
        O_RDWR | O_NOCTTY | O_NONBLOCK
    );

    if (fd_ < 0)
    {
        return false;
    }


    termios tty{};

    if (tcgetattr(fd_, &tty) != 0)
    {
        close();
        return false;
    }


    speed_t speed;

    switch (baudrate)
    {
        case 9600:
            speed = B9600;
            break;

        case 19200:
            speed = B19200;
            break;

        case 57600:
            speed = B57600;
            break;

        case 115200:
            speed = B115200;
            break;

        default:
            close();
            return false;
    }


    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);


    // 8N1 configuration
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;


    // Enable receiver
    tty.c_cflag |= CREAD | CLOCAL;


    // Raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;


    // Non-blocking read
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;


    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        close();
        return false;
    }


    return true;
}


void UartLinux::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}


bool UartLinux::isOpen() const
{
    return fd_ >= 0;
}


bool UartLinux::write(
    const std::vector<uint8_t>& data)
{
    if (!isOpen())
        return false;


    ssize_t result = ::write(
        fd_,
        data.data(),
        data.size()
    );

    return result == static_cast<ssize_t>(data.size());
}


ssize_t UartLinux::read(
    uint8_t* buffer,
    size_t length)
{
    if (!isOpen())
        return -1;


    return ::read(
        fd_,
        buffer,
        length
    );
}


int UartLinux::available() const
{
    if (!isOpen())
        return 0;


    int bytes_available = 0;

    ioctl(
        fd_,
        FIONREAD,
        &bytes_available
    );

    return bytes_available;
}
