// CAN adapters on a PC: PEAK PCAN (Windows, Linux, macOS), slcan serial
// adapters (Windows, Linux, macOS) and SocketCAN (Linux).
//
//   auto t = openrobot::open_transport("pcan",  "PCAN_USBBUS1");
//   auto t = openrobot::open_transport("slcan", "COM5");          // or /dev/ttyACM0
//   auto t = openrobot::open_transport("socketcan", "can0");      // Linux
//
// PCAN needs PEAK's PCAN-Basic library at run time (PCANBasic.dll on
// Windows, libpcanbasic.so on Linux, libPCBUSB.dylib from MacCAN on macOS);
// it is loaded on demand, so nothing extra is needed to build.
//
// SPDX-License-Identifier: MIT
#ifndef OPENROBOT_MOTOR_PC_TRANSPORT_HPP
#define OPENROBOT_MOTOR_PC_TRANSPORT_HPP

#include <chrono>
#include <memory>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <thread>

#include "bus.hpp"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <dlfcn.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#endif

namespace openrobot {

class PcTransport : public Transport {
public:
    uint32_t now_ms() override {
        using namespace std::chrono;
        return (uint32_t)duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    }
};

// ---------------------------------------------------------------- PCAN --

class PcanTransport : public PcTransport {
public:
    PcanTransport(const std::string &channel, uint32_t bitrate) {
        ch_ = channel_handle(channel);
        uint16_t btr = baud_code(bitrate);
        load();
        uint32_t st = init_(ch_, btr, 0, 0, 0);
        if (st != 0) throw std::runtime_error("PCAN: cannot open " + channel + " (status 0x" + hex(st) +
                                              "; is another program using it?)");
#ifdef _WIN32
        event_ = CreateEvent(NULL, FALSE, FALSE, NULL);
        HANDLE h = event_;
        if (setval_(ch_, 0x03 /* PCAN_RECEIVE_EVENT */, &h, sizeof(h)) != 0) {
            CloseHandle(event_);
            event_ = NULL;
        }
#endif
    }
    ~PcanTransport() override {
        if (uninit_) uninit_(ch_);
#ifdef _WIN32
        if (event_) CloseHandle(event_);
        if (lib_) FreeLibrary((HMODULE)lib_);
#else
        if (lib_) dlclose(lib_);
#endif
    }

    bool send(uint16_t id, const uint8_t data[8]) override {
        Msg m;
        memset(&m, 0, sizeof(m));
        m.id = id;
        m.type = 0;                                  // standard frame
        m.len = 8;
        memcpy(m.data, data, 8);
        return write_(ch_, &m) == 0;
    }

    bool recv(uint16_t &id, uint8_t data[8], uint32_t timeout_ms) override {
        const uint32_t start = now_ms();
        for (;;) {
            Msg m;
            Stamp ts;
            uint32_t st = read_(ch_, &m, &ts);
            if (st == 0) {
                if (m.type != 0 || m.len != 8) continue;      // extended, status, short
                id = (uint16_t)m.id;
                memcpy(data, m.data, 8);
                return true;
            }
            uint32_t spent = now_ms() - start;
            if (spent >= timeout_ms) return false;
#ifdef _WIN32
            if (event_) {
                WaitForSingleObject(event_, timeout_ms - spent);
                continue;
            }
#endif
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

private:
    struct Msg {                                     // TPCANMsg
        uint32_t id;
        uint8_t type;
        uint8_t len;
        uint8_t data[8];
    };
    struct Stamp {                                   // TPCANTimestamp
        uint32_t millis;
        uint16_t overflow, micros;
    };
#ifdef _WIN32
#define OR_PCAN_API WINAPI
#else
#define OR_PCAN_API
#endif
    typedef uint32_t(OR_PCAN_API *InitFn)(uint16_t, uint16_t, uint8_t, uint32_t, uint16_t);
    typedef uint32_t(OR_PCAN_API *UninitFn)(uint16_t);
    typedef uint32_t(OR_PCAN_API *ReadFn)(uint16_t, Msg *, Stamp *);
    typedef uint32_t(OR_PCAN_API *WriteFn)(uint16_t, Msg *);
    typedef uint32_t(OR_PCAN_API *SetValueFn)(uint16_t, uint8_t, void *, uint32_t);

    static std::string hex(uint32_t v) {
        char b[16];
        snprintf(b, sizeof(b), "%X", (unsigned)v);
        return b;
    }

    static uint16_t channel_handle(const std::string &c) {
        // PCAN_USBBUS1..8 = 0x51..0x58, 9..16 = 0x509..0x510
        const char *p = "PCAN_USBBUS";
        if (c.compare(0, strlen(p), p) != 0) throw std::runtime_error("PCAN: channel must be PCAN_USBBUSn");
        int n = atoi(c.c_str() + strlen(p));
        if (n >= 1 && n <= 8) return (uint16_t)(0x50 + n);
        if (n >= 9 && n <= 16) return (uint16_t)(0x500 + n);
        throw std::runtime_error("PCAN: bad channel " + c);
    }

    static uint16_t baud_code(uint32_t b) {
        switch (b) {
        case 1000000: return 0x0014;
        case 800000: return 0x0016;
        case 500000: return 0x001C;
        case 250000: return 0x011C;
        case 125000: return 0x031C;
        default: throw std::runtime_error("PCAN: unsupported bit rate");
        }
    }

    void load() {
#ifdef _WIN32
        lib_ = (void *)LoadLibraryA("PCANBasic.dll");
        if (!lib_) throw std::runtime_error("PCAN: PCANBasic.dll not found (install the PEAK driver)");
#define OR_SYM(name) GetProcAddress((HMODULE)lib_, name)
#else
#ifdef __APPLE__
        lib_ = dlopen("libPCBUSB.dylib", RTLD_NOW);
        if (!lib_) throw std::runtime_error("PCAN: libPCBUSB.dylib not found (install PCBUSB from MacCAN)");
#else
        lib_ = dlopen("libpcanbasic.so", RTLD_NOW);
        if (!lib_) throw std::runtime_error("PCAN: libpcanbasic.so not found (or use socketcan with can0)");
#endif
#define OR_SYM(name) dlsym(lib_, name)
#endif
        init_ = (InitFn)OR_SYM("CAN_Initialize");
        uninit_ = (UninitFn)OR_SYM("CAN_Uninitialize");
        read_ = (ReadFn)OR_SYM("CAN_Read");
        write_ = (WriteFn)OR_SYM("CAN_Write");
        setval_ = (SetValueFn)OR_SYM("CAN_SetValue");
#undef OR_SYM
        if (!init_ || !uninit_ || !read_ || !write_ || !setval_) throw std::runtime_error("PCAN: library is incomplete");
    }

    uint16_t ch_ = 0;
    void *lib_ = nullptr;
    InitFn init_ = nullptr;
    UninitFn uninit_ = nullptr;
    ReadFn read_ = nullptr;
    WriteFn write_ = nullptr;
    SetValueFn setval_ = nullptr;
#ifdef _WIN32
    HANDLE event_ = NULL;
#endif
};

// ---------------------------------------------------------------- slcan --

// Serial-line CAN (CANable and compatible, Lawicel protocol).
class SlcanTransport : public PcTransport {
public:
    SlcanTransport(const std::string &port, uint32_t bitrate) {
        char s = 0;
        switch (bitrate) {
        case 1000000: s = '8'; break;
        case 800000: s = '7'; break;
        case 500000: s = '6'; break;
        case 250000: s = '5'; break;
        case 125000: s = '4'; break;
        default: throw std::runtime_error("slcan: unsupported bit rate");
        }
        open_port(port);
        write_str("C\r");                               // close in case it was open
        flush_in(50);
        char cmd[] = {'S', s, '\r', 0};
        write_str(cmd);
        write_str("O\r");
        flush_in(50);
    }
    ~SlcanTransport() override {
        if (is_open()) write_str("C\r");
        close_port();
    }

    bool send(uint16_t id, const uint8_t data[8]) override {
        char line[32];
        int n = snprintf(line, sizeof(line), "t%03X8", (unsigned)(id & 0x7FF));
        for (int i = 0; i < 8; ++i) n += snprintf(line + n, sizeof(line) - n, "%02X", data[i]);
        line[n++] = '\r';
        line[n] = 0;
        return write_str(line);
    }

    bool recv(uint16_t &id, uint8_t data[8], uint32_t timeout_ms) override {
        const uint32_t start = now_ms();
        for (;;) {
            // Parse complete lines already buffered.
            for (;;) {
                size_t eol = buf_.find_first_of("\r\a");
                if (eol == std::string::npos) break;
                std::string line = buf_.substr(0, eol);
                buf_.erase(0, eol + 1);
                if (parse(line, id, data)) return true;
            }
            uint32_t spent = now_ms() - start;
            if (spent > timeout_ms) return false;
            char tmp[256];
            int n = read_some(tmp, sizeof(tmp), timeout_ms - spent);
            if (n > 0) buf_.append(tmp, n);
            else if (now_ms() - start >= timeout_ms) return false;
        }
    }

private:
    static int hexv(char c) {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    }

    static bool parse(const std::string &l, uint16_t &id, uint8_t data[8]) {
        // "tIIIL" + 2*L hex; only standard frames with 8 bytes are replies.
        if (l.size() != 5 + 16 || l[0] != 't' || l[4] != '8') return false;
        int a = hexv(l[1]), b = hexv(l[2]), c = hexv(l[3]);
        if (a < 0 || b < 0 || c < 0) return false;
        id = (uint16_t)((a << 8) | (b << 4) | c);
        for (int i = 0; i < 8; ++i) {
            int h = hexv(l[5 + 2 * i]), lo = hexv(l[6 + 2 * i]);
            if (h < 0 || lo < 0) return false;
            data[i] = (uint8_t)((h << 4) | lo);
        }
        return true;
    }

    bool write_str(const char *s) { return write_bytes(s, strlen(s)); }

    void flush_in(uint32_t ms) {
        char tmp[256];
        uint32_t start = now_ms();
        while (now_ms() - start < ms) read_some(tmp, sizeof(tmp), 5);
        buf_.clear();
    }

    std::string buf_;

#ifdef _WIN32
    HANDLE h_ = INVALID_HANDLE_VALUE;
    bool is_open() const { return h_ != INVALID_HANDLE_VALUE; }
    void open_port(const std::string &port) {
        std::string path = "\\\\.\\" + port;
        h_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
        if (h_ == INVALID_HANDLE_VALUE) throw std::runtime_error("slcan: cannot open " + port);
        DCB dcb;
        memset(&dcb, 0, sizeof(dcb));
        dcb.DCBlength = sizeof(dcb);
        GetCommState(h_, &dcb);
        dcb.BaudRate = 115200;                          // ignored by USB CDC adapters
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        dcb.fRtsControl = RTS_CONTROL_ENABLE;
        SetCommState(h_, &dcb);
    }
    void close_port() {
        if (is_open()) CloseHandle(h_);
        h_ = INVALID_HANDLE_VALUE;
    }
    bool write_bytes(const char *p, size_t n) {
        DWORD w = 0;
        return WriteFile(h_, p, (DWORD)n, &w, NULL) && w == n;
    }
    int read_some(char *p, size_t n, uint32_t timeout_ms) {
        // Return as soon as any byte is there, or after timeout_ms.
        COMMTIMEOUTS to;
        memset(&to, 0, sizeof(to));
        to.ReadIntervalTimeout = MAXDWORD;
        to.ReadTotalTimeoutMultiplier = MAXDWORD;
        to.ReadTotalTimeoutConstant = timeout_ms ? timeout_ms : 1;
        SetCommTimeouts(h_, &to);
        DWORD r = 0;
        if (!ReadFile(h_, p, (DWORD)n, &r, NULL)) return -1;
        return (int)r;
    }
#else
    int fd_ = -1;
    bool is_open() const { return fd_ >= 0; }
    void open_port(const std::string &port) {
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) throw std::runtime_error("slcan: cannot open " + port);
        termios tio;
        tcgetattr(fd_, &tio);
        cfmakeraw(&tio);
        cfsetispeed(&tio, B115200);
        cfsetospeed(&tio, B115200);
        tcsetattr(fd_, TCSANOW, &tio);
    }
    void close_port() {
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
    }
    bool write_bytes(const char *p, size_t n) {
        while (n) {
            ssize_t w = ::write(fd_, p, n);
            if (w < 0) {
                pollfd pf = {fd_, POLLOUT, 0};
                if (poll(&pf, 1, 100) <= 0) return false;
                continue;
            }
            p += w;
            n -= (size_t)w;
        }
        return true;
    }
    int read_some(char *p, size_t n, uint32_t timeout_ms) {
        pollfd pf = {fd_, POLLIN, 0};
        if (poll(&pf, 1, (int)timeout_ms) <= 0) return 0;
        ssize_t r = ::read(fd_, p, n);
        return r < 0 ? 0 : (int)r;
    }
#endif
};

// ---------------------------------------------------------------- SocketCAN --

#ifdef __linux__
// Bit rate is set outside: sudo ip link set can0 up type can bitrate 1000000
class SocketcanTransport : public PcTransport {
public:
    explicit SocketcanTransport(const std::string &ifname) {
        fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd_ < 0) throw std::runtime_error("socketcan: cannot create socket");
        ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
        if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
            ::close(fd_);
            throw std::runtime_error("socketcan: no interface " + ifname + " (is it up?)");
        }
        sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(fd_, (sockaddr *)&addr, sizeof(addr)) < 0) {
            ::close(fd_);
            throw std::runtime_error("socketcan: cannot bind " + ifname);
        }
    }
    ~SocketcanTransport() override { ::close(fd_); }

    bool send(uint16_t id, const uint8_t data[8]) override {
        can_frame f;
        memset(&f, 0, sizeof(f));
        f.can_id = id & CAN_SFF_MASK;
        f.can_dlc = 8;
        memcpy(f.data, data, 8);
        return ::write(fd_, &f, sizeof(f)) == (ssize_t)sizeof(f);
    }

    bool recv(uint16_t &id, uint8_t data[8], uint32_t timeout_ms) override {
        const uint32_t start = now_ms();
        for (;;) {
            uint32_t spent = now_ms() - start;
            int left = spent >= timeout_ms ? 0 : (int)(timeout_ms - spent);
            pollfd pf = {fd_, POLLIN, 0};
            if (poll(&pf, 1, left) <= 0) return false;
            can_frame f;
            if (::read(fd_, &f, sizeof(f)) != (ssize_t)sizeof(f)) return false;
            if ((f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) || f.can_dlc != 8) continue;
            id = (uint16_t)(f.can_id & CAN_SFF_MASK);
            memcpy(data, f.data, 8);
            return true;
        }
    }

private:
    int fd_ = -1;
};
#endif

// ---------------------------------------------------------------- factory --

// interface: "pcan", "slcan" or "socketcan" (Linux). Throws on failure.
inline std::unique_ptr<Transport> open_transport(const std::string &interface, const std::string &channel,
                                                 uint32_t bitrate = DEFAULT_BITRATE) {
    if (interface == "pcan") return std::unique_ptr<Transport>(new PcanTransport(channel, bitrate));
    if (interface == "slcan") return std::unique_ptr<Transport>(new SlcanTransport(channel, bitrate));
#ifdef __linux__
    if (interface == "socketcan") return std::unique_ptr<Transport>(new SocketcanTransport(channel));
#endif
    throw std::runtime_error("unknown interface " + interface);
}

}  // namespace openrobot

#endif  // OPENROBOT_MOTOR_PC_TRANSPORT_HPP
