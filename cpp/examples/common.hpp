// Shared command line for the C++ examples:
//   example <interface> <channel> [motor_id]
//   example pcan PCAN_USBBUS1 1
//   example slcan COM5 1            (Windows)   /dev/ttyACM0 (Linux)
//   example socketcan can0 1        (Linux)
#pragma once

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

#include "openrobot_motor/pc_transport.hpp"

#ifdef _WIN32
#include <timeapi.h>   // timeBeginPeriod: 1 ms sleeps instead of 15.6 ms
#endif

static volatile std::sig_atomic_t g_stop = 0;

inline void on_sigint(int) { g_stop = 1; }

struct Args {
    std::string interface, channel;
    int id = 1;
};

inline Args parse_args(int argc, char **argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <pcan|slcan|socketcan> <channel> [motor_id]\n", argv[0]);
        std::exit(2);
    }
    Args a;
    a.interface = argv[1];
    a.channel = argv[2];
    if (argc > 3) a.id = std::atoi(argv[3]);
    std::signal(SIGINT, on_sigint);     // Ctrl+C ends the loop; the Motor destructor releases
#ifdef _WIN32
    timeBeginPeriod(1);
#endif
    return a;
}

inline void confirm_motion(const char *what) {
    std::printf("This example will %s.\n"
                "Make sure the shaft is free to turn and nothing is attached that can be hurt.\n"
                "Type yes to continue: ", what);
    std::string s;
    std::getline(std::cin, s);
    if (s != "yes") {
        std::printf("cancelled\n");
        std::exit(1);
    }
}
