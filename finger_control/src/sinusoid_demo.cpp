#include <string>
#include <iostream>
#include <cstdio>

#include <unistd.h>
#include "serial/serial.h"

#include "fingerlib/transformer.hpp"
#include "fingerlib/sinusoid.hpp"

uint8_t crc8(const uint8_t* data, size_t length) {
    uint8_t crc = 0x00;  // initial value
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;  // 0x07 is the CRC8 polynomial
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

std::string port = "/dev/ttyACM0";  // adjust to your port
uint32_t baud = 115200;             // adjust to your baud rate

// Radius Matrix
const double ra = 0.0025; // splay
const double rb = 0.0025; // mcp
const double rc = 0.0025; // pip/dip

const arma::mat Ra = {{ra, 0, 0}, // splay
                        {0, rb, 0}, // mcp
                        {0, 0, rc}}; // pip/dip


// Structure Matrix
const double r11 = ra*3.5;
const double r1 = 8 * 0.001;
//const double r2 = 8 * 0.001;
const double r3 = 4.5 * 0.001;
//const double r4 = 4.5 * 0.001;
const double r5 = 8 * 0.001;
//const double r6 = 8 * 0.001;
const double r7 = 4.5 * 0.001;
//const double r8 = 4.5 * 0.001;
const double r9 = 9 * 0.001;
//const double r10 = 9 * 0.001;

const arma::mat St = {{r11, -r3, -r1}, //splay joint 
                        {0, r7, r5}, //mcp joint
                        {0, 0, r9}}; // pip/dip joint

// screw axes
const std::vector<arma::vec6> slist = {
    arma::vec6({1,2,3,4,5,6}),
    arma::vec6({6,5,4,3,2,1}),
    arma::vec6({6,5,4,3,2,1}),
    arma::vec6({6,5,4,3,2,1})
};

// fake M rn
const arma::mat44 M = {{1, 0, 0, 0.05},
                        {0, 1, 0, 0},
                        {0, 0, 1, 0.1},
                        {0, 0, 0, 1}};

// 4 bar lengths
const std::vector<double> four_bar_lengths = {
    8.83765 * 0.001,
    40.6 * 0.001,
    8.91536 * 0.001,
    37 * 0.001
};
int main() {
    auto transforms = Transformer{Ra, St, slist, M, four_bar_lengths};
    auto generator = Sinusoid{transforms, 100};    
    auto q_motor_list = generator.generate_sinusoid(1, 0.2, 1.0, 0.8);

    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
    if (my_serial.isOpen()) {
        std::cout << "Serial port opened successfully" << std::endl;
    } else {
        std::cerr << "Failed to open serial port" << std::endl;
        return 1;
    }

    // Build data lines first so we can CRC them the same way Teensy does
    std::vector<std::string> data_lines;
    for (auto& q_motor : q_motor_list) {
        data_lines.push_back(
            std::to_string(q_motor(0)) + " " +
            std::to_string(q_motor(1)) + " " +
            std::to_string(q_motor(2))   // no \n yet — match Teensy's strlen behavior
        );
    }

    // CRC over data lines only (indices 1..N-1 on Teensy = all data lines here)
    // Mirror crc8_message: iterate lines, iterate bytes via CRC8 table
    auto build_table = [](uint8_t tbl[256]) {
        for (int i = 0; i < 256; i++) {
            uint8_t crc = i;
            for (int b = 0; b < 8; b++)
                crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
            tbl[i] = crc;
        }
    };
    uint8_t crc_table[256];
    build_table(crc_table);

    uint8_t checksum = 0x00;
    for (auto& line : data_lines) {
        for (unsigned char c : line)
            checksum = crc_table[checksum ^ c];
    }

    // Assemble payload
    std::string payload = "s " + std::to_string(q_motor_list.size()) + " 1\n";
    for (auto& line : data_lines)
        payload += line + "\n";
    payload += std::to_string(checksum) + "\n";
    payload += "end\n";   // "end" on its own line

    std::cout << "Sending " << data_lines.size() << " motor position lines." << std::endl;
    my_serial.write(payload);

    // Teensy responds: "<type> <repeat> <lineCount> <error>\n"
    // lineCount = messageLineCount = header + data_lines + footer = N+2
    std::string response = my_serial.readline();
    std::cout << "Teensy response: " << response << std::endl;

    // Parse and validate
    char resp_type; int resp_repeat, resp_linecount, resp_recieved_chksm, resp_computed_chksm, resp_error;
    if (sscanf(response.c_str(), "%c %d %d %d %d %d",
               &resp_type, &resp_repeat, &resp_linecount, &resp_recieved_chksm, &resp_computed_chksm, &resp_error) == 6) {
        if (resp_error != 0) {
            std::cerr << "Teensy reported error code: " << resp_error << std::endl;
        } else {
            std::cout << "OK — type=" << resp_type
                      << " repeat=" << resp_repeat
                      << " lines=" << resp_linecount
                      << " recieved_chksm=" << resp_recieved_chksm
                      << " computed_chksm=" << resp_computed_chksm
                      << " error=" << resp_error << std::endl;
        }
    } else {
        std::cerr << "Could not parse Teensy response." << std::endl;
    }

    my_serial.close();
    return 0;
}