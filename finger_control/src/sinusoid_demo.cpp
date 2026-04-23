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

// 4 bar lengths
const std::vector<double> four_bar_lengths = {
    8.83765 * 0.001,
    40.6 * 0.001,
    8.91536 * 0.001,
    37 * 0.001
};

int main() {

    // create the sine wave
    auto transforms = Transformer{Ra, St, slist, four_bar_lengths};
    auto generator = Sinusoid{transforms, 100};    

    auto q_motor_list = generator.generate_sinusoid(0, 0.4, 1, 0.8);

    // Serial setup
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

    if (my_serial.isOpen()) {
        std::cout << "Serial port opened successfully" << std::endl;
    } else {
        std::cerr << "Failed to open serial port" << std::endl;
        return 1;
    }

    // setup basic commands
    std::string start_command = "s " + std::to_string(q_motor_list.size()) + " 1\n";
    std::cout << "Start command: " << start_command << std::endl;
    uint8_t checksum = 0x00;

    my_serial.write(start_command);
    for(auto q_motor: q_motor_list){
        std::string data = std::to_string(q_motor(0)) + " " + 
                            std::to_string(q_motor(1)) + " " + 
                            std::to_string(q_motor(2)) + "\n";

        std::cout << data;

        checksum ^= crc8(reinterpret_cast<const uint8_t*>(data.data()), data.size());
        my_serial.write(data);
    }
    std::string end_command = std::to_string(checksum) + " end\n";
    std::cout << "End command: " << end_command << std::endl;

    my_serial.write(end_command);

    std::string expected_stats = std::to_string(q_motor_list.size()) + " " + std::to_string(checksum) + "\n";

    std::string response = my_serial.readline();
    std::string stats = my_serial.readline();
    if(response != start_command || stats != expected_stats){
        std::cerr << "Unexpected response from microcontroller: " << std::endl;
        std::cerr << "\tReceived response: " << response << std::endl;
        std::cerr << "\tReceived stats: " << stats << std::endl;
    } else {
        std::cout << "Received expected response from microcontroller." << std::endl;
    }

    my_serial.close();
    return 0;
}