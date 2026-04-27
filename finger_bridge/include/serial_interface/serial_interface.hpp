#ifndef SERIAL_INTERFACE_HPP
#define SERIAL_INTERFACE_HPP

#include <string>
#include <iostream>
#include <cstdio>
#include <memory>
#include <unistd.h>
#include <vector>
#include <armadillo>
#include "serial/serial.h"

enum MessageStatus
{
  NO_STATUS,
  SUCCESS,
  FAILURE,
};

class SerialInterface
{
public:
  SerialInterface();

  void parse_response();

  void send_command(std::vector<arma::vec> q_motor_list);

  MessageStatus get_message_status() const;

private:
  /// \brief The name of the port to connect to the teensy
  std::string _port;
  /// \brief The baud rate of serial communicatino
  uint32_t _baud;
  /// \brief State variable containing state of sent messages
  MessageStatus _msg_status;
  /// \brief The serial interface variable
  std::shared_ptr<serial::Serial> _serial;
  /// \brief The crc8 lookup table
  uint8_t _crc_table[256];




};

#endif
