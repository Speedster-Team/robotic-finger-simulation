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

enum FeedbackStatus
{
  NEW_FEEDBACK,
  NOTHING_NEW,
};

class SerialInterface
{
public:
  SerialInterface();

  void parse_response();

  void send_command(std::vector<std::vector<float>> q_motor_list);

  MessageStatus get_message_status();

  FeedbackStatus get_feedback_status();

  std::vector<float> get_feedback();

private:
  /// \brief The name of the port to connect to the teensy
  std::string _port;
  /// \brief The baud rate of serial communicatino
  uint32_t _baud;
  /// \brief State variable containing state of sent messages
  MessageStatus _msg_status;
  /// \brief State variable containing state of sent messages
  FeedbackStatus _fdbk_status;
  /// \brief The serial interface variable
  std::shared_ptr<serial::Serial> _serial;
  /// \brief The crc8 lookup table
  uint8_t _crc_table[256];
  /// \brief The MCP splay motor position
  float _mcp_splay_motor_pos;
  /// \brief The MCP felx motor position
  float _mcp_flex_motor_pos;
  /// \brief The PIP flex motor position
  float _pip_flex_motor_pos;



};

#endif
