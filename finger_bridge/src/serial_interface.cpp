#include "serial_interface/serial_interface.hpp"


SerialInterface::SerialInterface()
: _port ("/dev/ttyACM0"),
  _baud (115200),
  _msg_status (MessageStatus::NO_STATUS),
  _serial (std::make_shared<serial::Serial>(_port, _baud, serial::Timeout::simpleTimeout(1000)))

{
  // check if it was successfully opened
  if (_serial->isOpen()) {
    std::cout << "Serial port opened successfully" << std::endl;
  } else {
    std::cerr << "Failed to open serial port" << std::endl;
  }

  // define lambda function to generate crc table
  auto build_table = [this]() {
      for (int i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (int b = 0; b < 8; b++) {
          crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
        _crc_table[i] = crc;
      }
    };

  // build the table
  build_table();
}

void SerialInterface::send_command(std::vector<arma::vec> q_motor_list)
{
  // Build data lines first so we can CRC them the same way Teensy does
  std::vector<std::string> data_lines;
  for (auto & q_motor : q_motor_list) {
    data_lines.push_back(
            std::to_string(q_motor(0)) + " " +
            std::to_string(q_motor(1)) + " " +
            std::to_string(q_motor(2))   // no \n yet — match Teensy's strlen behavior
    );
  }

  // compute checksum
  uint8_t checksum = 0x00;
  for (auto & line : data_lines) {
    for (unsigned char c : line) {
      checksum = _crc_table[checksum ^ c];
    }
  }

  // Assemble payload
  std::string payload = "s " + std::to_string(q_motor_list.size()) + " 1\n";
  for (auto & line : data_lines) {
    payload += line + "\n";
  }
  payload += std::to_string(checksum) + "\n";
  payload += "end\n";     // "end" on its own line

  std::cout << "Sending " << data_lines.size() << " motor position lines." << std::endl;
  _serial->write(payload);


}

void SerialInterface::parse_response()
{
  // get data
  std::string response = _serial->readline();
  std::cout << "Teensy response: " << response << std::endl;

  // Parse and validate
  int resp_success;
  float splay_pos, mcp_flex_pos, pip_flex_pos;
  if (sscanf(response.c_str(), "%d", &resp_success) == 1)
  {
    // check for success in message reception
    if (resp_success != 1){
        
    }
  } else if (sscanf(response.c_str(), "%f %f %f", &splay_pos, &mcp_flex_pos, &pip_flex_pos) == 3)
  {// save data somehow

  } else {
    std::cerr << "Could not parse Teensy response." << std::endl;
  }
}

MessageStatus SerialInterface::get_message_status() const
{
    return _msg_status;
}

