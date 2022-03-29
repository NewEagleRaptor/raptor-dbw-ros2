//  Copyright (c) 2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef CAN_DBC_PARSER__DBCBUILDER_HPP_
#define CAN_DBC_PARSER__DBCBUILDER_HPP_

#include <can_dbc_parser/Dbc.hpp>
#include <can_dbc_parser/LineParser.hpp>

#include <sstream>
#include <string>
#include <fstream>

namespace NewEagle
{
struct DbcSignalValueType
{
  uint32_t Id;
  std::string SignalName;
  NewEagle::DataType Type;
};

struct DbcAttribute
{
  std::string AttributeName;
  uint32_t Id;
  std::string ObjectType;
  std::string SignalName;
  std::string Value;
};

class DbcBuilder
{
public:
  DbcBuilder();

  NewEagle::Dbc NewDbc(const std::string & dbcFile);

private:
  std::string MessageToken;
  std::string SignalToken;
  std::string CommentToken;
  std::string EnumValueToken;
  std::string AttributeToken;
  std::string SignalValueTypeToken;
  std::string EndOfInitToken;
  bool isInitPassed;
};


__attribute__((unused)) static NewEagle::DbcSignalValueType ReadSignalValueType(NewEagle::LineParser parser)
{
  NewEagle::DbcSignalValueType signalValueType;

  signalValueType.Id = parser.ReadUInt("id");
  signalValueType.SignalName = parser.ReadCIdentifier();
  parser.SeekSeparator(':');
  signalValueType.Type = parser.ReadUInt("DataType") == 1 ? NewEagle::FLOAT : NewEagle::DOUBLE;

  return signalValueType;
}

__attribute__((unused)) static NewEagle::DbcAttribute ReadAttribute(NewEagle::LineParser parser)
{
  NewEagle::DbcAttribute attribute;

  try {
    attribute.AttributeName = parser.ReadQuotedString();
    if (attribute.AttributeName == "GenSigStartValue") {
      attribute.ObjectType = parser.ReadCIdentifier();
      attribute.Id = parser.ReadUInt("id");
      if (attribute.ObjectType == "SG_") {
        attribute.SignalName = parser.ReadCIdentifier();

        std::ostringstream sstream;
        sstream << parser.ReadDouble();
        attribute.Value = sstream.str();
      }
    }
  } catch (std::exception & ex) {
    throw;
  }

  return attribute;
}

 __attribute__((unused)) static NewEagle::DbcMessageComment ReadMessageComment(NewEagle::LineParser parser)
{
  NewEagle::DbcMessageComment comment;
  comment.Id = parser.ReadUInt("id");
  comment.Comment = parser.ReadQuotedString();

  return comment;
}

__attribute__((unused)) static NewEagle::DbcSignalComment ReadSignalComment(NewEagle::LineParser parser)
{
  NewEagle::DbcSignalComment comment;
  comment.Id = parser.ReadUInt("id");
  comment.SignalName = parser.ReadCIdentifier();
  comment.Comment = parser.ReadQuotedString();

  return comment;
}

__attribute__((unused)) static  NewEagle::DbcMessage ReadMessage(NewEagle::LineParser parser)
{
  uint32_t canId = parser.ReadUInt("message id");
  IdType idType = ((canId & 0x80000000u) > 0) ? NewEagle::EXT : NewEagle::STD;
  std::string name(parser.ReadCIdentifier("message name"));
  parser.SeekSeparator(':');
  uint8_t dlc = parser.ReadUInt("size");
  std::string sendingNode(parser.ReadCIdentifier("transmitter"));
  uint32_t id = (uint32_t)(canId & 0x3FFFFFFFu);

  NewEagle::DbcMessage msg(dlc, id, idType, name, canId);
  return msg;
}

__attribute__((unused)) static NewEagle::DbcSignal ReadSignal(NewEagle::LineParser parser)
{
  std::string name = parser.ReadCIdentifier();
  char mux = parser.ReadNextChar("mux");
  NewEagle::MultiplexerMode multiplexMode = NewEagle::NONE;
  int32_t muxSwitch = 0;

  switch (mux) {
    case ':':
      multiplexMode = NewEagle::NONE;
      break;
    case 'M':
      multiplexMode = NewEagle::MUX_SWITCH;
      parser.SeekSeparator(':');
      break;
    case 'm':
      multiplexMode = NewEagle::MUX_SIGNAL;
      muxSwitch = parser.ReadInt();
      parser.SeekSeparator(':');
      break;
    default:
      throw std::runtime_error("Synxax Error: Expected \':\' " + parser.GetPosition());
  }

  int32_t startBit = parser.ReadUInt("start bit");

  parser.SeekSeparator('|');

  uint8_t length = (uint8_t)parser.ReadUInt("size");
  parser.SeekSeparator('@');

  char byteOrder = parser.ReadNextChar("byte order");

  NewEagle::ByteOrder endianness;

  switch (byteOrder) {
    case '1':
      endianness = NewEagle::LITTLE_END;
      break;
    case '0':
      endianness = NewEagle::BIG_END;
      break;
    default:
      std::string error_msg("Syntax Error: Byte Order - Expected 0 or 1, got ");
      error_msg.push_back(byteOrder);
      throw std::runtime_error(error_msg);
  }

  char valType = parser.ReadNextChar("value type");
  NewEagle::SignType sign;

  switch (valType) {
    case '+':
      sign = NewEagle::UNSIGNED;
      break;
    case '-':
      sign = NewEagle::SIGNED;
      break;
    default:
      std::string error_msg("Syntax Error: Value Type - Expected + or -, got ");
      error_msg.push_back(valType);
      throw std::runtime_error(error_msg);
  }

  // Set the default data type: INT.
  //  Might be changed what parsing SIG_VALTYPE_
  NewEagle::DataType type = NewEagle::INT;


  parser.SeekSeparator('(');
  double gain = parser.ReadDouble("factor");
  parser.SeekSeparator(',');
  double offset = parser.ReadDouble("offset");
  parser.SeekSeparator(')');

  parser.SeekSeparator('[');
  // min value
  parser.ReadDouble("minimum");
  parser.SeekSeparator('|');
  // max value
  parser.ReadDouble("maximum");
  parser.SeekSeparator(']');

  // Need to include Min, Max, DataType, MuxSwitch, Unit, Receiver
  // Find a way to include the DLC...
  NewEagle::DbcSignal * signal;

  if (NewEagle::MUX_SIGNAL == multiplexMode) {
    signal = new NewEagle::DbcSignal(
      8, gain, offset, startBit, endianness,
      length, sign, name,
      multiplexMode, muxSwitch);
  } else {
    signal = new NewEagle::DbcSignal(
      8, gain, offset, startBit, endianness,
      length, sign, name,
      multiplexMode);
  }

  signal->SetDataType(type);
  return NewEagle::DbcSignal(*signal);
}
}  // namespace NewEagle

#endif  // CAN_DBC_PARSER__DBCBUILDER_HPP_
