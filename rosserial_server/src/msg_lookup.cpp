/**
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2019, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 */

#include <regex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>

#include <boost/process.hpp>

#include "rosserial_server/msg_lookup.h"

namespace rosserial_server
{

// See https://wiki.ros.org/Names
static const std::regex valid_name{"^[A-Za-z][A-Za-z0-9_]*$"};

static std::unordered_map<std::string, MsgInfo> lookup_cache;

MsgInfo lookupMessage(const std::string& message_type, const std::string& submodule)
{
  if (submodule != "msg" && submodule != "srv")
  {
    throw std::invalid_argument("Only \"msg\" and \"srv\" are supported as submodule!");
  }

  // A cache miss will construct a MsgInfo object and return a reference to it.
  MsgInfo& cache_entry = lookup_cache[message_type];
  if (!cache_entry.md5sum.empty())
  {
    return cache_entry;
  }

  size_t slash_pos = message_type.find('/');
  if (slash_pos == std::string::npos)
  {
    throw std::runtime_error("Passed message type string does not include a slash character.");
  }
  std::string module_name = message_type.substr(0, slash_pos);
  std::string class_name = message_type.substr(slash_pos + 1, std::string::npos);

  // Sanitize input, we don't want anyone to mess with our python interpreter.
  if (!regex_match(module_name, valid_name) || !regex_match(class_name, valid_name))
  {
    throw std::runtime_error("Message type name is invalid!");
  }

  // Spawn single subprocess that prints both MD5 sum and full text.
  std::string py_script = "from " + module_name + "." + submodule + " import " + class_name + " as MsgType;";
  py_script += " print(MsgType._md5sum); print(MsgType._full_text);";

  boost::process::ipstream output;
  boost::process::child proc{PYTHON_EXECUTABLE, "-c", py_script, boost::process::std_out > output};

  MsgInfo msginfo;
  if (!std::getline(output, msginfo.md5sum))
  {
    throw std::runtime_error("Could not fetch MD5 checksum.");
  }
  std::string line;
  while (std::getline(output, line))
  {
    msginfo.full_text.append(line);
  }

  // TODO: Implement some timeout mechanism
  std::error_code error;
  proc.wait(error);
  if (error)
  {
    throw std::runtime_error("Message type lookup returned with error: " + error.message());
  }

  // See https://github.com/ros/ros_comm/issues/344 and https://github.com/ros/gencpp/pull/14
  // At this point, we can assume that the message definition is in fact empty and insert an empty line to suppress
  // further warnings.
  if (msginfo.full_text.empty())
  {
    msginfo.full_text = "\n";
  }

  cache_entry = msginfo;
  return msginfo;
}

}  // namespace rosserial_server
