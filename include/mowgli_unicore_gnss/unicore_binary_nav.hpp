// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <optional>

#include "mowgli_unicore_gnss/unicore_transport.hpp"
#include "mowgli_unicore_gnss/um982_parser.hpp"

namespace mowgli_unicore_gnss
{

class UnicoreBinaryNavParser
{
public:
  std::optional<ParsedSentence> parse(const UnicoreBinaryFrame& frame) const;

private:
  static std::optional<ParsedSentence> parse_bestnavb(const UnicoreBinaryFrame& frame);
  static std::optional<ParsedSentence> parse_pvtslnb(const UnicoreBinaryFrame& frame);
};

}  // namespace mowgli_unicore_gnss
