// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "mowgli_unicore_gnss/unicore_binary_nav.hpp"
#include "mowgli_unicore_gnss/unicore_transport.hpp"
#include "mowgli_unicore_gnss/um982_parser.hpp"
#include <gtest/gtest.h>

namespace mowgli_unicore_gnss
{
namespace
{

uint32_t crc32_unicore_binary(const std::string& bytes)
{
  uint32_t crc = 0U;
  for (const unsigned char byte : bytes)
  {
    crc ^= static_cast<uint32_t>(byte);
    for (int bit = 0; bit < 8; ++bit)
    {
      const bool lsb = (crc & 1U) != 0U;
      crc >>= 1U;
      if (lsb)
      {
        crc ^= 0xEDB88320U;
      }
    }
  }
  return crc;
}

uint32_t crc32_unicore_ascii(const std::string& text)
{
  uint32_t crc = 0U;
  for (const unsigned char byte : text)
  {
    crc ^= static_cast<uint32_t>(byte);
    for (int bit = 0; bit < 8; ++bit)
    {
      const bool lsb = (crc & 1U) != 0U;
      crc >>= 1U;
      if (lsb)
      {
        crc ^= 0xEDB88320U;
      }
    }
  }
  return crc;
}

std::string make_unicore_ascii(const std::string& payload)
{
  char crc_text[9];
  std::snprintf(crc_text, sizeof(crc_text), "%08x", crc32_unicore_ascii(payload));
  return "#" + payload + "*" + crc_text;
}

void append_le8(std::vector<uint8_t>& out, uint8_t value)
{
  out.push_back(value);
}

void append_le32(std::vector<uint8_t>& out, uint32_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 16U) & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 24U) & 0xFFU));
}

void append_float32(std::vector<uint8_t>& out, float value)
{
  uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  append_le32(out, bits);
}

void append_float64(std::vector<uint8_t>& out, double value)
{
  uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  for (std::size_t i = 0U; i < 8U; ++i)
  {
    out.push_back(static_cast<uint8_t>((bits >> (8U * i)) & 0xFFU));
  }
}

void append_char4(std::vector<uint8_t>& out, const char* text)
{
  for (int i = 0; i < 4; ++i)
  {
    const char ch = text[i];
    out.push_back(static_cast<uint8_t>(ch));
    if (ch == '\0')
    {
      for (int j = i + 1; j < 4; ++j)
      {
        out.push_back(0U);
      }
      return;
    }
  }
}

std::string make_binary_frame(uint16_t message_id,
                              const std::vector<uint8_t>& payload,
                              bool valid_crc = true)
{
  std::string frame;
  frame.push_back(static_cast<char>(0xAAU));
  frame.push_back(static_cast<char>(0x44U));
  frame.push_back(static_cast<char>(0xB5U));
  frame.push_back(static_cast<char>(17U));
  frame.push_back(static_cast<char>(message_id & 0xFFU));
  frame.push_back(static_cast<char>((message_id >> 8U) & 0xFFU));
  frame.push_back(static_cast<char>(payload.size() & 0xFFU));
  frame.push_back(static_cast<char>((payload.size() >> 8U) & 0xFFU));
  frame.push_back(static_cast<char>(1U));
  frame.push_back(static_cast<char>(2U));
  frame.push_back(static_cast<char>(0xF6U));
  frame.push_back(static_cast<char>(0x08U));
  frame.push_back(static_cast<char>(0x20U));
  frame.push_back(static_cast<char>(0x6DU));
  frame.push_back(static_cast<char>(0x27U));
  frame.push_back(static_cast<char>(0x1CU));
  frame.push_back(static_cast<char>(16U));
  frame.push_back(static_cast<char>(0U));
  frame.push_back(static_cast<char>(0U));
  frame.push_back(static_cast<char>(0U));
  frame.push_back(static_cast<char>(0U));
  frame.push_back(static_cast<char>(18U));
  frame.push_back(static_cast<char>(97U));
  frame.push_back(static_cast<char>(0U));
  frame.append(reinterpret_cast<const char*>(payload.data()), payload.size());

  uint32_t crc = crc32_unicore_binary(frame);
  if (!valid_crc)
  {
    crc ^= 0xFFFFFFFFU;
  }
  frame.push_back(static_cast<char>(crc & 0xFFU));
  frame.push_back(static_cast<char>((crc >> 8U) & 0xFFU));
  frame.push_back(static_cast<char>((crc >> 16U) & 0xFFU));
  frame.push_back(static_cast<char>((crc >> 24U) & 0xFFU));
  return frame;
}

std::vector<uint8_t> make_bestnavb_payload()
{
  std::vector<uint8_t> payload;
  payload.reserve(116U);
  append_le32(payload, 0U);      // SOL_COMPUTED
  append_le32(payload, 16U);     // SINGLE
  append_float64(payload, 40.07895888272);
  append_float64(payload, 116.23651029820);
  append_float64(payload, 65.8312);
  append_float32(payload, -8.4925F);
  append_le32(payload, 61U);     // WGS84
  append_float32(payload, 1.2221F);
  append_float32(payload, 1.1053F);
  append_float32(payload, 2.1970F);
  append_char4(payload, "0");
  append_float32(payload, 0.0F);
  append_float32(payload, 0.0F);
  append_le8(payload, 50U);
  append_le8(payload, 28U);
  append_le8(payload, 28U);
  append_le8(payload, 0U);
  append_le8(payload, 1U);
  append_le8(payload, 0x12U);
  append_le8(payload, 0x12U);
  append_le8(payload, 0x41U);
  append_le32(payload, 0U);      // SOL_COMPUTED
  append_le32(payload, 8U);      // DOPPLER_VELOCITY
  append_float32(payload, 0.0F);
  append_float32(payload, 0.0F);
  append_float64(payload, 0.0046);
  append_float64(payload, 335.592288);
  append_float32(payload, 0.0045F);
  append_float32(payload, 0.0194F);
  append_float32(payload, 0.0123F);
  return payload;
}

std::vector<uint8_t> make_pvtslnb_payload()
{
  std::vector<uint8_t> payload;
  payload.reserve(224U);
  append_le32(payload, 50U);     // NARROW_INT
  append_float32(payload, 60.5060F);
  append_float64(payload, 40.07898130522);
  append_float64(payload, 116.23663134427);
  append_float32(payload, 4.3353F);
  append_float32(payload, 1.8063F);
  append_float32(payload, 1.8796F);
  append_float32(payload, 0.0F);
  append_le32(payload, 16U);     // SINGLE
  append_float32(payload, 60.5060F);
  append_float64(payload, 40.07898130522);
  append_float64(payload, 116.23663134427);
  append_float32(payload, -8.4923F);
  append_le8(payload, 46U);
  append_le8(payload, 28U);
  append_le8(payload, 46U);
  append_le8(payload, 28U);
  append_float64(payload, 0.0009);
  append_float64(payload, -0.0031);
  append_float64(payload, 0.0032);
  append_le32(payload, 0U);      // heading SOL_COMPUTED
  append_float32(payload, 1.5000F);
  append_float32(payload, 123.4500F);
  append_float32(payload, 0.8000F);
  append_le8(payload, 20U);
  append_le8(payload, 18U);
  append_le8(payload, 12U);
  append_le8(payload, 8U);
  append_float32(payload, 2.1753F);
  append_float32(payload, 1.3480F);
  append_float32(payload, 0.6840F);
  append_float32(payload, 1.8392F);
  append_float32(payload, 1.7072F);
  append_float32(payload, 5.0F);
  append_le32(payload, 28U);     // PRN count placeholder
  for (int i = 0; i < 41; ++i)
  {
    payload.push_back(static_cast<uint8_t>(i + 1));
    payload.push_back(0U);
  }
  return payload;
}

}  // namespace

TEST(UnicoreBinaryNavParser, ParsesBestnavbPayload)
{
  UnicoreBinaryNavParser parser;
  UnicoreBinaryFrame frame;
  frame.message_id = 2118U;
  frame.payload = make_bestnavb_payload();

  const auto parsed = parser.parse(frame);

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->bestnav.has_value());
  ASSERT_TRUE(parsed->velocity.has_value());
  EXPECT_EQ(parsed->sentence_type, "BESTNAVB");
  EXPECT_EQ(parsed->bestnav->solution_status, "SOL_COMPUTED");
  EXPECT_EQ(parsed->bestnav->position_type, "SINGLE");
  EXPECT_EQ(parsed->bestnav->fix_quality, 1);
  EXPECT_NEAR(parsed->bestnav->latitude_deg, 40.07895888272, 1e-12);
  EXPECT_NEAR(parsed->bestnav->longitude_deg, 116.23651029820, 1e-12);
  EXPECT_NEAR(parsed->bestnav->height_msl_m, 65.8312, 1e-6);
  EXPECT_NEAR(parsed->bestnav->undulation_m, -8.4925, 1e-4);
  EXPECT_NEAR(parsed->bestnav->latitude_std_m, 1.2221, 1e-4);
  EXPECT_NEAR(parsed->bestnav->longitude_std_m, 1.1053, 1e-4);
  EXPECT_NEAR(parsed->bestnav->height_std_m, 2.1970, 1e-4);
  EXPECT_EQ(parsed->bestnav->satellites_tracked, 50);
  EXPECT_EQ(parsed->bestnav->satellites_used, 28);
  EXPECT_EQ(parsed->bestnav->extended_solution_status, 0x12);
  EXPECT_EQ(parsed->bestnav->galileo_bds3_signal_mask, 0x12);
  EXPECT_EQ(parsed->bestnav->gps_glonass_bds2_signal_mask, 0x41);
  EXPECT_EQ(parsed->bestnav->velocity_solution_status, "SOL_COMPUTED");
  EXPECT_EQ(parsed->bestnav->velocity_type, "DOPPLER_VELOCITY");
  EXPECT_NEAR(parsed->velocity->horizontal_std_mps, 0.0123, 1e-4);
}

TEST(UnicoreBinaryNavParser, ParsesPvtslnbPayload)
{
  UnicoreBinaryNavParser parser;
  UnicoreBinaryFrame frame;
  frame.message_id = 1021U;
  frame.payload = make_pvtslnb_payload();

  const auto parsed = parser.parse(frame);

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->fix.has_value());
  EXPECT_EQ(parsed->sentence_type, "PVTSLNB");
  EXPECT_EQ(parsed->fix->source, FixSource::kPvtslnb);
  EXPECT_TRUE(parsed->fix->valid_fix);
  EXPECT_EQ(parsed->fix->fix_quality, 4);
  EXPECT_NEAR(parsed->fix->latitude_deg, 40.07898130522, 1e-12);
  EXPECT_NEAR(parsed->fix->longitude_deg, 116.23663134427, 1e-12);
  EXPECT_NEAR(parsed->fix->altitude_m, 52.0137, 1e-3);
  EXPECT_EQ(parsed->fix->satellites, 28);
  EXPECT_NEAR(parsed->fix->hdop, 0.6840, 1e-4);
  ASSERT_TRUE(parsed->heading.has_value());
  EXPECT_EQ(parsed->heading->source, HeadingSource::kPvtslnb);
  EXPECT_NEAR(parsed->heading->heading_deg, 123.45, 1e-4);
  ASSERT_TRUE(parsed->heading->pitch_deg.has_value());
  EXPECT_NEAR(*parsed->heading->pitch_deg, 0.8, 1e-4);
  ASSERT_TRUE(parsed->heading->baseline_m.has_value());
  EXPECT_NEAR(*parsed->heading->baseline_m, 1.5, 1e-4);
}

TEST(UnicoreBinaryNavParser, HybridSamplesMatchAsciiWithinTolerance)
{
  Um982Parser ascii_parser;
  UnicoreBinaryNavParser binary_parser;

  const auto ascii_bestnav = ascii_parser.parse_line(make_unicore_ascii(
      "BESTNAVA,97,GPS,FINE,2294,472312000,0,0,18,16;SOL_COMPUTED,SINGLE,"
      "40.07895888272,116.23651029820,65.8312,-8.4925,WGS84,1.2221,1.1053,2.1970,"
      "\"0\",0.000,0.000,50,28,28,0,1,12,12,41,SOL_COMPUTED,DOPPLER_VELOCITY,"
      "0.000,0.000,0.0046,335.592288,0.0045,0.0194,0.0123"));
  const auto ascii_pvtsln = ascii_parser.parse_line(make_unicore_ascii(
      "PVTSLNA,97,GPS,FINE,2190,364536000,0,0,18,13;NARROW_INT,"
      "60.5060,40.07898130522,116.23663134427,4.3353,1.8063,1.8796,"
      "0.000,SINGLE,60.5060,40.07898130522,116.23663134427,-8.4923,"
      "46,28,46,28,0.0009,-0.0031,0.0032,NONE,0.0000,0.0000,0.0000,"
      "0,0,0,0,2.1753,1.3480,0.6840,1.8392,1.7072,5.0,28,25"));

  UnicoreBinaryFrame bestnav_frame;
  bestnav_frame.message_id = 2118U;
  bestnav_frame.payload = make_bestnavb_payload();
  UnicoreBinaryFrame pvtsln_frame;
  pvtsln_frame.message_id = 1021U;
  pvtsln_frame.payload = make_pvtslnb_payload();

  const auto binary_bestnav = binary_parser.parse(bestnav_frame);
  const auto binary_pvtsln = binary_parser.parse(pvtsln_frame);

  ASSERT_TRUE(ascii_bestnav.has_value() && ascii_bestnav->bestnav.has_value());
  ASSERT_TRUE(ascii_pvtsln.has_value() && ascii_pvtsln->fix.has_value());
  ASSERT_TRUE(binary_bestnav.has_value() && binary_bestnav->bestnav.has_value());
  ASSERT_TRUE(binary_pvtsln.has_value() && binary_pvtsln->fix.has_value());

  EXPECT_NEAR(binary_bestnav->bestnav->latitude_deg, ascii_bestnav->bestnav->latitude_deg, 1e-10);
  EXPECT_NEAR(binary_bestnav->bestnav->longitude_deg, ascii_bestnav->bestnav->longitude_deg, 1e-10);
  EXPECT_NEAR(binary_bestnav->bestnav->height_msl_m, ascii_bestnav->bestnav->height_msl_m, 1e-3);
  EXPECT_EQ(binary_bestnav->bestnav->fix_quality, ascii_bestnav->bestnav->fix_quality);

  EXPECT_NEAR(binary_pvtsln->fix->latitude_deg, ascii_pvtsln->fix->latitude_deg, 1e-10);
  EXPECT_NEAR(binary_pvtsln->fix->longitude_deg, ascii_pvtsln->fix->longitude_deg, 1e-10);
  EXPECT_NEAR(binary_pvtsln->fix->altitude_m, ascii_pvtsln->fix->altitude_m, 1e-3);
  EXPECT_EQ(binary_pvtsln->fix->fix_quality, ascii_pvtsln->fix->fix_quality);
  EXPECT_NEAR(std::sqrt(binary_pvtsln->fix->covariance[0]),
              std::sqrt(ascii_pvtsln->fix->covariance[0]),
              1e-3);
}

TEST(UnicoreBinaryNavParser, TransportRejectsBadCrcBeforeParsing)
{
  UnicoreTransport transport({true, true, 4096U});
  UnicoreBinaryNavParser parser;
  const std::string frame = make_binary_frame(2118U, make_bestnavb_payload(), false);

  transport.append(reinterpret_cast<const uint8_t*>(frame.data()), frame.size());
  const auto events = transport.drain();

  EXPECT_TRUE(events.empty());
  EXPECT_EQ(transport.binary_counters().crc_errors, 1U);

  UnicoreBinaryFrame raw_frame;
  raw_frame.message_id = 2118U;
  raw_frame.payload = make_bestnavb_payload();
  EXPECT_TRUE(parser.parse(raw_frame).has_value());
}

}  // namespace mowgli_unicore_gnss
