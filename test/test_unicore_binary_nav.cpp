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

void append_le16(std::vector<uint8_t>& out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
}

void append_i16(std::vector<uint8_t>& out, int16_t value)
{
  append_le16(out, static_cast<uint16_t>(value));
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

std::vector<uint8_t> make_bestsatb_payload()
{
  std::vector<uint8_t> payload;
  append_le32(payload, 4U);

  append_le32(payload, 0U);           // GPS
  append_le32(payload, 19U);
  append_le32(payload, 0U);
  append_le32(payload, 0x03U);        // L1 + L2

  append_le32(payload, 1U);           // GLONASS
  append_le32(payload, (9U << 16U) | 57U);
  append_le32(payload, 0U);
  append_le32(payload, 0x03U);        // L1 + L2

  append_le32(payload, 5U);           // GALILEO
  append_le32(payload, 12U);
  append_le32(payload, 0U);
  append_le32(payload, 0x17U);        // E1 + E5 + common-view bit

  append_le32(payload, 6U);           // BEIDOU
  append_le32(payload, 27U);
  append_le32(payload, 0U);
  append_le32(payload, 0x05U);        // B1 + B3

  return payload;
}

std::vector<uint8_t> make_satsinfob_payload()
{
  std::vector<uint8_t> payload;
  append_le8(payload, 4U);            // satellite count
  append_le8(payload, 2U);            // version
  append_le8(payload, 0U);
  append_le8(payload, 0U);
  append_le8(payload, 0U);
  append_le8(payload, 0x07U);         // L1/L2/L5 families present

  append_le8(payload, 19U);
  append_le16(payload, 123U);
  append_le8(payload, 45U);
  append_le8(payload, 0U);
  append_le8(payload, 44U);
  append_le8(payload, 0U);
  append_le8(payload, 2U);
  append_le8(payload, 0U);
  append_le8(payload, 41U);
  append_le8(payload, 17U);
  append_le8(payload, 2U);

  append_le8(payload, 57U);
  append_le16(payload, 210U);
  append_le8(payload, 33U);
  append_le8(payload, 1U);
  append_le8(payload, 39U);
  append_le8(payload, 5U);
  append_le8(payload, 1U);

  append_le8(payload, 12U);
  append_le16(payload, 300U);
  append_le8(payload, 56U);
  append_le8(payload, 3U);
  append_le8(payload, 46U);
  append_le8(payload, 1U);
  append_le8(payload, 2U);
  append_le8(payload, 3U);
  append_le8(payload, 40U);
  append_le8(payload, 17U);
  append_le8(payload, 2U);

  append_le8(payload, 27U);
  append_le16(payload, 150U);
  append_le8(payload, 28U);
  append_le8(payload, 4U);
  append_le8(payload, 42U);
  append_le8(payload, 0U);
  append_le8(payload, 3U);
  append_le8(payload, 4U);
  append_le8(payload, 38U);
  append_le8(payload, 17U);
  append_le8(payload, 3U);
  append_le8(payload, 4U);
  append_le8(payload, 36U);
  append_le8(payload, 21U);
  append_le8(payload, 3U);

  return payload;
}

std::vector<uint8_t> make_rtcmstatusb_payload()
{
  std::vector<uint8_t> payload;
  append_le32(payload, 1124U);
  append_le32(payload, 21186U);
  append_le32(payload, 42U);
  append_le32(payload, 21U);
  append_le8(payload, 0U);
  append_le8(payload, 6U);
  append_le8(payload, 11U);
  append_le8(payload, 0U);
  append_le8(payload, 0U);
  append_le8(payload, 21U);
  return payload;
}

std::vector<uint8_t> make_rtkstatusb_payload()
{
  std::vector<uint8_t> payload;
  payload.reserve(56U);
  append_le32(payload, 0x0000000FU);
  append_le32(payload, 0U);
  append_le32(payload, 0x00000003U);
  append_le32(payload, 0x00000000U);
  append_le32(payload, 0U);
  append_le32(payload, 0x00000007U);
  append_le32(payload, 0U);
  append_le32(payload, 0x00000001U);
  append_le32(payload, 0x00000000U);
  append_le32(payload, 0x00000000U);
  append_le32(payload, 0U);
  append_le32(payload, 34U);  // NARROW_FLOAT
  append_le32(payload, 5U);
  append_le8(payload, 2U);
  append_le8(payload, 1U);
  append_le8(payload, 24U);
  append_le8(payload, 0U);
  return payload;
}

std::vector<uint8_t> make_agcb_payload()
{
  std::vector<uint8_t> payload;
  payload.reserve(20U);
  append_i16(payload, 44);
  append_i16(payload, 46);
  append_i16(payload, 63);
  append_i16(payload, -1);
  append_i16(payload, -1);
  append_i16(payload, 41);
  append_i16(payload, 1);
  append_i16(payload, 0);
  append_i16(payload, -1);
  append_i16(payload, -1);
  return payload;
}

std::vector<uint8_t> make_hwstatusb_payload()
{
  std::vector<uint8_t> payload;
  payload.reserve(40U);
  append_le32(payload, 66807U);
  append_float32(payload, 0.920F);
  append_float32(payload, 1.020F);
  append_float32(payload, 0.908F);
  append_le32(payload, 1U);
  append_float32(payload, -0.693F);
  append_float32(payload, 0.0F);
  append_le8(payload, 0U);
  append_le8(payload, 0U);
  append_le16(payload, 0x0377U);
  append_le32(payload, 0U);
  append_le32(payload, 0U);
  return payload;
}

std::vector<uint8_t> make_jamstatusb_payload()
{
  std::vector<uint8_t> payload;
  append_le32(payload, 16U);  // SINGLE
  append_le8(payload, 0U);
  append_le8(payload, 0U);
  append_le8(payload, 0U);
  append_le8(payload, 0U);
  return payload;
}

std::vector<uint8_t> make_freqjamstatusb_payload()
{
  std::vector<uint8_t> payload;
  append_le32(payload, 16U);  // SINGLE
  append_le8(payload, 255U);
  append_le8(payload, 2U);
  append_le8(payload, 0U);
  append_le8(payload, 0U);
  append_le8(payload, 0U);
  append_le8(payload, 0U);
  append_le8(payload, 0U);
  append_le8(payload, 0U);
  return payload;
}

double mean_cn0(const SatsInfoData& data)
{
  double sum = 0.0;
  int count = 0;
  for (const auto& entry : data.entries)
  {
    for (const auto& signal : entry.signals)
    {
      sum += signal.cn0_db_hz;
      ++count;
    }
  }
  return count > 0 ? (sum / static_cast<double>(count)) : 0.0;
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

TEST(UnicoreBinaryNavParser, ParsesBestsatbPayload)
{
  UnicoreBinaryNavParser parser;
  UnicoreBinaryFrame frame;
  frame.message_id = 1041U;
  frame.payload = make_bestsatb_payload();

  const auto parsed = parser.parse(frame);

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->bestsat.has_value());
  ASSERT_EQ(parsed->bestsat->entries.size(), 4U);
  EXPECT_EQ(parsed->sentence_type, "BESTSATB");
  EXPECT_EQ(parsed->bestsat->entries[0].constellation, "GPS");
  EXPECT_EQ(parsed->bestsat->entries[0].satellite_id, "19");
  EXPECT_EQ(parsed->bestsat->entries[0].used_signal_bands.size(), 2U);
  EXPECT_EQ(parsed->bestsat->entries[1].constellation, "GLO");
  EXPECT_EQ(parsed->bestsat->entries[1].satellite_id, "57+9");
  EXPECT_EQ(parsed->bestsat->entries[2].constellation, "GAL");
  EXPECT_EQ(parsed->bestsat->entries[2].status, "GOOD");
  EXPECT_EQ(parsed->bestsat->entries[3].constellation, "BDS");
}

TEST(UnicoreBinaryNavParser, ParsesSatsinfobPayload)
{
  UnicoreBinaryNavParser parser;
  UnicoreBinaryFrame frame;
  frame.message_id = 2124U;
  frame.payload = make_satsinfob_payload();

  const auto parsed = parser.parse(frame);

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->satsinfo.has_value());
  ASSERT_EQ(parsed->satsinfo->entries.size(), 4U);
  EXPECT_EQ(parsed->sentence_type, "SATSINFOB");
  EXPECT_EQ(parsed->satsinfo->version, 2);
  EXPECT_EQ(parsed->satsinfo->frequency_flag, 0x07);
  EXPECT_EQ(parsed->satsinfo->entries[0].constellation, "GPS");
  EXPECT_EQ(parsed->satsinfo->entries[0].signals.size(), 2U);
  EXPECT_EQ(parsed->satsinfo->entries[0].signals[1].band, "L2");
  EXPECT_EQ(parsed->satsinfo->entries[1].constellation, "GLO");
  EXPECT_EQ(parsed->satsinfo->entries[2].constellation, "GAL");
  EXPECT_EQ(parsed->satsinfo->entries[2].signals[1].band, "E5");
  EXPECT_EQ(parsed->satsinfo->entries[3].constellation, "BDS");
  EXPECT_EQ(parsed->satsinfo->entries[3].signals[2].band, "B3");
  EXPECT_DOUBLE_EQ(parsed->satsinfo->entries[3].signals[2].cn0_db_hz, 36.0);
}

TEST(UnicoreBinaryNavParser, ParsesRtcmstatusbPayload)
{
  UnicoreBinaryNavParser parser;
  UnicoreBinaryFrame frame;
  frame.message_id = 2125U;
  frame.payload = make_rtcmstatusb_payload();

  const auto parsed = parser.parse(frame);

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->rtcm_status.has_value());
  EXPECT_EQ(parsed->sentence_type, "RTCMSTATUSB");
  EXPECT_EQ(parsed->rtcm_status->message_id, 1124);
  EXPECT_EQ(parsed->rtcm_status->message_count, 21186);
  EXPECT_EQ(parsed->rtcm_status->base_station_id, 42);
  EXPECT_EQ(parsed->rtcm_status->satellite_count, 21);
  EXPECT_EQ(parsed->rtcm_status->observable_count[1], 6);
  EXPECT_EQ(parsed->rtcm_status->observable_count[2], 11);
  EXPECT_EQ(parsed->rtcm_status->observable_count[5], 21);
}

TEST(UnicoreBinaryNavParser, ParsesRtkstatusbPayload)
{
  UnicoreBinaryNavParser parser;
  UnicoreBinaryFrame frame;
  frame.message_id = 509U;
  frame.payload = make_rtkstatusb_payload();

  const auto parsed = parser.parse(frame);

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->rtk_status.has_value());
  EXPECT_EQ(parsed->sentence_type, "RTKSTATUSB");
  EXPECT_EQ(parsed->rtk_status->gps_source_mask, 0x0000000FU);
  EXPECT_EQ(parsed->rtk_status->bds_source_mask_1, 0x00000003U);
  EXPECT_EQ(parsed->rtk_status->glonass_source_mask, 0x00000007U);
  EXPECT_EQ(parsed->rtk_status->galileo_source_mask_1, 0x00000001U);
  EXPECT_EQ(parsed->rtk_status->position_type, "NARROW_FLOAT");
  EXPECT_EQ(parsed->rtk_status->fix_quality, 5);
  EXPECT_EQ(parsed->rtk_status->calculate_status, 5);
  EXPECT_EQ(parsed->rtk_status->ion_detected, 2);
  EXPECT_EQ(parsed->rtk_status->dual_rtk_flag, 1);
  EXPECT_EQ(parsed->rtk_status->adr_observation_count, 24);
}

TEST(UnicoreBinaryNavParser, ParsesAgcbPayload)
{
  UnicoreBinaryNavParser parser;
  UnicoreBinaryFrame frame;
  frame.message_id = 220U;
  frame.payload = make_agcb_payload();

  const auto parsed = parser.parse(frame);

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->agc.has_value());
  EXPECT_EQ(parsed->sentence_type, "AGCB");
  EXPECT_EQ(parsed->agc->antenna1[0], 44);
  EXPECT_EQ(parsed->agc->antenna1[1], 46);
  EXPECT_EQ(parsed->agc->antenna1[2], 63);
  EXPECT_EQ(parsed->agc->antenna2[0], 41);
  EXPECT_EQ(parsed->agc->antenna2[1], 1);
  EXPECT_EQ(parsed->agc->antenna2[2], 0);
}

TEST(UnicoreBinaryNavParser, ParsesHwstatusbPayload)
{
  UnicoreBinaryNavParser parser;
  UnicoreBinaryFrame frame;
  frame.message_id = 218U;
  frame.payload = make_hwstatusb_payload();

  const auto parsed = parser.parse(frame);

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->hw_status.has_value());
  EXPECT_EQ(parsed->sentence_type, "HWSTATUSB");
  EXPECT_NEAR(parsed->hw_status->dc09_v, 0.920, 1e-6);
  EXPECT_NEAR(parsed->hw_status->dc10_v, 1.020, 1e-6);
  EXPECT_NEAR(parsed->hw_status->dc18_v, 0.908, 1e-6);
  EXPECT_EQ(parsed->hw_status->clock_flag, 1);
  EXPECT_NEAR(parsed->hw_status->clock_drift_mps, -0.693, 1e-6);
  EXPECT_EQ(parsed->hw_status->hw_flag, 0x00);
  EXPECT_EQ(parsed->hw_status->pll_lock, 0x0377);
}

TEST(UnicoreBinaryNavParser, ParsesJamstatusbPayload)
{
  UnicoreBinaryNavParser parser;
  UnicoreBinaryFrame frame;
  frame.message_id = 511U;
  frame.payload = make_jamstatusb_payload();

  const auto parsed = parser.parse(frame);

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->jam_status.has_value());
  EXPECT_EQ(parsed->sentence_type, "JAMSTATUSB");
  EXPECT_EQ(parsed->jam_status->position_type, "SINGLE");
  EXPECT_EQ(parsed->jam_status->cw_ratio, 0);
  EXPECT_EQ(parsed->jam_status->cw_flag, 0);
}

TEST(UnicoreBinaryNavParser, ParsesFreqjamstatusbPayload)
{
  UnicoreBinaryNavParser parser;
  UnicoreBinaryFrame frame;
  frame.message_id = 519U;
  frame.payload = make_freqjamstatusb_payload();

  const auto parsed = parser.parse(frame);

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->freq_jam_status.has_value());
  EXPECT_EQ(parsed->sentence_type, "FREQJAMSTATUSB");
  EXPECT_EQ(parsed->freq_jam_status->position_type, "SINGLE");
  EXPECT_EQ(parsed->freq_jam_status->cw_ratio[0], 255);
  EXPECT_EQ(parsed->freq_jam_status->cw_flag[0], 2);
  EXPECT_EQ(parsed->freq_jam_status->cw_ratio[1], 0);
  EXPECT_EQ(parsed->freq_jam_status->cw_flag[2], 0);
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

TEST(UnicoreBinaryNavParser, HybridSatelliteAndRtcmSamplesMatchAsciiWithinTolerance)
{
  Um982Parser ascii_parser;
  UnicoreBinaryNavParser binary_parser;

  const auto ascii_bestsat = ascii_parser.parse_line(make_unicore_ascii(
      "BESTSATA,41,GPS,FINE,2294,472312000,0,0,18,16;4,"
      "GPS,19,GOOD,00000003,"
      "GLONASS,57+9,GOOD,00000003,"
      "GALILEO,12,GOOD,00000017,"
      "BEIDOU,27,GOOD,00000005"));
  const auto ascii_satsinfo = ascii_parser.parse_line(make_unicore_ascii(
      "SATSINFOA,97,GPS,FINE,2294,472312000,0,0,18,16;4,2,0,0,0,7,"
      "19,123,45,0,44,0,2,0,41,17,2,"
      "57,210,33,1,39,5,1,"
      "12,300,56,3,46,1,2,3,40,17,2,"
      "27,150,28,4,42,0,3,4,38,17,3,4,36,21,3"));
  const auto ascii_rtcm = ascii_parser.parse_line(make_unicore_ascii(
      "RTCMSTATUSA,76,GPS,FINE,2219,392572000,0,0,18,187;1124,21186,42,21,0,6,11,0,0,21"));

  UnicoreBinaryFrame bestsat_frame;
  bestsat_frame.message_id = 1041U;
  bestsat_frame.payload = make_bestsatb_payload();
  UnicoreBinaryFrame satsinfo_frame;
  satsinfo_frame.message_id = 2124U;
  satsinfo_frame.payload = make_satsinfob_payload();
  UnicoreBinaryFrame rtcm_frame;
  rtcm_frame.message_id = 2125U;
  rtcm_frame.payload = make_rtcmstatusb_payload();

  const auto binary_bestsat = binary_parser.parse(bestsat_frame);
  const auto binary_satsinfo = binary_parser.parse(satsinfo_frame);
  const auto binary_rtcm = binary_parser.parse(rtcm_frame);

  ASSERT_TRUE(ascii_bestsat.has_value() && ascii_bestsat->bestsat.has_value());
  ASSERT_TRUE(ascii_satsinfo.has_value() && ascii_satsinfo->satsinfo.has_value());
  ASSERT_TRUE(ascii_rtcm.has_value() && ascii_rtcm->rtcm_status.has_value());
  ASSERT_TRUE(binary_bestsat.has_value() && binary_bestsat->bestsat.has_value());
  ASSERT_TRUE(binary_satsinfo.has_value() && binary_satsinfo->satsinfo.has_value());
  ASSERT_TRUE(binary_rtcm.has_value() && binary_rtcm->rtcm_status.has_value());

  EXPECT_EQ(binary_bestsat->bestsat->entries.size(), ascii_bestsat->bestsat->entries.size());
  EXPECT_EQ(binary_satsinfo->satsinfo->entries.size(), ascii_satsinfo->satsinfo->entries.size());
  EXPECT_NEAR(mean_cn0(*binary_satsinfo->satsinfo), mean_cn0(*ascii_satsinfo->satsinfo), 1e-9);
  EXPECT_EQ(binary_rtcm->rtcm_status->message_id, ascii_rtcm->rtcm_status->message_id);
  EXPECT_EQ(binary_rtcm->rtcm_status->message_count, ascii_rtcm->rtcm_status->message_count);
  EXPECT_EQ(binary_rtcm->rtcm_status->satellite_count, ascii_rtcm->rtcm_status->satellite_count);
}

TEST(UnicoreBinaryNavParser, HybridRtkRfHardwareJammingSamplesMatchAscii)
{
  Um982Parser ascii_parser;
  UnicoreBinaryNavParser binary_parser;

  const auto ascii_rtk = ascii_parser.parse_line(make_unicore_ascii(
      "RTKSTATUSA,97,GPS,FINE,2190,365354000,0,0,18,1;0000000F,0,00000003,00000000,0,00000007,0,"
      "00000001,00000000,00000000,0,NARROW_FLOAT,5,2,1,24,0"));
  const auto ascii_agc = ascii_parser.parse_line(make_unicore_ascii(
      "AGCA,65,GPS,FINE,2190,375570000,0,0,18,37;44,46,63,-1,-1,41,1,0,-1,-1"));
  const auto ascii_hw = ascii_parser.parse_line(make_unicore_ascii(
      "HWSTATUSA,97,GPS,FINE,2221,111183000,0,0,18,15;66807,0.920,1.020,0.908,1,-0.693,0.0,0x00,0,0x0377,0,0"));
  const auto ascii_jam = ascii_parser.parse_line(make_unicore_ascii(
      "JAMSTATUSA,97,GPS,FINE,2190,365412000,0,0,18,14;SINGLE,0,0,0,0"));
  const auto ascii_freq_jam = ascii_parser.parse_line(make_unicore_ascii(
      "FREQJAMSTATUSA,97,GPS,FINE,2164,559464000,0,0,18,8;SINGLE,255,2,0,0,0,0,0,0"));

  UnicoreBinaryFrame rtk_frame;
  rtk_frame.message_id = 509U;
  rtk_frame.payload = make_rtkstatusb_payload();
  UnicoreBinaryFrame agc_frame;
  agc_frame.message_id = 220U;
  agc_frame.payload = make_agcb_payload();
  UnicoreBinaryFrame hw_frame;
  hw_frame.message_id = 218U;
  hw_frame.payload = make_hwstatusb_payload();
  UnicoreBinaryFrame jam_frame;
  jam_frame.message_id = 511U;
  jam_frame.payload = make_jamstatusb_payload();
  UnicoreBinaryFrame freq_jam_frame;
  freq_jam_frame.message_id = 519U;
  freq_jam_frame.payload = make_freqjamstatusb_payload();

  const auto binary_rtk = binary_parser.parse(rtk_frame);
  const auto binary_agc = binary_parser.parse(agc_frame);
  const auto binary_hw = binary_parser.parse(hw_frame);
  const auto binary_jam = binary_parser.parse(jam_frame);
  const auto binary_freq_jam = binary_parser.parse(freq_jam_frame);

  ASSERT_TRUE(ascii_rtk.has_value() && ascii_rtk->rtk_status.has_value());
  ASSERT_TRUE(ascii_agc.has_value() && ascii_agc->agc.has_value());
  ASSERT_TRUE(ascii_hw.has_value() && ascii_hw->hw_status.has_value());
  ASSERT_TRUE(ascii_jam.has_value() && ascii_jam->jam_status.has_value());
  ASSERT_TRUE(ascii_freq_jam.has_value() && ascii_freq_jam->freq_jam_status.has_value());
  ASSERT_TRUE(binary_rtk.has_value() && binary_rtk->rtk_status.has_value());
  ASSERT_TRUE(binary_agc.has_value() && binary_agc->agc.has_value());
  ASSERT_TRUE(binary_hw.has_value() && binary_hw->hw_status.has_value());
  ASSERT_TRUE(binary_jam.has_value() && binary_jam->jam_status.has_value());
  ASSERT_TRUE(binary_freq_jam.has_value() && binary_freq_jam->freq_jam_status.has_value());

  EXPECT_EQ(binary_rtk->rtk_status->gps_source_mask, ascii_rtk->rtk_status->gps_source_mask);
  EXPECT_EQ(binary_rtk->rtk_status->position_type, ascii_rtk->rtk_status->position_type);
  EXPECT_EQ(binary_rtk->rtk_status->adr_observation_count,
            ascii_rtk->rtk_status->adr_observation_count);
  EXPECT_EQ(binary_agc->agc->antenna1[0], ascii_agc->agc->antenna1[0]);
  EXPECT_EQ(binary_agc->agc->antenna2[0], ascii_agc->agc->antenna2[0]);
  EXPECT_NEAR(binary_hw->hw_status->clock_drift_mps, ascii_hw->hw_status->clock_drift_mps, 1e-6);
  EXPECT_EQ(binary_hw->hw_status->pll_lock, ascii_hw->hw_status->pll_lock);
  EXPECT_EQ(binary_jam->jam_status->cw_flag, ascii_jam->jam_status->cw_flag);
  EXPECT_EQ(binary_freq_jam->freq_jam_status->cw_ratio[0], ascii_freq_jam->freq_jam_status->cw_ratio[0]);
}

TEST(UnicoreBinaryNavParser, RejectsTruncatedSatelliteAndRtcmPayloads)
{
  UnicoreBinaryNavParser parser;

  UnicoreBinaryFrame bestsat_frame;
  bestsat_frame.message_id = 1041U;
  bestsat_frame.payload = make_bestsatb_payload();
  bestsat_frame.payload.resize(7U);
  EXPECT_FALSE(parser.parse(bestsat_frame).has_value());

  UnicoreBinaryFrame satsinfo_frame;
  satsinfo_frame.message_id = 2124U;
  satsinfo_frame.payload = make_satsinfob_payload();
  satsinfo_frame.payload.resize(14U);
  EXPECT_FALSE(parser.parse(satsinfo_frame).has_value());

  UnicoreBinaryFrame rtcm_frame;
  rtcm_frame.message_id = 2125U;
  rtcm_frame.payload = make_rtcmstatusb_payload();
  rtcm_frame.payload.resize(20U);
  EXPECT_FALSE(parser.parse(rtcm_frame).has_value());
}

TEST(UnicoreBinaryNavParser, RejectsTruncatedRtkRfHardwareAndJammingPayloads)
{
  UnicoreBinaryNavParser parser;

  UnicoreBinaryFrame rtk_frame;
  rtk_frame.message_id = 509U;
  rtk_frame.payload = make_rtkstatusb_payload();
  rtk_frame.payload.resize(55U);
  EXPECT_FALSE(parser.parse(rtk_frame).has_value());

  UnicoreBinaryFrame agc_frame;
  agc_frame.message_id = 220U;
  agc_frame.payload = make_agcb_payload();
  agc_frame.payload.resize(18U);
  EXPECT_FALSE(parser.parse(agc_frame).has_value());

  UnicoreBinaryFrame hw_frame;
  hw_frame.message_id = 218U;
  hw_frame.payload = make_hwstatusb_payload();
  hw_frame.payload.resize(36U);
  EXPECT_FALSE(parser.parse(hw_frame).has_value());

  UnicoreBinaryFrame jam_frame;
  jam_frame.message_id = 511U;
  jam_frame.payload = make_jamstatusb_payload();
  jam_frame.payload.resize(6U);
  EXPECT_FALSE(parser.parse(jam_frame).has_value());

  UnicoreBinaryFrame freq_jam_frame;
  freq_jam_frame.message_id = 519U;
  freq_jam_frame.payload = make_freqjamstatusb_payload();
  freq_jam_frame.payload.resize(10U);
  EXPECT_FALSE(parser.parse(freq_jam_frame).has_value());
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
