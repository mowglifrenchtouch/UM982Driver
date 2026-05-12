// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#include <cstdio>
#include <string>

#include "mowgli_unicore_gnss/um982_parser.hpp"
#include <gtest/gtest.h>

namespace mowgli_unicore_gnss
{
namespace
{

uint32_t crc32_unicore(const std::string& text)
{
  uint32_t crc = 0U;
  for (const unsigned char ch : text)
  {
    crc ^= static_cast<uint32_t>(ch);
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

std::string make_nmea(std::string payload)
{
  unsigned int checksum = 0U;
  for (const unsigned char ch : payload)
  {
    checksum ^= static_cast<unsigned int>(ch);
  }

  char checksum_text[3];
  std::snprintf(checksum_text, sizeof(checksum_text), "%02X", checksum);
  return "$" + payload + "*" + checksum_text;
}

std::string make_unicore(std::string payload)
{
  char crc_text[9];
  std::snprintf(crc_text, sizeof(crc_text), "%08x", crc32_unicore(payload));
  return "#" + payload + "*" + crc_text;
}

}  // namespace

TEST(Um982Parser, ParsesGgaFix)
{
  Um982Parser parser;
  const auto parsed =
      parser.parse_line(make_nmea("GNGGA,123519,4807.038,N,01131.000,E,4,12,0.8,545.4,M,46.9,M,,"));

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->fix.has_value());
  EXPECT_EQ(parsed->sentence_type, "GGA");
  EXPECT_TRUE(parsed->fix->valid_fix);
  EXPECT_NEAR(parsed->fix->latitude_deg, 48.1173, 1e-6);
  EXPECT_NEAR(parsed->fix->longitude_deg, 11.5166667, 1e-6);
  EXPECT_NEAR(parsed->fix->altitude_m, 592.3, 1e-6);
  EXPECT_EQ(parsed->fix->fix_quality, 4);
  EXPECT_EQ(parsed->fix->satellites, 12);
  EXPECT_NEAR(parsed->fix->hdop, 0.8, 1e-6);
}

TEST(Um982Parser, ParsesHprHeading)
{
  Um982Parser parser;
  const auto parsed = parser.parse_line(make_nmea("GNHPR,235959.00,123.45,-1.25,0.50"));

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->heading.has_value());
  EXPECT_EQ(parsed->sentence_type, "HPR");
  EXPECT_NEAR(parsed->heading->heading_deg, 123.45, 1e-6);
  ASSERT_TRUE(parsed->heading->pitch_deg.has_value());
  ASSERT_TRUE(parsed->heading->roll_deg.has_value());
  EXPECT_NEAR(*parsed->heading->pitch_deg, -1.25, 1e-6);
  EXPECT_NEAR(*parsed->heading->roll_deg, 0.50, 1e-6);
}

TEST(Um982Parser, ParsesPvtslnaFixWithRtkFixed)
{
  // Real UM982 PVTSLNA layout: 10-token header followed by `;`, then
  // data starts. A `,`-only split surfaces the position-type as the
  // suffix of field 9 (`"<rx_sw>;<position_type>"`). parse_pvtslna
  // peels the prefix off via find(';').
  // Indices: 0=PVTSLNA, 1=port, 2=time_sys, 3=time_status, 4=gnss_week,
  // 5=gnss_seconds, 6-7=status, 8=leap_sec, 9=`<rx_sw>;<pos_type>`,
  // 10=altitude, 11=lat, 12=lon, 13-15=stddevs.

  Um982Parser parser;
  const auto parsed = parser.parse_line(make_unicore(
      "PVTSLNA,97,GPS,FINE,2190,364536000,0,0,18,13;NARROW_INT,"
      "60.5060,40.07898130522,116.23663134427,4.3353,1.8063,1.8796,"
      "0.000,SINGLE,60.5060,40.07898130522,116.23663134427,-8.4923,"
      "46,28,46,28,0.0009,-0.0031,0.0032,NONE,0.0000,0.0000,0.0000,"
      "0,0,0,0,2.1753,1.3480,0.6840,1.8392,1.7072,5.0,28,25"));

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->fix.has_value());
  EXPECT_EQ(parsed->sentence_type, "PVTSLNA");
  EXPECT_TRUE(parsed->fix->valid_fix);
  EXPECT_EQ(parsed->fix->fix_quality, 4);
  EXPECT_EQ(parsed->fix->satellites, 28);
  EXPECT_NEAR(parsed->fix->latitude_deg, 40.07898130522, 1e-12);
  EXPECT_NEAR(parsed->fix->longitude_deg, 116.23663134427, 1e-12);
  EXPECT_NEAR(parsed->fix->altitude_m, 52.0137, 1e-9);
  EXPECT_NEAR(parsed->fix->hdop, 0.6840, 1e-9);
  EXPECT_TRUE(parsed->fix->has_covariance);
  EXPECT_NEAR(parsed->fix->covariance[0], 1.8796 * 1.8796, 1e-9);
  EXPECT_NEAR(parsed->fix->covariance[4], 1.8063 * 1.8063, 1e-9);
  EXPECT_NEAR(parsed->fix->covariance[8], 4.3353 * 4.3353, 1e-9);
}

TEST(Um982Parser, ParsesPvtslnaFloatRtk)
{
  Um982Parser parser;
  const auto parsed = parser.parse_line(make_unicore(
      "PVTSLNA,97,GPS,FINE,2190,364536000,0,0,18,13;NARROW_FLOAT,"
      "60.5060,40.07898130522,116.23663134427,4.3353,1.8063,1.8796,"
      "1.250,SINGLE,60.5060,40.07898130522,116.23663134427,-8.4923,"
      "46,28,46,28,0.0009,-0.0031,0.0032,NONE,0.0000,0.0000,0.0000,"
      "0,0,0,0,2.1753,1.3480,0.6840,1.8392,1.7072,5.0,28,25"));

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->fix.has_value());
  EXPECT_TRUE(parsed->fix->valid_fix);
  EXPECT_EQ(parsed->fix->fix_quality, 5);  // float RTK -> NMEA quality 5
}

TEST(Um982Parser, ParsesPvtslnaNumericPositionType)
{
  // Some firmware variants emit BESTPOSA position-type as numeric code
  // ("50" = NARROW_INT) instead of the string form. Position-type lives
  // after the `;` in field 9.
  Um982Parser parser;
  const auto parsed = parser.parse_line(make_unicore(
      "PVTSLNA,97,GPS,FINE,2190,364536000,0,0,18,13;50,"
      "60.5060,40.07898130522,116.23663134427,4.3353,1.8063,1.8796,"
      "0.000,SINGLE,60.5060,40.07898130522,116.23663134427,-8.4923,"
      "46,28,46,28,0.0009,-0.0031,0.0032,NONE,0.0000,0.0000,0.0000,"
      "0,0,0,0,2.1753,1.3480,0.6840,1.8392,1.7072,5.0,28,25"));

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->fix.has_value());
  EXPECT_TRUE(parsed->fix->valid_fix);
  EXPECT_EQ(parsed->fix->fix_quality, 4);
}

TEST(Um982Parser, ParsesPvtslnaNoFixWhenPositionTypeNone)
{
  Um982Parser parser;
  const auto parsed = parser.parse_line(make_unicore(
      "PVTSLNA,97,GPS,FINE,2190,364536000,0,0,18,13;NONE,"
      "60.5060,40.07898130522,116.23663134427,4.3353,1.8063,1.8796,"
      "0.000,SINGLE,60.5060,40.07898130522,116.23663134427,-8.4923,"
      "46,28,46,28,0.0009,-0.0031,0.0032,NONE,0.0000,0.0000,0.0000,"
      "0,0,0,0,2.1753,1.3480,0.6840,1.8392,1.7072,5.0,28,25"));

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->fix.has_value());
  EXPECT_FALSE(parsed->fix->valid_fix);
  EXPECT_EQ(parsed->fix->fix_quality, 0);
}

TEST(Um982Parser, ParsesGsvSatellitesInView)
{
  Um982Parser parser;
  const auto parsed = parser.parse_line(
      make_nmea("GPGSV,3,1,12,01,40,083,46,02,17,308,41,03,52,210,42,04,71,047,46"));

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->gsv.has_value());
  EXPECT_EQ(parsed->sentence_type, "GSV");
  EXPECT_EQ(parsed->gsv->talker, "GP");
  EXPECT_EQ(parsed->gsv->satellites_in_view, 12);
}

TEST(Um982Parser, ParsesGsvPerConstellationTalkers)
{
  Um982Parser parser;
  const auto gl = parser.parse_line(make_nmea("GLGSV,1,1,07"));
  const auto ga = parser.parse_line(make_nmea("GAGSV,1,1,10"));
  const auto gb = parser.parse_line(make_nmea("GBGSV,1,1,08"));

  ASSERT_TRUE(gl.has_value() && gl->gsv.has_value());
  ASSERT_TRUE(ga.has_value() && ga->gsv.has_value());
  ASSERT_TRUE(gb.has_value() && gb->gsv.has_value());
  EXPECT_EQ(gl->gsv->talker, "GL");
  EXPECT_EQ(gl->gsv->satellites_in_view, 7);
  EXPECT_EQ(ga->gsv->talker, "GA");
  EXPECT_EQ(ga->gsv->satellites_in_view, 10);
  EXPECT_EQ(gb->gsv->talker, "GB");
  EXPECT_EQ(gb->gsv->satellites_in_view, 8);
}

TEST(Um982Parser, ParsesBestnavaVelocity)
{
  Um982Parser parser;
  const auto parsed = parser.parse_line(make_unicore("BESTNAVA,foo,bar,3.5,90.0,-0.4,0.2,0.1"));

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->velocity.has_value());
  EXPECT_EQ(parsed->sentence_type, "BESTNAVA");
  EXPECT_NEAR(parsed->velocity->east_mps, 3.5, 1e-6);
  EXPECT_NEAR(parsed->velocity->north_mps, 0.0, 1e-6);
  EXPECT_NEAR(parsed->velocity->up_mps, -0.4, 1e-6);
  EXPECT_NEAR(parsed->velocity->horizontal_std_mps, 0.1, 1e-6);
  EXPECT_NEAR(parsed->velocity->vertical_std_mps, 0.2, 1e-6);
}

TEST(Um982Parser, ParsesBestnavaStructuredDiagnostics)
{
  Um982Parser parser;
  const auto parsed = parser.parse_line(make_unicore(
      "BESTNAVA,97,GPS,FINE,2294,472312000,0,0,18,16;SOL_COMPUTED,SINGLE,"
      "40.07895888272,116.23651029820,65.8312,-8.4925,WGS84,1.2221,1.1053,2.1970,"
      "\"0\",0.000,0.000,50,28,28,0,1,12,12,41,SOL_COMPUTED,DOPPLER_VELOCITY,"
      "0.000,0.000,0.0046,335.592288,0.0045,0.0194,0.0123"));

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->bestnav.has_value());
  ASSERT_TRUE(parsed->velocity.has_value());
  EXPECT_EQ(parsed->sentence_type, "BESTNAVA");
  EXPECT_EQ(parsed->bestnav->solution_status, "SOL_COMPUTED");
  EXPECT_EQ(parsed->bestnav->position_type, "SINGLE");
  EXPECT_EQ(parsed->bestnav->fix_quality, 1);
  EXPECT_NEAR(parsed->bestnav->latitude_deg, 40.07895888272, 1e-12);
  EXPECT_NEAR(parsed->bestnav->longitude_deg, 116.23651029820, 1e-12);
  EXPECT_NEAR(parsed->bestnav->height_msl_m, 65.8312, 1e-9);
  EXPECT_NEAR(parsed->bestnav->latitude_std_m, 1.2221, 1e-9);
  EXPECT_NEAR(parsed->bestnav->longitude_std_m, 1.1053, 1e-9);
  EXPECT_NEAR(parsed->bestnav->height_std_m, 2.1970, 1e-9);
  EXPECT_EQ(parsed->bestnav->base_station_id, "0");
  EXPECT_NEAR(parsed->bestnav->diff_age_sec, 0.0, 1e-9);
  EXPECT_NEAR(parsed->bestnav->sol_age_sec, 0.0, 1e-9);
  EXPECT_EQ(parsed->bestnav->satellites_tracked, 50);
  EXPECT_EQ(parsed->bestnav->satellites_used, 28);
  EXPECT_EQ(parsed->bestnav->extended_solution_status, 0x12);
  EXPECT_EQ(parsed->bestnav->galileo_bds3_signal_mask, 0x12);
  EXPECT_EQ(parsed->bestnav->gps_glonass_bds2_signal_mask, 0x41);
  EXPECT_EQ(parsed->bestnav->velocity_solution_status, "SOL_COMPUTED");
  EXPECT_EQ(parsed->bestnav->velocity_type, "DOPPLER_VELOCITY");
  EXPECT_NEAR(parsed->bestnav->horizontal_speed_mps, 0.0046, 1e-9);
  EXPECT_NEAR(parsed->bestnav->track_over_ground_deg, 335.592288, 1e-9);
  EXPECT_NEAR(parsed->bestnav->vertical_speed_mps, 0.0045, 1e-9);
  EXPECT_NEAR(parsed->bestnav->vertical_speed_std_mps, 0.0194, 1e-9);
  EXPECT_NEAR(parsed->bestnav->horizontal_speed_std_mps, 0.0123, 1e-9);
}

TEST(Um982Parser, ParsesRtkstatusaDiagnostics)
{
  Um982Parser parser;
  const auto parsed = parser.parse_line(make_unicore(
      "RTKSTATUSA,97,GPS,FINE,2190,365354000,0,0,18,1;0000000F,0,00000003,00000000,0,00000007,0,"
      "00000001,00000000,00000000,0,NARROW_FLOAT,5,2,1,24,0"));

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->rtk_status.has_value());
  EXPECT_EQ(parsed->sentence_type, "RTKSTATUSA");
  EXPECT_EQ(parsed->rtk_status->gps_source_mask, 0x0000000FU);
  EXPECT_EQ(parsed->rtk_status->bds_source_mask_1, 0x00000003U);
  EXPECT_EQ(parsed->rtk_status->bds_source_mask_2, 0x00000000U);
  EXPECT_EQ(parsed->rtk_status->glonass_source_mask, 0x00000007U);
  EXPECT_EQ(parsed->rtk_status->galileo_source_mask_1, 0x00000001U);
  EXPECT_EQ(parsed->rtk_status->galileo_source_mask_2, 0x00000000U);
  EXPECT_EQ(parsed->rtk_status->qzss_source_mask, 0x00000000U);
  EXPECT_EQ(parsed->rtk_status->position_type, "NARROW_FLOAT");
  EXPECT_EQ(parsed->rtk_status->fix_quality, 5);
  EXPECT_EQ(parsed->rtk_status->calculate_status, 5);
  EXPECT_EQ(parsed->rtk_status->ion_detected, 2);
  EXPECT_EQ(parsed->rtk_status->dual_rtk_flag, 1);
  EXPECT_EQ(parsed->rtk_status->adr_observation_count, 24);
}

TEST(Um982Parser, ParsesRtcmstatusaDiagnostics)
{
  Um982Parser parser;
  const auto parsed = parser.parse_line(make_unicore(
      "RTCMSTATUSA,76,GPS,FINE,2219,392572000,0,0,18,187;1124,21186,0,21,0,6,11,0,0,21"));

  ASSERT_TRUE(parsed.has_value());
  ASSERT_TRUE(parsed->rtcm_status.has_value());
  EXPECT_EQ(parsed->sentence_type, "RTCMSTATUSA");
  EXPECT_EQ(parsed->rtcm_status->message_id, 1124);
  EXPECT_EQ(parsed->rtcm_status->message_count, 21186);
  EXPECT_EQ(parsed->rtcm_status->base_station_id, 0);
  EXPECT_EQ(parsed->rtcm_status->satellite_count, 21);
  EXPECT_EQ(parsed->rtcm_status->observable_count[0], 0);
  EXPECT_EQ(parsed->rtcm_status->observable_count[1], 6);
  EXPECT_EQ(parsed->rtcm_status->observable_count[2], 11);
  EXPECT_EQ(parsed->rtcm_status->observable_count[3], 0);
  EXPECT_EQ(parsed->rtcm_status->observable_count[4], 0);
  EXPECT_EQ(parsed->rtcm_status->observable_count[5], 21);
}

TEST(Um982Parser, RejectsBadChecksum)
{
  Um982Parser parser;
  const auto parsed = parser.parse_line("$GPHDT,10.0,T*00");
  EXPECT_FALSE(parsed.has_value());
  EXPECT_EQ(parser.counters().nmea_checksum_errors, 1U);
}

TEST(Um982Parser, CountsParseErrorsOnMalformedStructuredLog)
{
  Um982Parser parser;
  const auto parsed = parser.parse_line(make_unicore(
      "RTCMSTATUSA,76,GPS,FINE,2219,392572000,0,0,18,187;1124,broken,0,21,0,6,11,0,0,21"));

  EXPECT_FALSE(parsed.has_value());
  EXPECT_EQ(parser.counters().parse_errors, 1U);
}

}  // namespace mowgli_unicore_gnss
