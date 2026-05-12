// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#include "mowgli_unicore_gnss/unicore_binary_nav.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>

namespace mowgli_unicore_gnss
{

namespace
{

constexpr std::size_t kBestnavPayloadSize = 116U;
constexpr std::size_t kAgcPayloadSize = 20U;
constexpr std::size_t kBestsatEntrySize = 16U;
constexpr std::size_t kFreqjamstatusPayloadSize = 12U;
constexpr std::size_t kHwstatusPayloadSize = 40U;
constexpr std::size_t kJamstatusPayloadSize = 8U;
constexpr std::size_t kPvtslnMinimumPayloadSize = 140U;
constexpr std::size_t kRtkstatusPayloadSize = 56U;
constexpr std::size_t kRtcmstatusPayloadSize = 22U;
constexpr std::size_t kSatsinfoMinimumPayloadSize = 6U;

bool has_range(const std::vector<uint8_t>& payload, std::size_t offset, std::size_t size)
{
  return offset <= payload.size() && size <= payload.size() - offset;
}

bool read_u8(const std::vector<uint8_t>& payload, std::size_t offset, uint8_t& value)
{
  if (!has_range(payload, offset, 1U))
  {
    return false;
  }
  value = payload[offset];
  return true;
}

bool read_u32(const std::vector<uint8_t>& payload, std::size_t offset, uint32_t& value)
{
  if (!has_range(payload, offset, 4U))
  {
    return false;
  }
  value = static_cast<uint32_t>(payload[offset]) |
          (static_cast<uint32_t>(payload[offset + 1U]) << 8U) |
          (static_cast<uint32_t>(payload[offset + 2U]) << 16U) |
          (static_cast<uint32_t>(payload[offset + 3U]) << 24U);
  return true;
}

bool read_i16(const std::vector<uint8_t>& payload, std::size_t offset, int16_t& value)
{
  if (!has_range(payload, offset, 2U))
  {
    return false;
  }

  const uint16_t bits = static_cast<uint16_t>(payload[offset]) |
                        (static_cast<uint16_t>(payload[offset + 1U]) << 8U);
  value = static_cast<int16_t>(bits);
  return true;
}

bool read_u16(const std::vector<uint8_t>& payload, std::size_t offset, uint16_t& value)
{
  if (!has_range(payload, offset, 2U))
  {
    return false;
  }

  value = static_cast<uint16_t>(payload[offset]) |
          (static_cast<uint16_t>(payload[offset + 1U]) << 8U);
  return true;
}

bool read_float32(const std::vector<uint8_t>& payload, std::size_t offset, double& value)
{
  uint32_t bits = 0U;
  if (!read_u32(payload, offset, bits))
  {
    return false;
  }

  float native = 0.0F;
  std::memcpy(&native, &bits, sizeof(native));
  value = static_cast<double>(native);
  return std::isfinite(value);
}

bool read_float64(const std::vector<uint8_t>& payload, std::size_t offset, double& value)
{
  if (!has_range(payload, offset, 8U))
  {
    return false;
  }

  uint64_t bits = 0U;
  for (std::size_t i = 0U; i < 8U; ++i)
  {
    bits |= static_cast<uint64_t>(payload[offset + i]) << (8U * i);
  }

  std::memcpy(&value, &bits, sizeof(value));
  return std::isfinite(value);
}

std::string read_char_array(const std::vector<uint8_t>& payload, std::size_t offset, std::size_t size)
{
  if (!has_range(payload, offset, size))
  {
    return {};
  }

  std::string text;
  text.reserve(size);
  for (std::size_t i = 0U; i < size; ++i)
  {
    const char ch = static_cast<char>(payload[offset + i]);
    if (ch == '\0')
    {
      break;
    }
    text.push_back(ch);
  }
  return text;
}

std::string bestsat_constellation_name_from_system_id(uint32_t system_id)
{
  switch (system_id)
  {
    case 0U:
      return "GPS";
    case 1U:
      return "GLO";
    case 2U:
      return "SBAS";
    case 5U:
      return "GAL";
    case 6U:
      return "BDS";
    case 7U:
      return "QZSS";
    case 9U:
      return "IRNSS";
    default:
      return "UNKNOWN";
  }
}

std::string satsinfo_constellation_name_from_system_id(uint32_t system_id)
{
  switch (system_id)
  {
    case 0U:
      return "GPS";
    case 1U:
      return "GLO";
    case 2U:
      return "SBAS";
    case 3U:
      return "GAL";
    case 4U:
      return "BDS";
    case 5U:
      return "QZSS";
    case 6U:
      return "IRNSS";
    default:
      return "UNKNOWN";
  }
}

std::vector<std::string> bestsat_signal_bands(std::string_view constellation, uint32_t signal_mask)
{
  std::vector<std::string> bands;
  if (constellation == "GPS")
  {
    if ((signal_mask & 0x01U) != 0U) bands.emplace_back("L1");
    if ((signal_mask & 0x02U) != 0U) bands.emplace_back("L2");
    if ((signal_mask & 0x04U) != 0U) bands.emplace_back("L5");
  }
  else if (constellation == "GLO")
  {
    if ((signal_mask & 0x01U) != 0U) bands.emplace_back("L1");
    if ((signal_mask & 0x02U) != 0U) bands.emplace_back("L2");
    if ((signal_mask & 0x04U) != 0U) bands.emplace_back("L3");
  }
  else if (constellation == "BDS")
  {
    if ((signal_mask & 0x01U) != 0U) bands.emplace_back("B1");
    if ((signal_mask & 0x02U) != 0U) bands.emplace_back("B2");
    if ((signal_mask & 0x04U) != 0U) bands.emplace_back("B3");
  }
  else if (constellation == "GAL")
  {
    if ((signal_mask & 0x01U) != 0U) bands.emplace_back("E1");
    if ((signal_mask & 0x02U) != 0U) bands.emplace_back("E5");
    if ((signal_mask & 0x04U) != 0U) bands.emplace_back("E5");
    if ((signal_mask & 0x08U) != 0U) bands.emplace_back("E6");
  }
  return bands;
}

std::string signal_band_from_frequency_id(std::string_view constellation, int frequency_id)
{
  if (constellation == "GPS" || constellation == "QZSS")
  {
    if (frequency_id == 0 || frequency_id == 3 || frequency_id == 11) return "L1";
    if (frequency_id == 9 || frequency_id == 17) return "L2";
    if (frequency_id == 6 || frequency_id == 14) return "L5";
    if (constellation == "QZSS" &&
        (frequency_id == 18 || frequency_id == 22 || frequency_id == 24 || frequency_id == 25))
    {
      return "L6";
    }
  }
  else if (constellation == "GLO")
  {
    if (frequency_id == 0) return "L1";
    if (frequency_id == 5) return "L2";
    if (frequency_id == 6 || frequency_id == 7) return "L3";
  }
  else if (constellation == "GAL")
  {
    if (frequency_id == 1 || frequency_id == 2) return "E1";
    if (frequency_id == 12 || frequency_id == 17) return "E5";
    if (frequency_id == 18 || frequency_id == 22) return "E6";
  }
  else if (constellation == "BDS")
  {
    if (frequency_id == 0 || frequency_id == 4 || frequency_id == 8 || frequency_id == 23)
    {
      return "B1";
    }
    if (frequency_id == 5 || frequency_id == 17 || frequency_id == 12 ||
        frequency_id == 28 || frequency_id == 13)
    {
      return "B2";
    }
    if (frequency_id == 6 || frequency_id == 21)
    {
      return "B3";
    }
  }
  else if (constellation == "SBAS")
  {
    if (frequency_id == 0) return "L1";
    if (frequency_id == 6) return "L5";
  }
  else if (constellation == "IRNSS")
  {
    if (frequency_id == 6 || frequency_id == 14) return "L5";
  }
  return "";
}

std::string solution_status_name(uint32_t code)
{
  switch (code)
  {
    case 0U: return "SOL_COMPUTED";
    case 1U: return "INSUFFICIENT_OBS";
    case 2U: return "NO_CONVERGENCE";
    case 4U: return "COV_TRACE";
    default: return std::string("UNKNOWN_") + std::to_string(code);
  }
}

std::string position_type_name(uint32_t code)
{
  switch (code)
  {
    case 0U: return "NONE";
    case 1U: return "FIXEDPOS";
    case 2U: return "FIXEDHEIGHT";
    case 8U: return "DOPPLER_VELOCITY";
    case 16U: return "SINGLE";
    case 17U: return "PSRDIFF";
    case 18U: return "SBAS";
    case 32U: return "L1_FLOAT";
    case 33U: return "IONOFREE_FLOAT";
    case 34U: return "NARROW_FLOAT";
    case 48U: return "L1_INT";
    case 49U: return "WIDE_INT";
    case 50U: return "NARROW_INT";
    case 52U: return "INS";
    case 53U: return "INS_PSRSP";
    case 54U: return "INS_PSRDIFF";
    case 55U: return "INS_RTKFLOAT";
    case 56U: return "INS_RTKFIXED";
    case 68U: return "PPP_CONVERGING";
    case 69U: return "PPP";
    default: return std::string("UNKNOWN_") + std::to_string(code);
  }
}

int position_type_to_gga_quality(uint32_t code)
{
  switch (code)
  {
    case 0U: return 0;
    case 1U:
    case 2U:
    case 16U:
    case 52U:
    case 53U:
    case 68U:
    case 69U:
      return 1;
    case 17U:
    case 54U:
      return 2;
    case 18U:
      return 9;
    case 32U:
    case 33U:
    case 34U:
    case 55U:
      return 5;
    case 48U:
    case 49U:
    case 50U:
    case 56U:
      return 4;
    default:
      return 0;
  }
}

std::string bestsat_satellite_id(uint32_t system_id, uint32_t raw_satellite_id)
{
  const uint16_t identifier = static_cast<uint16_t>(raw_satellite_id & 0xFFFFU);
  const int16_t glo_channel = static_cast<int16_t>((raw_satellite_id >> 16U) & 0xFFFFU);
  if (system_id == 1U && glo_channel != 0)
  {
    return std::to_string(identifier) +
           (glo_channel > 0 ? "+" : std::string()) + std::to_string(glo_channel);
  }
  return std::to_string(identifier);
}

std::string bestsat_status_name(uint32_t status_code)
{
  if (status_code == 0U)
  {
    return "GOOD";
  }
  return std::string("STATUS_") + std::to_string(status_code);
}

}  // namespace

std::optional<ParsedSentence> UnicoreBinaryNavParser::parse(const UnicoreBinaryFrame& frame) const
{
  switch (frame.message_id)
  {
    case 220U:
      return parse_agcb(frame);
    case 509U:
      return parse_rtkstatusb(frame);
    case 511U:
      return parse_jamstatusb(frame);
    case 519U:
      return parse_freqjamstatusb(frame);
    case 1021U:
      return parse_pvtslnb(frame);
    case 1041U:
      return parse_bestsatb(frame);
    case 2118U:
      return parse_bestnavb(frame);
    case 2124U:
      return parse_satsinfob(frame);
    case 2125U:
      return parse_rtcmstatusb(frame);
    case 218U:
      return parse_hwstatusb(frame);
    default:
      return std::nullopt;
  }
}

std::optional<ParsedSentence> UnicoreBinaryNavParser::parse_agcb(const UnicoreBinaryFrame& frame)
{
  if (frame.payload.size() < kAgcPayloadSize)
  {
    return std::nullopt;
  }

  int16_t ant1_l1 = -1;
  int16_t ant1_l2 = -1;
  int16_t ant1_l5 = -1;
  int16_t ant2_l1 = -1;
  int16_t ant2_l2 = -1;
  int16_t ant2_l5 = -1;
  if (!read_i16(frame.payload, 0U, ant1_l1) ||
      !read_i16(frame.payload, 2U, ant1_l2) ||
      !read_i16(frame.payload, 4U, ant1_l5) ||
      !read_i16(frame.payload, 10U, ant2_l1) ||
      !read_i16(frame.payload, 12U, ant2_l2) ||
      !read_i16(frame.payload, 14U, ant2_l5))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "AGCB";
  sentence.agc = AgcData{};
  sentence.agc->antenna1 = {static_cast<int>(ant1_l1),
                            static_cast<int>(ant1_l2),
                            static_cast<int>(ant1_l5)};
  sentence.agc->antenna2 = {static_cast<int>(ant2_l1),
                            static_cast<int>(ant2_l2),
                            static_cast<int>(ant2_l5)};
  return sentence;
}

std::optional<ParsedSentence> UnicoreBinaryNavParser::parse_bestsatb(const UnicoreBinaryFrame& frame)
{
  uint32_t entry_count = 0U;
  if (!read_u32(frame.payload, 0U, entry_count))
  {
    return std::nullopt;
  }

  const std::size_t required_size =
      4U + static_cast<std::size_t>(entry_count) * kBestsatEntrySize;
  if (frame.payload.size() < required_size)
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "BESTSATB";
  sentence.bestsat = BestSatData{};
  sentence.bestsat->entry_count = static_cast<int>(entry_count);
  sentence.bestsat->entries.reserve(static_cast<std::size_t>(entry_count));

  std::size_t cursor = 4U;
  for (uint32_t i = 0U; i < entry_count; ++i)
  {
    uint32_t system_id = 0U;
    uint32_t satellite_id = 0U;
    uint32_t status_code = 0U;
    uint32_t signal_mask = 0U;
    if (!read_u32(frame.payload, cursor, system_id) ||
        !read_u32(frame.payload, cursor + 4U, satellite_id) ||
        !read_u32(frame.payload, cursor + 8U, status_code) ||
        !read_u32(frame.payload, cursor + 12U, signal_mask))
    {
      return std::nullopt;
    }

    BestSatEntry entry;
    entry.constellation = bestsat_constellation_name_from_system_id(system_id);
    entry.satellite_id = bestsat_satellite_id(system_id, satellite_id);
    // N4 R1.4 documents the binary status field as a constant zero while the
    // ASCII log emits "GOOD". We normalize zero to GOOD and keep non-zero
    // values visible for future captures.
    entry.status = bestsat_status_name(status_code);
    entry.signal_mask = static_cast<int>(signal_mask);
    entry.common_view = (signal_mask & 0x10U) != 0U;
    entry.used_signal_bands = bestsat_signal_bands(entry.constellation, signal_mask);
    sentence.bestsat->entries.push_back(entry);
    cursor += kBestsatEntrySize;
  }

  return sentence;
}

std::optional<ParsedSentence> UnicoreBinaryNavParser::parse_bestnavb(const UnicoreBinaryFrame& frame)
{
  if (frame.payload.size() < kBestnavPayloadSize)
  {
    return std::nullopt;
  }

  uint32_t solution_status_code = 0U;
  uint32_t position_type_code = 0U;
  uint8_t galileo_bds3_signal_mask = 0U;
  uint8_t gps_glo_bds2_signal_mask = 0U;
  uint32_t velocity_solution_status_code = 0U;
  uint32_t velocity_type_code = 0U;
  uint8_t satellites_tracked = 0U;
  uint8_t satellites_used = 0U;
  uint8_t extended_solution_status = 0U;
  double latitude = 0.0;
  double longitude = 0.0;
  double height_msl = 0.0;
  double undulation = 0.0;
  double latitude_std = 0.0;
  double longitude_std = 0.0;
  double height_std = 0.0;
  double diff_age = 0.0;
  double sol_age = 0.0;
  double velocity_latency = 0.0;
  double velocity_age = 0.0;
  double horizontal_speed = 0.0;
  double track_deg = 0.0;
  double vertical_speed = 0.0;
  double vertical_speed_std = 0.0;
  double horizontal_speed_std = 0.0;

  if (!read_u32(frame.payload, 0U, solution_status_code) ||
      !read_u32(frame.payload, 4U, position_type_code) ||
      !read_float64(frame.payload, 8U, latitude) ||
      !read_float64(frame.payload, 16U, longitude) ||
      !read_float64(frame.payload, 24U, height_msl) ||
      !read_float32(frame.payload, 32U, undulation) ||
      !read_float32(frame.payload, 40U, latitude_std) ||
      !read_float32(frame.payload, 44U, longitude_std) ||
      !read_float32(frame.payload, 48U, height_std) ||
      !read_float32(frame.payload, 56U, diff_age) ||
      !read_float32(frame.payload, 60U, sol_age) ||
      !read_u8(frame.payload, 64U, satellites_tracked) ||
      !read_u8(frame.payload, 65U, satellites_used) ||
      !read_u8(frame.payload, 69U, extended_solution_status) ||
      !read_u8(frame.payload, 70U, galileo_bds3_signal_mask) ||
      !read_u8(frame.payload, 71U, gps_glo_bds2_signal_mask) ||
      !read_u32(frame.payload, 72U, velocity_solution_status_code) ||
      !read_u32(frame.payload, 76U, velocity_type_code) ||
      !read_float32(frame.payload, 80U, velocity_latency) ||
      !read_float32(frame.payload, 84U, velocity_age) ||
      !read_float64(frame.payload, 88U, horizontal_speed) ||
      !read_float64(frame.payload, 96U, track_deg) ||
      !read_float32(frame.payload, 104U, vertical_speed) ||
      !read_float32(frame.payload, 108U, vertical_speed_std) ||
      !read_float32(frame.payload, 112U, horizontal_speed_std))
  {
    return std::nullopt;
  }

  const std::string position_type = position_type_name(position_type_code);
  const double track_rad = track_deg * M_PI / 180.0;

  ParsedSentence sentence;
  sentence.sentence_type = "BESTNAVB";
  sentence.bestnav = BestNavData{};
  sentence.bestnav->solution_status = solution_status_name(solution_status_code);
  sentence.bestnav->position_type = position_type;
  sentence.bestnav->fix_quality = position_type_to_gga_quality(position_type_code);
  sentence.bestnav->latitude_deg = latitude;
  sentence.bestnav->longitude_deg = longitude;
  sentence.bestnav->height_msl_m = height_msl;
  sentence.bestnav->undulation_m = undulation;
  sentence.bestnav->latitude_std_m = latitude_std;
  sentence.bestnav->longitude_std_m = longitude_std;
  sentence.bestnav->height_std_m = height_std;
  sentence.bestnav->base_station_id = read_char_array(frame.payload, 52U, 4U);
  sentence.bestnav->diff_age_sec = diff_age;
  sentence.bestnav->sol_age_sec = sol_age;
  sentence.bestnav->satellites_tracked = static_cast<int>(satellites_tracked);
  sentence.bestnav->satellites_used = static_cast<int>(satellites_used);
  sentence.bestnav->extended_solution_status = static_cast<int>(extended_solution_status);
  sentence.bestnav->galileo_bds3_signal_mask = static_cast<int>(galileo_bds3_signal_mask);
  sentence.bestnav->gps_glonass_bds2_signal_mask = static_cast<int>(gps_glo_bds2_signal_mask);
  sentence.bestnav->velocity_solution_status =
      solution_status_name(velocity_solution_status_code);
  sentence.bestnav->velocity_type = position_type_name(velocity_type_code);
  sentence.bestnav->velocity_latency_sec = velocity_latency;
  sentence.bestnav->velocity_age_sec = velocity_age;
  sentence.bestnav->horizontal_speed_mps = horizontal_speed;
  sentence.bestnav->track_over_ground_deg = track_deg;
  sentence.bestnav->vertical_speed_mps = vertical_speed;
  sentence.bestnav->vertical_speed_std_mps = vertical_speed_std;
  sentence.bestnav->horizontal_speed_std_mps = horizontal_speed_std;

  sentence.velocity = VelocityData{};
  sentence.velocity->east_mps = horizontal_speed * std::sin(track_rad);
  sentence.velocity->north_mps = horizontal_speed * std::cos(track_rad);
  sentence.velocity->up_mps = vertical_speed;
  sentence.velocity->horizontal_std_mps = horizontal_speed_std;
  sentence.velocity->vertical_std_mps = vertical_speed_std;
  return sentence;
}

std::optional<ParsedSentence> UnicoreBinaryNavParser::parse_pvtslnb(const UnicoreBinaryFrame& frame)
{
  if (frame.payload.size() < kPvtslnMinimumPayloadSize)
  {
    return std::nullopt;
  }

  uint32_t bestpos_type_code = 0U;
  uint32_t psrpos_type_code = 0U;
  uint32_t heading_type_code = 0U;
  uint8_t bestpos_svs = 0U;
  uint8_t bestpos_solnsvs = 0U;
  uint8_t heading_trackedsvs = 0U;
  uint8_t heading_solnsvs = 0U;
  double bestpos_height_msl = 0.0;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude_std = 0.0;
  double latitude_std = 0.0;
  double longitude_std = 0.0;
  double diff_age = 0.0;
  double undulation = 0.0;
  double psrvel_north = 0.0;
  double psrvel_east = 0.0;
  double psrvel_ground = 0.0;
  double heading_length = 0.0;
  double heading_degree = 0.0;
  double heading_pitch = 0.0;
  double hdop = 0.0;

  if (!read_u32(frame.payload, 0U, bestpos_type_code) ||
      !read_float32(frame.payload, 4U, bestpos_height_msl) ||
      !read_float64(frame.payload, 8U, latitude) ||
      !read_float64(frame.payload, 16U, longitude) ||
      !read_float32(frame.payload, 24U, altitude_std) ||
      !read_float32(frame.payload, 28U, latitude_std) ||
      !read_float32(frame.payload, 32U, longitude_std) ||
      !read_float32(frame.payload, 36U, diff_age) ||
      !read_u32(frame.payload, 40U, psrpos_type_code) ||
      !read_float32(frame.payload, 64U, undulation) ||
      !read_u8(frame.payload, 68U, bestpos_svs) ||
      !read_u8(frame.payload, 69U, bestpos_solnsvs) ||
      !read_float64(frame.payload, 72U, psrvel_north) ||
      !read_float64(frame.payload, 80U, psrvel_east) ||
      !read_float64(frame.payload, 88U, psrvel_ground) ||
      !read_u32(frame.payload, 96U, heading_type_code) ||
      !read_float32(frame.payload, 100U, heading_length) ||
      !read_float32(frame.payload, 104U, heading_degree) ||
      !read_float32(frame.payload, 108U, heading_pitch) ||
      !read_u8(frame.payload, 112U, heading_trackedsvs) ||
      !read_u8(frame.payload, 113U, heading_solnsvs) ||
      !read_float32(frame.payload, 124U, hdop))
  {
    return std::nullopt;
  }

  const int fix_quality = position_type_to_gga_quality(bestpos_type_code);

  ParsedSentence sentence;
  sentence.sentence_type = "PVTSLNB";
  sentence.fix = FixData{};
  sentence.fix->source = FixSource::kPvtslnb;
  sentence.fix->valid_fix = fix_quality > 0;
  sentence.fix->latitude_deg = latitude;
  sentence.fix->longitude_deg = longitude;
  sentence.fix->altitude_m = bestpos_height_msl + undulation;
  sentence.fix->fix_quality = fix_quality;
  sentence.fix->satellites = static_cast<int>(bestpos_solnsvs > 0U ? bestpos_solnsvs : bestpos_svs);
  sentence.fix->hdop = hdop;
  sentence.fix->has_covariance = true;
  sentence.fix->covariance.fill(0.0);
  sentence.fix->covariance[0] = longitude_std * longitude_std;
  sentence.fix->covariance[4] = latitude_std * latitude_std;
  sentence.fix->covariance[8] = altitude_std * altitude_std;

  // PVTSLNB carries a compact heading/baseline block. Expose it only when the
  // heading solution itself is valid, so hybrid mode does not override ASCII
  // heading with the zeroed "NONE" startup state.
  if (heading_type_code == 0U || heading_length > 0.0)
  {
    sentence.heading = HeadingData{};
    sentence.heading->source = HeadingSource::kPvtslnb;
    sentence.heading->heading_deg = heading_degree;
    sentence.heading->pitch_deg = heading_pitch;
    sentence.heading->baseline_m = heading_length;
    sentence.heading->variance_deg2 = 0.0;
  }

  (void)psrpos_type_code;
  (void)diff_age;
  (void)heading_trackedsvs;
  (void)heading_solnsvs;
  (void)psrvel_ground;
  (void)psrvel_north;
  (void)psrvel_east;
  return sentence;
}

std::optional<ParsedSentence> UnicoreBinaryNavParser::parse_freqjamstatusb(
    const UnicoreBinaryFrame& frame)
{
  if (frame.payload.size() < kFreqjamstatusPayloadSize)
  {
    return std::nullopt;
  }

  uint32_t position_type_code = 0U;
  std::array<uint8_t, 3> cw_ratio{{0U, 0U, 0U}};
  std::array<uint8_t, 3> cw_flag{{0U, 0U, 0U}};
  if (!read_u32(frame.payload, 0U, position_type_code) ||
      !read_u8(frame.payload, 4U, cw_ratio[0]) ||
      !read_u8(frame.payload, 5U, cw_flag[0]) ||
      !read_u8(frame.payload, 6U, cw_ratio[1]) ||
      !read_u8(frame.payload, 7U, cw_flag[1]) ||
      !read_u8(frame.payload, 8U, cw_ratio[2]) ||
      !read_u8(frame.payload, 9U, cw_flag[2]))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "FREQJAMSTATUSB";
  sentence.freq_jam_status = FreqJamStatusData{};
  sentence.freq_jam_status->position_type = position_type_name(position_type_code);
  for (std::size_t i = 0U; i < cw_ratio.size(); ++i)
  {
    sentence.freq_jam_status->cw_ratio[i] = static_cast<int>(cw_ratio[i]);
    sentence.freq_jam_status->cw_flag[i] = static_cast<int>(cw_flag[i]);
  }
  return sentence;
}

std::optional<ParsedSentence> UnicoreBinaryNavParser::parse_hwstatusb(
    const UnicoreBinaryFrame& frame)
{
  if (frame.payload.size() < kHwstatusPayloadSize)
  {
    return std::nullopt;
  }

  double dc09_v = 0.0;
  double dc10_v = 0.0;
  double dc18_v = 0.0;
  uint32_t clock_flag = 0U;
  double clock_drift_mps = 0.0;
  uint8_t hw_flag = 0U;
  uint16_t pll_lock = 0U;
  if (!read_float32(frame.payload, 4U, dc09_v) ||
      !read_float32(frame.payload, 8U, dc10_v) ||
      !read_float32(frame.payload, 12U, dc18_v) ||
      !read_u32(frame.payload, 16U, clock_flag) ||
      !read_float32(frame.payload, 20U, clock_drift_mps) ||
      !read_u8(frame.payload, 28U, hw_flag) ||
      !read_u16(frame.payload, 30U, pll_lock))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "HWSTATUSB";
  sentence.hw_status = HwStatusData{};
  sentence.hw_status->dc09_v = dc09_v;
  sentence.hw_status->dc10_v = dc10_v;
  sentence.hw_status->dc18_v = dc18_v;
  sentence.hw_status->clock_flag = static_cast<int>(clock_flag);
  sentence.hw_status->clock_drift_mps = clock_drift_mps;
  sentence.hw_status->hw_flag = static_cast<int>(hw_flag);
  sentence.hw_status->pll_lock = static_cast<int>(pll_lock);
  return sentence;
}

std::optional<ParsedSentence> UnicoreBinaryNavParser::parse_jamstatusb(
    const UnicoreBinaryFrame& frame)
{
  if (frame.payload.size() < kJamstatusPayloadSize)
  {
    return std::nullopt;
  }

  uint32_t position_type_code = 0U;
  uint8_t cw_ratio = 0U;
  uint8_t cw_flag = 0U;
  if (!read_u32(frame.payload, 0U, position_type_code) ||
      !read_u8(frame.payload, 4U, cw_ratio) ||
      !read_u8(frame.payload, 5U, cw_flag))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "JAMSTATUSB";
  sentence.jam_status = JamStatusData{};
  sentence.jam_status->position_type = position_type_name(position_type_code);
  sentence.jam_status->cw_ratio = static_cast<int>(cw_ratio);
  sentence.jam_status->cw_flag = static_cast<int>(cw_flag);
  return sentence;
}

std::optional<ParsedSentence> UnicoreBinaryNavParser::parse_rtkstatusb(
    const UnicoreBinaryFrame& frame)
{
  if (frame.payload.size() < kRtkstatusPayloadSize)
  {
    return std::nullopt;
  }

  uint32_t gps_source_mask = 0U;
  uint32_t bds_source_mask_1 = 0U;
  uint32_t bds_source_mask_2 = 0U;
  uint32_t glonass_source_mask = 0U;
  uint32_t galileo_source_mask_1 = 0U;
  uint32_t galileo_source_mask_2 = 0U;
  uint32_t qzss_source_mask = 0U;
  uint32_t position_type_code = 0U;
  uint32_t calculate_status = 0U;
  uint8_t ion_detected = 0U;
  uint8_t dual_rtk_flag = 0U;
  uint8_t adr_observation_count = 0U;

  if (!read_u32(frame.payload, 0U, gps_source_mask) ||
      !read_u32(frame.payload, 8U, bds_source_mask_1) ||
      !read_u32(frame.payload, 12U, bds_source_mask_2) ||
      !read_u32(frame.payload, 20U, glonass_source_mask) ||
      !read_u32(frame.payload, 28U, galileo_source_mask_1) ||
      !read_u32(frame.payload, 32U, galileo_source_mask_2) ||
      !read_u32(frame.payload, 36U, qzss_source_mask) ||
      !read_u32(frame.payload, 44U, position_type_code) ||
      // N4 R1.4's table layout implies a 4-byte calculate-status enum at
      // H+48 so the trailing ion/dual/ADR bytes still land at H+52..H+54.
      !read_u32(frame.payload, 48U, calculate_status) ||
      !read_u8(frame.payload, 52U, ion_detected) ||
      !read_u8(frame.payload, 53U, dual_rtk_flag) ||
      !read_u8(frame.payload, 54U, adr_observation_count))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "RTKSTATUSB";
  sentence.rtk_status = RtkStatusData{};
  sentence.rtk_status->gps_source_mask = gps_source_mask;
  sentence.rtk_status->bds_source_mask_1 = bds_source_mask_1;
  sentence.rtk_status->bds_source_mask_2 = bds_source_mask_2;
  sentence.rtk_status->glonass_source_mask = glonass_source_mask;
  sentence.rtk_status->galileo_source_mask_1 = galileo_source_mask_1;
  sentence.rtk_status->galileo_source_mask_2 = galileo_source_mask_2;
  sentence.rtk_status->qzss_source_mask = qzss_source_mask;
  sentence.rtk_status->position_type = position_type_name(position_type_code);
  sentence.rtk_status->fix_quality = position_type_to_gga_quality(position_type_code);
  sentence.rtk_status->calculate_status = static_cast<int>(calculate_status);
  sentence.rtk_status->ion_detected = static_cast<int>(ion_detected);
  sentence.rtk_status->dual_rtk_flag = static_cast<int>(dual_rtk_flag);
  sentence.rtk_status->adr_observation_count = static_cast<int>(adr_observation_count);
  return sentence;
}

std::optional<ParsedSentence> UnicoreBinaryNavParser::parse_rtcmstatusb(
    const UnicoreBinaryFrame& frame)
{
  if (frame.payload.size() < kRtcmstatusPayloadSize)
  {
    return std::nullopt;
  }

  uint32_t message_id = 0U;
  uint32_t message_count = 0U;
  uint32_t base_station_id = 0U;
  uint32_t satellite_count = 0U;
  std::array<uint8_t, 6> observable_count{{0U, 0U, 0U, 0U, 0U, 0U}};

  if (!read_u32(frame.payload, 0U, message_id) ||
      !read_u32(frame.payload, 4U, message_count) ||
      !read_u32(frame.payload, 8U, base_station_id) ||
      !read_u32(frame.payload, 12U, satellite_count))
  {
    return std::nullopt;
  }

  for (std::size_t i = 0U; i < observable_count.size(); ++i)
  {
    if (!read_u8(frame.payload, 16U + i, observable_count[i]))
    {
      return std::nullopt;
    }
  }

  ParsedSentence sentence;
  sentence.sentence_type = "RTCMSTATUSB";
  sentence.rtcm_status = RtcmStatusData{};
  sentence.rtcm_status->message_id = static_cast<int>(message_id);
  sentence.rtcm_status->message_count = static_cast<int>(message_count);
  sentence.rtcm_status->base_station_id = static_cast<int>(base_station_id);
  sentence.rtcm_status->satellite_count = static_cast<int>(satellite_count);
  for (std::size_t i = 0U; i < observable_count.size(); ++i)
  {
    sentence.rtcm_status->observable_count[i] = static_cast<int>(observable_count[i]);
  }
  return sentence;
}

std::optional<ParsedSentence> UnicoreBinaryNavParser::parse_satsinfob(
    const UnicoreBinaryFrame& frame)
{
  if (frame.payload.size() < kSatsinfoMinimumPayloadSize)
  {
    return std::nullopt;
  }

  uint8_t satellite_count = 0U;
  uint8_t version = 0U;
  uint8_t frequency_flag = 0U;
  if (!read_u8(frame.payload, 0U, satellite_count) ||
      !read_u8(frame.payload, 1U, version) ||
      !read_u8(frame.payload, 5U, frequency_flag))
  {
    return std::nullopt;
  }

  ParsedSentence sentence;
  sentence.sentence_type = "SATSINFOB";
  sentence.satsinfo = SatsInfoData{};
  sentence.satsinfo->satellite_count = static_cast<int>(satellite_count);
  sentence.satsinfo->version = static_cast<int>(version);
  sentence.satsinfo->frequency_flag = static_cast<int>(frequency_flag);
  sentence.satsinfo->entries.reserve(static_cast<std::size_t>(satellite_count));

  std::size_t cursor = 6U;
  for (uint8_t sat_index = 0U; sat_index < satellite_count; ++sat_index)
  {
    if (!has_range(frame.payload, cursor, 8U))
    {
      return std::nullopt;
    }

    uint8_t prn = 0U;
    int16_t azimuth = 0;
    uint8_t elevation = 0U;
    uint8_t system_id = 0U;
    uint8_t cn0 = 0U;
    uint8_t frequency_id = 0U;
    uint8_t frequency_count = 0U;
    if (!read_u8(frame.payload, cursor, prn) ||
        !read_i16(frame.payload, cursor + 1U, azimuth) ||
        !read_u8(frame.payload, cursor + 3U, elevation) ||
        !read_u8(frame.payload, cursor + 4U, system_id) ||
        !read_u8(frame.payload, cursor + 5U, cn0) ||
        !read_u8(frame.payload, cursor + 6U, frequency_id) ||
        !read_u8(frame.payload, cursor + 7U, frequency_count) ||
        frequency_count == 0U)
    {
      return std::nullopt;
    }

    const std::size_t satellite_record_size = 4U + static_cast<std::size_t>(frequency_count) * 4U;
    if (!has_range(frame.payload, cursor, satellite_record_size))
    {
      return std::nullopt;
    }

    SatsInfoEntry entry;
    entry.constellation = satsinfo_constellation_name_from_system_id(system_id);
    entry.prn = static_cast<int>(prn);
    entry.azimuth_deg = static_cast<int>(azimuth);
    entry.elevation_deg = static_cast<int>(elevation);

    for (uint8_t freq_index = 0U; freq_index < frequency_count; ++freq_index)
    {
      const std::size_t signal_offset = cursor + 4U + static_cast<std::size_t>(freq_index) * 4U;
      uint8_t signal_system_id = 0U;
      uint8_t signal_cn0 = 0U;
      uint8_t signal_frequency_id = 0U;
      uint8_t repeated_frequency_count = 0U;
      if (!read_u8(frame.payload, signal_offset, signal_system_id) ||
          !read_u8(frame.payload, signal_offset + 1U, signal_cn0) ||
          !read_u8(frame.payload, signal_offset + 2U, signal_frequency_id) ||
          !read_u8(frame.payload, signal_offset + 3U, repeated_frequency_count))
      {
        return std::nullopt;
      }

      SatsInfoSignal signal;
      signal.constellation = satsinfo_constellation_name_from_system_id(signal_system_id);
      signal.system_id = static_cast<int>(signal_system_id);
      signal.frequency_id = static_cast<int>(signal_frequency_id);
      signal.band =
          signal_band_from_frequency_id(signal.constellation, static_cast<int>(signal_frequency_id));
      signal.cn0_db_hz = static_cast<double>(signal_cn0);
      entry.signals.push_back(signal);

      // N4 R1.4 labels these bytes "Sys status" and "Freq status" in the
      // binary table, but Table 7-111/7-112 and the ASCII SATSINFOA sample
      // show that they behave like system/frequency identifiers plus a
      // repeated total-frequency count. We treat the repeated count as a
      // soft hint only, not a hard validation gate.
      (void)repeated_frequency_count;
    }

    sentence.satsinfo->entries.push_back(entry);
    cursor += satellite_record_size;
  }

  return sentence;
}

}  // namespace mowgli_unicore_gnss
