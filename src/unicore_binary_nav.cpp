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
constexpr std::size_t kPvtslnMinimumPayloadSize = 140U;

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

}  // namespace

std::optional<ParsedSentence> UnicoreBinaryNavParser::parse(const UnicoreBinaryFrame& frame) const
{
  switch (frame.message_id)
  {
    case 1021U:
      return parse_pvtslnb(frame);
    case 2118U:
      return parse_bestnavb(frame);
    default:
      return std::nullopt;
  }
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

}  // namespace mowgli_unicore_gnss
