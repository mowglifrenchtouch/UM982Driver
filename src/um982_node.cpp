// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include "mowgli_unicore_gnss/serial_port.hpp"
#include "mowgli_unicore_gnss/um982_parser.hpp"
#include <compass_msgs/msg/azimuth.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rtcm_msgs/msg/message.hpp>

namespace mowgli_unicore_gnss
{

namespace
{

using SteadyTime = std::chrono::steady_clock::time_point;

template <typename T>
struct TimedData
{
  T data;
  SteadyTime received_at;
};

struct Cn0Accumulator
{
  double sum_db_hz{0.0};
  double max_db_hz{-1.0};
  std::size_t sample_count{0U};
};

diagnostic_msgs::msg::KeyValue kv(const std::string& key, const std::string& value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string to_string_or_nan(double value)
{
  if (!std::isfinite(value))
  {
    return "nan";
  }
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(3);
  oss << value;
  return oss.str();
}

std::string fix_source_name(FixSource source)
{
  switch (source)
  {
    case FixSource::kGga:
      return "GGA";
    case FixSource::kPvtslna:
      return "PVTSLNA";
  }
  return "unknown";
}

std::string heading_source_name(HeadingSource source)
{
  switch (source)
  {
    case HeadingSource::kHdt:
      return "HDT";
    case HeadingSource::kHpr:
      return "HPR";
  }
  return "unknown";
}

const char* fix_type_from_quality(int quality)
{
  if (quality == 4) return "3D-RTK-Fixed";
  if (quality == 5) return "3D-RTK-Float";
  if (quality == 2 || quality == 9) return "3D-DGPS";
  if (quality == 1) return "3D";
  return "no-fix";
}

const char* carrier_solution_from_quality(int quality)
{
  if (quality == 4) return "fixed";
  if (quality == 5) return "float";
  return "none";
}

std::string to_hex_byte(int value)
{
  if (value < 0)
  {
    return "n/a";
  }
  std::ostringstream oss;
  oss << "0x";
  oss.setf(std::ios::hex, std::ios::basefield);
  oss.setf(std::ios::uppercase);
  if (value < 0x10)
  {
    oss << '0';
  }
  oss << value;
  return oss.str();
}

std::string to_hex_word(uint32_t value)
{
  std::ostringstream oss;
  oss << "0x";
  oss.setf(std::ios::hex, std::ios::basefield);
  oss.setf(std::ios::uppercase);
  oss << value;
  return oss.str();
}

std::string join_strings(const std::vector<std::string>& values)
{
  std::string joined;
  for (const auto& value : values)
  {
    if (value.empty())
    {
      continue;
    }
    if (!joined.empty())
    {
      joined += ", ";
    }
    joined += value;
  }
  return joined.empty() ? std::string("n/a") : joined;
}

std::string describe_gps_glo_bds2_signal_mask(int mask)
{
  if (mask < 0)
  {
    return "n/a";
  }

  std::vector<std::string> parts;
  if ((mask & 0x01) != 0) parts.emplace_back("GPS L1");
  if ((mask & 0x02) != 0) parts.emplace_back("GPS L2");
  if ((mask & 0x04) != 0) parts.emplace_back("GPS L5");
  if ((mask & 0x08) != 0) parts.emplace_back("BDS2 B3I");
  if ((mask & 0x10) != 0) parts.emplace_back("GLO L1");
  if ((mask & 0x20) != 0) parts.emplace_back("GLO L2");
  if ((mask & 0x40) != 0) parts.emplace_back("BDS2 B1I");
  if ((mask & 0x80) != 0) parts.emplace_back("BDS2 B2I");
  return join_strings(parts);
}

std::string describe_galileo_bds3_signal_mask(int mask)
{
  if (mask < 0)
  {
    return "n/a";
  }

  std::vector<std::string> parts;
  if ((mask & 0x01) != 0) parts.emplace_back("GAL E1");
  if ((mask & 0x02) != 0) parts.emplace_back("GAL E5b");
  if ((mask & 0x04) != 0) parts.emplace_back("GAL E5a");
  if ((mask & 0x10) != 0) parts.emplace_back("BDS3 B1I");
  if ((mask & 0x20) != 0) parts.emplace_back("BDS3 B3I");
  if ((mask & 0x40) != 0) parts.emplace_back("BDS3 B2a");
  if ((mask & 0x80) != 0) parts.emplace_back("BDS3 B1C");
  return join_strings(parts);
}

std::string describe_ext_solution_status(int value)
{
  if (value < 0)
  {
    return "n/a";
  }

  const bool rtk_checked = (value & 0x01) != 0;
  const int iono_mode = (value >> 1) & 0x07;
  std::string iono_text = "unknown";
  switch (iono_mode)
  {
    case 1:
      iono_text = "klobuchar";
      break;
    case 2:
      iono_text = "sbas_grid";
      break;
    case 3:
      iono_text = "multi_freq";
      break;
    case 4:
      iono_text = "pseudorange_diff";
      break;
    default:
      break;
  }
  return std::string("rtk_checked=") + (rtk_checked ? "true" : "false") + ", iono=" + iono_text;
}

std::string describe_rtk_calculate_status(int value)
{
  switch (value)
  {
    case 0:
      return "no differential data";
    case 1:
      return "base insufficient obs";
    case 2:
      return "high differential latency";
    case 3:
      return "active ionosphere";
    case 4:
      return "rover insufficient obs";
    case 5:
      return "rtk solution available";
    default:
      return "unknown";
  }
}

std::string describe_dual_rtk_flag(int value)
{
  switch (value)
  {
    case 0:
      return "baseline unsolved";
    case 1:
      return "within limit";
    case 2:
      return "out of limit";
    case 255:
      return "baseline length not configured";
    default:
      return "unknown";
  }
}

std::size_t count_bits(uint32_t value)
{
  std::size_t count = 0U;
  while (value != 0U)
  {
    count += static_cast<std::size_t>(value & 1U);
    value >>= 1U;
  }
  return count;
}

void add_cn0_sample(Cn0Accumulator& acc, double cn0_db_hz)
{
  if (!std::isfinite(cn0_db_hz) || cn0_db_hz <= 0.0)
  {
    return;
  }
  acc.sum_db_hz += cn0_db_hz;
  acc.max_db_hz = std::max(acc.max_db_hz, cn0_db_hz);
  ++acc.sample_count;
}

std::string mean_cn0_or_na(const Cn0Accumulator& acc)
{
  if (acc.sample_count == 0U)
  {
    return "n/a";
  }
  return to_string_or_nan(acc.sum_db_hz / static_cast<double>(acc.sample_count));
}

std::string max_cn0_or_na(const Cn0Accumulator& acc)
{
  if (acc.sample_count == 0U)
  {
    return "n/a";
  }
  return to_string_or_nan(acc.max_db_hz);
}

std::size_t count_valid_agc_channels(const std::array<int, 3>& values)
{
  std::size_t count = 0U;
  for (const int value : values)
  {
    if (value >= 0)
    {
      ++count;
    }
  }
  return count;
}

double mean_valid_agc(const std::array<int, 3>& values)
{
  double sum = 0.0;
  std::size_t count = 0U;
  for (const int value : values)
  {
    if (value >= 0)
    {
      sum += static_cast<double>(value);
      ++count;
    }
  }
  if (count == 0U)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return sum / static_cast<double>(count);
}

int min_valid_agc(const std::array<int, 3>& values)
{
  int min_value = std::numeric_limits<int>::max();
  bool found = false;
  for (const int value : values)
  {
    if (value >= 0)
    {
      min_value = std::min(min_value, value);
      found = true;
    }
  }
  return found ? min_value : -1;
}

std::string describe_agc_values(const std::array<int, 3>& values)
{
  static const std::array<const char*, 3> kBands = {"L1", "L2", "L5"};
  std::vector<std::string> parts;
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    if (values[i] >= 0)
    {
      parts.emplace_back(std::string(kBands[i]) + "=" + std::to_string(values[i]));
    }
  }
  return join_strings(parts);
}

bool voltage_in_range(double value, double min_value, double max_value)
{
  return std::isfinite(value) && value >= min_value && value <= max_value;
}

std::string describe_hw_flag_bits(int hw_flag)
{
  if (hw_flag < 0)
  {
    return "n/a";
  }

  std::vector<std::string> parts;
  parts.emplace_back((hw_flag & 0x01) != 0 ? "crystal" : "oscillator");
  parts.emplace_back((hw_flag & 0x02) != 0 ? "tcxo" : "vcxo");
  parts.emplace_back((hw_flag & 0x04) != 0 ? "20mhz" : "26mhz");
  parts.emplace_back((hw_flag & 0x08) != 0 ? "osc+crystal" : "osc-only");
  parts.emplace_back((hw_flag & 0x10) != 0 ? "external_clock" : "internal_clock");
  parts.emplace_back((hw_flag & 0x80) != 0 ? "flag_valid" : "flag_unknown");
  return join_strings(parts);
}

std::string describe_clock_status(int clock_flag, int hw_flag)
{
  if (clock_flag < 0)
  {
    return "n/a";
  }

  std::string status = clock_flag == 1 ? "valid" : "invalid";
  if (hw_flag >= 0)
  {
    status += std::string(", ") + (((hw_flag & 0x10) != 0) ? "external" : "internal");
  }
  return status;
}

std::string describe_pll_status(int pll_lock)
{
  if (pll_lock < 0)
  {
    return "n/a";
  }
  if (pll_lock == 0)
  {
    return "unlocked";
  }
  return std::string("lock-mask=") + to_hex_word(static_cast<uint32_t>(pll_lock));
}

std::string describe_antenna_status(const std::optional<AgcData>& agc)
{
  if (!agc.has_value())
  {
    return "unknown";
  }

  const std::size_t main_valid = count_valid_agc_channels(agc->antenna1);
  const std::size_t aux_valid = count_valid_agc_channels(agc->antenna2);
  if (main_valid == 0U)
  {
    return "main invalid";
  }
  if (aux_valid == 0U)
  {
    return "main ok, aux unavailable";
  }
  return "main ok, aux ok";
}

std::string describe_jam_flag(int flag)
{
  switch (flag)
  {
    case 0:
      return "none";
    case 1:
      return "cw_jam";
    case 2:
      return "strong_cw_jam";
    default:
      return "unknown";
  }
}

}  // namespace

class Um982Node : public rclcpp::Node
{
public:
  Um982Node() : rclcpp::Node("um982_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/gps");
    baudrate_ = declare_parameter<int>("baudrate", 921600);
    frame_id_ = declare_parameter<std::string>("frame_id", "gps");
    data_timeout_sec_ = declare_parameter<double>("data_timeout_sec", 1.0);
    reconnect_interval_sec_ = declare_parameter<double>("reconnect_interval_sec", 1.0);
    read_poll_hz_ = declare_parameter<double>("read_poll_hz", 200.0);
    fix_topic_ = declare_parameter<std::string>("fix_topic", "/gps/fix");
    heading_topic_ = declare_parameter<std::string>("heading_topic", "/gps/azimuth");
    diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic", "/gps/diagnostics");
    rtcm_topic_ = declare_parameter<std::string>("rtcm_topic", "/ntrip_client/rtcm");
    rtcm_timeout_sec_ = declare_parameter<double>("rtcm_timeout_sec", 5.0);
    max_diff_age_sec_ = declare_parameter<double>("max_diff_age_sec", 5.0);
    enable_rtk_status_ = declare_parameter<bool>("enable_rtk_status", true);
    enable_rtcm_status_ = declare_parameter<bool>("enable_rtcm_status", true);
    enable_satellite_status_ = declare_parameter<bool>("enable_satellite_status", true);
    enable_satsinfo_ = declare_parameter<bool>("enable_satsinfo", true);
    satellite_diag_timeout_sec_ = declare_parameter<double>("satellite_diag_timeout_sec", 5.0);
    enable_rf_status_ = declare_parameter<bool>("enable_rf_status", true);
    enable_hw_status_ = declare_parameter<bool>("enable_hw_status", true);
    enable_jamming_status_ = declare_parameter<bool>("enable_jamming_status", true);
    rf_diag_timeout_sec_ = declare_parameter<double>("rf_diag_timeout_sec", 5.0);

    serial_.configure(port_, baudrate_);

    auto fix_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(fix_topic_, fix_qos);
    heading_pub_ = create_publisher<compass_msgs::msg::Azimuth>(heading_topic_, rclcpp::QoS(10));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_,
                                                                               rclcpp::QoS(10));
    rtcm_sub_ = create_subscription<rtcm_msgs::msg::Message>(rtcm_topic_,
                                                             rclcpp::QoS(10),
                                                             std::bind(&Um982Node::handle_rtcm,
                                                                       this,
                                                                       std::placeholders::_1));

    const auto poll_period = std::chrono::duration<double>(1.0 / std::max(1.0, read_poll_hz_));
    poll_timer_ =
        create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(poll_period),
                          std::bind(&Um982Node::poll_serial, this));
    diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1),
                                           std::bind(&Um982Node::publish_diagnostics, this));

    RCLCPP_INFO(get_logger(),
                "UM982 node configured: port=%s baudrate=%d fix_topic=%s heading_topic=%s "
                "rtcm_timeout=%.1fs max_diff_age=%.1fs sat_diag_timeout=%.1fs rf_diag_timeout=%.1fs",
                port_.c_str(),
                baudrate_,
                fix_topic_.c_str(),
                heading_topic_.c_str(),
                rtcm_timeout_sec_,
                max_diff_age_sec_,
                satellite_diag_timeout_sec_,
                rf_diag_timeout_sec_);
  }

private:
  void poll_serial()
  {
    ensure_serial_open();
    if (!serial_.is_open())
    {
      return;
    }

    std::uint8_t buffer[1024];
    while (true)
    {
      const ssize_t bytes_read = serial_.read(buffer, sizeof(buffer));
      if (bytes_read > 0)
      {
        rx_buffer_.append(reinterpret_cast<const char*>(buffer),
                          static_cast<std::size_t>(bytes_read));
        drain_lines();
        continue;
      }

      if (bytes_read == 0)
      {
        break;
      }

      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        break;
      }

      RCLCPP_ERROR_THROTTLE(get_logger(),
                            *get_clock(),
                            5000,
                            "Serial read failed on %s: %s",
                            port_.c_str(),
                            std::strerror(errno));
      serial_.close();
      break;
    }

    if (rx_buffer_.size() > 8192U)
    {
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           5000,
                           "Dropping oversized UM982 receive buffer");
      rx_buffer_.clear();
    }
  }

  void ensure_serial_open()
  {
    const auto now = std::chrono::steady_clock::now();
    if (serial_.is_open())
    {
      return;
    }
    if (last_open_attempt_.has_value() &&
        std::chrono::duration<double>(now - *last_open_attempt_).count() < reconnect_interval_sec_)
    {
      return;
    }

    last_open_attempt_ = now;
    if (serial_.open())
    {
      RCLCPP_INFO(get_logger(), "Opened UM982 serial port %s at %d baud", port_.c_str(), baudrate_);
      rx_buffer_.clear();
    }
    else
    {
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           5000,
                           "Unable to open UM982 serial port %s: %s",
                           port_.c_str(),
                           std::strerror(errno));
    }
  }

  void drain_lines()
  {
    std::size_t newline = rx_buffer_.find('\n');
    while (newline != std::string::npos)
    {
      std::string line = rx_buffer_.substr(0U, newline);
      rx_buffer_.erase(0U, newline + 1U);

      if (!line.empty() && line.back() == '\r')
      {
        line.pop_back();
      }

      process_line(line);
      newline = rx_buffer_.find('\n');
    }
  }

  void process_line(const std::string& line)
  {
    auto parsed = parser_.parse_line(line);
    if (!parsed.has_value())
    {
      return;
    }

    ++sentence_counts_[parsed->sentence_type];
    const auto received_at = std::chrono::steady_clock::now();

    if (parsed->fix.has_value())
    {
      if (parsed->fix->source == FixSource::kPvtslna)
      {
        latest_pvtslna_fix_ = TimedData<FixData>{*parsed->fix, received_at};
      }
      else
      {
        latest_gga_fix_ = TimedData<FixData>{*parsed->fix, received_at};
      }
      publish_active_fix();
    }

    if (parsed->heading.has_value())
    {
      if (parsed->heading->source == HeadingSource::kHpr)
      {
        latest_hpr_heading_ = TimedData<HeadingData>{*parsed->heading, received_at};
      }
      else
      {
        latest_hdt_heading_ = TimedData<HeadingData>{*parsed->heading, received_at};
      }
      publish_active_heading();
    }

    if (parsed->velocity.has_value())
    {
      latest_velocity_ = TimedData<VelocityData>{*parsed->velocity, received_at};
    }

    if (parsed->bestnav.has_value())
    {
      latest_bestnav_ = TimedData<BestNavData>{*parsed->bestnav, received_at};
    }

    if (parsed->rtk_status.has_value())
    {
      latest_rtk_status_ = TimedData<RtkStatusData>{*parsed->rtk_status, received_at};
      last_rtkstatus_time_ = received_at;
    }

    if (parsed->rtcm_status.has_value())
    {
      latest_rtcm_status_ = TimedData<RtcmStatusData>{*parsed->rtcm_status, received_at};
      last_rtcmstatus_time_ = received_at;
      if (parsed->rtcm_status->message_id >= 0)
      {
        recent_rtcm_message_ids_.push_back(parsed->rtcm_status->message_id);
        if (recent_rtcm_message_ids_.size() > 8U)
        {
          recent_rtcm_message_ids_.pop_front();
        }
      }
    }

    if (parsed->bestsat.has_value())
    {
      latest_bestsat_ = TimedData<BestSatData>{*parsed->bestsat, received_at};
    }

    if (parsed->satsinfo.has_value())
    {
      latest_satsinfo_ = TimedData<SatsInfoData>{*parsed->satsinfo, received_at};
    }

    if (parsed->agc.has_value())
    {
      latest_agc_ = TimedData<AgcData>{*parsed->agc, received_at};
    }

    if (parsed->hw_status.has_value())
    {
      latest_hw_status_ = TimedData<HwStatusData>{*parsed->hw_status, received_at};
    }

    if (parsed->jam_status.has_value())
    {
      latest_jam_status_ = TimedData<JamStatusData>{*parsed->jam_status, received_at};
    }

    if (parsed->freq_jam_status.has_value())
    {
      latest_freq_jam_status_ = TimedData<FreqJamStatusData>{*parsed->freq_jam_status, received_at};
    }

    if (parsed->gsv.has_value())
    {
      // Per-constellation satellite-in-view tally. Talker prefix
      // ("GP", "GL", "GA", "GB", "GQ", "GI", "GN") is the constellation
      // key. We overwrite — total_in_view is identical across every
      // fragment of a GSV burst, so the latest write is authoritative.
      gsv_counts_[parsed->gsv->talker] =
          TimedData<int>{parsed->gsv->satellites_in_view, received_at};
    }
  }

  bool is_fresh(const SteadyTime& stamp) const
  {
    return is_fresh(stamp, data_timeout_sec_);
  }

  bool is_fresh(const SteadyTime& stamp, double timeout_sec) const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp).count() <=
           timeout_sec;
  }

  double age_seconds(const SteadyTime& stamp) const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp).count();
  }

  std::optional<BestNavData> active_bestnav() const
  {
    if (latest_bestnav_.has_value() &&
        is_fresh(latest_bestnav_->received_at, std::max(2.0, rtcm_timeout_sec_)))
    {
      return latest_bestnav_->data;
    }
    return std::nullopt;
  }

  std::optional<RtkStatusData> active_rtk_status() const
  {
    if (latest_rtk_status_.has_value() &&
        is_fresh(latest_rtk_status_->received_at, rtcm_timeout_sec_))
    {
      return latest_rtk_status_->data;
    }
    return std::nullopt;
  }

  std::optional<RtcmStatusData> active_rtcm_status() const
  {
    if (latest_rtcm_status_.has_value() &&
        is_fresh(latest_rtcm_status_->received_at, rtcm_timeout_sec_))
    {
      return latest_rtcm_status_->data;
    }
    return std::nullopt;
  }

  std::optional<BestSatData> active_bestsat() const
  {
    if (latest_bestsat_.has_value() &&
        is_fresh(latest_bestsat_->received_at, satellite_diag_timeout_sec_))
    {
      return latest_bestsat_->data;
    }
    return std::nullopt;
  }

  std::optional<SatsInfoData> active_satsinfo() const
  {
    if (latest_satsinfo_.has_value() &&
        is_fresh(latest_satsinfo_->received_at, satellite_diag_timeout_sec_))
    {
      return latest_satsinfo_->data;
    }
    return std::nullopt;
  }

  std::optional<AgcData> active_agc() const
  {
    if (latest_agc_.has_value() && is_fresh(latest_agc_->received_at, rf_diag_timeout_sec_))
    {
      return latest_agc_->data;
    }
    return std::nullopt;
  }

  std::optional<HwStatusData> active_hw_status() const
  {
    if (latest_hw_status_.has_value() &&
        is_fresh(latest_hw_status_->received_at, rf_diag_timeout_sec_))
    {
      return latest_hw_status_->data;
    }
    return std::nullopt;
  }

  std::optional<JamStatusData> active_jam_status() const
  {
    if (latest_jam_status_.has_value() &&
        is_fresh(latest_jam_status_->received_at, rf_diag_timeout_sec_))
    {
      return latest_jam_status_->data;
    }
    return std::nullopt;
  }

  std::optional<FreqJamStatusData> active_freq_jam_status() const
  {
    if (latest_freq_jam_status_.has_value() &&
        is_fresh(latest_freq_jam_status_->received_at, rf_diag_timeout_sec_))
    {
      return latest_freq_jam_status_->data;
    }
    return std::nullopt;
  }

  std::optional<FixData> active_fix() const
  {
    // Keep NavSatFix publication tied to the validated position streams
    // from PR1: PVTSLNA first, then GGA as a fallback.
    if (latest_pvtslna_fix_.has_value() && latest_pvtslna_fix_->data.valid_fix &&
        is_fresh(latest_pvtslna_fix_->received_at))
    {
      return latest_pvtslna_fix_->data;
    }
    if (latest_gga_fix_.has_value() && latest_gga_fix_->data.valid_fix &&
        is_fresh(latest_gga_fix_->received_at))
    {
      return latest_gga_fix_->data;
    }
    return std::nullopt;
  }

  std::optional<HeadingData> active_heading() const
  {
    if (latest_hpr_heading_.has_value() && is_fresh(latest_hpr_heading_->received_at))
    {
      return latest_hpr_heading_->data;
    }
    if (latest_hdt_heading_.has_value() && is_fresh(latest_hdt_heading_->received_at))
    {
      return latest_hdt_heading_->data;
    }
    return std::nullopt;
  }

  sensor_msgs::msg::NavSatStatus build_nav_status(const FixData& fix) const
  {
    sensor_msgs::msg::NavSatStatus status;
    status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
                     sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
                     sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS |
                     sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;

    int quality = fix.fix_quality;
    if (quality <= 0 && latest_gga_fix_.has_value() && is_fresh(latest_gga_fix_->received_at))
    {
      quality = latest_gga_fix_->data.fix_quality;
    }

    switch (quality)
    {
      case 0:
        status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        break;
      case 1:
        status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        break;
      case 2:
      case 9:
        status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        break;
      case 4:
      case 5:
        status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        break;
      default:
        status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        break;
    }

    return status;
  }

  void publish_active_fix()
  {
    const auto fix = active_fix();
    if (!fix.has_value())
    {
      return;
    }

    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.status = build_nav_status(*fix);
    msg.latitude = fix->latitude_deg;
    msg.longitude = fix->longitude_deg;
    msg.altitude = fix->altitude_m;

    if (fix->has_covariance)
    {
      msg.position_covariance = fix->covariance;
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    }
    else
    {
      msg.position_covariance.fill(0.0);
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }

    fix_pub_->publish(msg);
  }

  void publish_active_heading()
  {
    const auto heading = active_heading();
    if (!heading.has_value())
    {
      return;
    }

    compass_msgs::msg::Azimuth msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.azimuth = heading->heading_deg;
    msg.variance = heading->variance_deg2;
    msg.unit = compass_msgs::msg::Azimuth::UNIT_DEG;
    msg.orientation = compass_msgs::msg::Azimuth::ORIENTATION_NED;
    msg.reference = compass_msgs::msg::Azimuth::REFERENCE_GEOGRAPHIC;
    heading_pub_->publish(msg);
  }

  // Build the "GPS: fix" DiagnosticStatus matching the GUI conventions
  // (gpsHealth["GPS: fix"] in DiagnosticsPage.tsx). Carrier-solution
  // label (none/float/fixed) is unambiguous — unlike the raw NMEA
  // GGA quality field, where 4=Fixed and 5=Float (a common confusion).
  diagnostic_msgs::msg::DiagnosticStatus gps_fix_status(const std::optional<FixData>& fix) const
  {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "GPS: fix";
    s.hardware_id = "unicore_um982";

    const auto bestnav = active_bestnav();
    const int raw_quality = fix.has_value() ? fix->fix_quality : 0;
    const int q = raw_quality > 0 ? raw_quality : (bestnav.has_value() ? bestnav->fix_quality : 0);
    const char* carr_soln = carrier_solution_from_quality(q);
    const char* fix_type = fix_type_from_quality(q);

    double sigma_xy_mm = -1.0;
    if (fix.has_value() && fix->has_covariance)
    {
      const double cxx = fix->covariance[0];
      const double cyy = fix->covariance[4];
      sigma_xy_mm = std::sqrt(std::max(0.0, cxx + cyy)) * 1000.0;
    }

    s.values.push_back(kv("carr_soln", carr_soln));
    s.values.push_back(kv("fix_type", fix_type));
    s.values.push_back(kv("gps_fix_ok", q > 0 ? "True" : "False"));
    s.values.push_back(kv("diff_corr", q >= 2 ? "True" : "False"));
    s.values.push_back(kv("fix_quality", std::to_string(q)));
    s.values.push_back(kv("sigma_xy_mm",
                          sigma_xy_mm >= 0.0 ? to_string_or_nan(sigma_xy_mm) : "n/a"));
    if (fix.has_value())
    {
      s.values.push_back(kv("latitude_deg", to_string_or_nan(fix->latitude_deg)));
      s.values.push_back(kv("longitude_deg", to_string_or_nan(fix->longitude_deg)));
      s.values.push_back(kv("altitude_m", to_string_or_nan(fix->altitude_m)));
      s.values.push_back(kv("fix_source", fix_source_name(fix->source)));
    }
    if (bestnav.has_value())
    {
      s.values.push_back(kv("solution_status", bestnav->solution_status));
      s.values.push_back(kv("position_type", bestnav->position_type));
      s.values.push_back(kv("diff_age_s", to_string_or_nan(bestnav->diff_age_sec)));
      s.values.push_back(kv("sol_age_s", to_string_or_nan(bestnav->sol_age_sec)));
      s.values.push_back(kv("tracked_svs", std::to_string(bestnav->satellites_tracked)));
      s.values.push_back(kv("soln_svs", std::to_string(bestnav->satellites_used)));
      s.values.push_back(
          kv("ext_solution_status", to_hex_byte(bestnav->extended_solution_status)));
      s.values.push_back(kv("ext_solution_detail",
                            describe_ext_solution_status(bestnav->extended_solution_status)));
    }

    if (!serial_.is_open())
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      s.message = "serial disconnected";
    }
    else if (!fix.has_value() || q <= 0)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      s.message = "no fix";
    }
    else if (q == 4)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = "RTK Fixed";
    }
    else if (q == 5)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "RTK Float — converging, not yet validated";
    }
    else
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = std::string(fix_type) + " fix, no RTK";
    }
    return s;
  }

  diagnostic_msgs::msg::DiagnosticStatus gps_satellites_status() const
  {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "GPS: satellites";
    s.hardware_id = "unicore_um982";
    const auto bestnav = active_bestnav();
    const auto bestsat = active_bestsat();
    const auto satsinfo = active_satsinfo();
    const double bestsat_age = latest_bestsat_.has_value()
                                   ? age_seconds(latest_bestsat_->received_at)
                                   : std::numeric_limits<double>::infinity();
    const double satsinfo_age = latest_satsinfo_.has_value()
                                    ? age_seconds(latest_satsinfo_->received_at)
                                    : std::numeric_limits<double>::infinity();

    static const std::array<std::string, 5> kPrimaryConstellations = {
        "GPS", "GLO", "GAL", "BDS", "QZSS"};
    static const std::array<std::string, 8> kTrackedBands = {
        "L1", "L2", "L5", "E1", "E5", "B1", "B2", "B3"};
    static const std::unordered_map<std::string, std::string> kTalkerNames = {
        {"GP", "GPS"}, {"GL", "GLO"}, {"GA", "GAL"}, {"GB", "BDS"},
        {"GQ", "QZSS"}, {"GI", "IRNSS"}, {"GN", "GNSS"}};

    std::unordered_map<std::string, int> visible_by_constellation;
    std::unordered_map<std::string, int> used_by_constellation;
    std::unordered_map<std::string, Cn0Accumulator> cn0_by_constellation;
    std::unordered_map<std::string, int> signal_count_by_band;
    Cn0Accumulator cn0_all;

    int total = 0;
    for (const auto& [talker, timed] : gsv_counts_)
    {
      const int v = is_fresh(timed.received_at) ? timed.data : 0;
      total += v;
      const auto it = kTalkerNames.find(talker);
      const std::string label = it != kTalkerNames.end() ? it->second : talker;
      visible_by_constellation[label] = v;
      s.values.push_back(kv("sats_" + talker, std::to_string(v)));
    }

    int visible_total = total;
    if (enable_satellite_status_ && enable_satsinfo_ && satsinfo.has_value())
    {
      visible_total = 0;
      visible_by_constellation.clear();
      for (const auto& entry : satsinfo->entries)
      {
        ++visible_total;
        ++visible_by_constellation[entry.constellation];
        for (const auto& signal : entry.signals)
        {
          add_cn0_sample(cn0_all, signal.cn0_db_hz);
          add_cn0_sample(cn0_by_constellation[signal.constellation], signal.cn0_db_hz);
          if (!signal.band.empty() && signal.cn0_db_hz > 0.0)
          {
            ++signal_count_by_band[signal.band];
          }
        }
      }
    }

    int used_total = bestnav.has_value() ? bestnav->satellites_used : total;
    if (enable_satellite_status_ && bestsat.has_value())
    {
      used_total = static_cast<int>(bestsat->entries.size());
      used_by_constellation.clear();
      for (const auto& entry : bestsat->entries)
      {
        ++used_by_constellation[entry.constellation];
      }
    }

    std::string constellation_summary;
    for (const auto& name : kPrimaryConstellations)
    {
      const int visible_count = visible_by_constellation[name];
      const int used_count = used_by_constellation[name];
      if (visible_count <= 0 && used_count <= 0)
      {
        continue;
      }
      if (!constellation_summary.empty())
      {
        constellation_summary += ", ";
      }
      constellation_summary += name + "=" + std::to_string(visible_count) + "/" +
                               std::to_string(used_count);
    }

    const int tracked = bestnav.has_value() ? bestnav->satellites_tracked : visible_total;
    s.values.push_back(kv("satellite_status_enabled", enable_satellite_status_ ? "True" : "False"));
    s.values.push_back(kv("satsinfo_enabled", enable_satsinfo_ ? "True" : "False"));
    s.values.push_back(kv("bestsat_available", bestsat.has_value() ? "True" : "False"));
    s.values.push_back(kv("satsinfo_available", satsinfo.has_value() ? "True" : "False"));
    s.values.push_back(
        kv("last_bestsat_age_s", std::isfinite(bestsat_age) ? to_string_or_nan(bestsat_age) : "inf"));
    s.values.push_back(kv("last_satsinfo_age_s",
                          std::isfinite(satsinfo_age) ? to_string_or_nan(satsinfo_age) : "inf"));
    s.values.push_back(kv("visible", std::to_string(visible_total)));
    s.values.push_back(kv("tracked", std::to_string(tracked)));
    s.values.push_back(kv("used", std::to_string(used_total)));
    s.values.push_back(kv("constellations_used",
                          constellation_summary.empty() ? "n/a" : constellation_summary));
    s.values.push_back(kv("cn0_mean_db_hz", mean_cn0_or_na(cn0_all)));
    s.values.push_back(kv("cn0_max_db_hz", max_cn0_or_na(cn0_all)));

    for (const auto& name : kPrimaryConstellations)
    {
      s.values.push_back(kv("visible_" + name, std::to_string(visible_by_constellation[name])));
      s.values.push_back(kv("used_" + name, std::to_string(used_by_constellation[name])));
      s.values.push_back(kv("cn0_mean_" + name + "_db_hz", mean_cn0_or_na(cn0_by_constellation[name])));
    }

    for (const auto& band : kTrackedBands)
    {
      s.values.push_back(kv("tracked_signals_" + band,
                            std::to_string(signal_count_by_band[band])));
    }

    if (!enable_satellite_status_)
    {
      if (gsv_counts_.empty() || total == 0)
      {
        s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        s.message = "advanced satellite diagnostics disabled; waiting for GSV fallback";
      }
      else
      {
        s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        s.message = std::to_string(total) + " sats via GSV fallback";
      }
    }
    else if (enable_satsinfo_ && !satsinfo.has_value() && gsv_counts_.empty())
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      s.message = "SATSINFOA missing and no GSV fallback";
    }
    else if (enable_satsinfo_ && !satsinfo.has_value())
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "SATSINFOA stale or missing";
    }
    else if (visible_total == 0)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      s.message = "no visible satellites";
    }
    else if (!bestsat.has_value())
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "BESTSATA stale or missing";
    }
    else if (bestsat.has_value() && used_total == 0)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "satellites visible but none used";
    }
    else if (visible_total < 6)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = std::to_string(visible_total) + " visible sats — weak sky view";
    }
    else
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = std::to_string(visible_total) + " visible / " + std::to_string(used_total) +
                  " used";
    }
    return s;
  }

  diagnostic_msgs::msg::DiagnosticStatus gps_rtk_status() const
  {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "GPS: RTK";
    s.hardware_id = "unicore_um982";

    const auto bestnav = active_bestnav();
    const auto rtk_status = active_rtk_status();
    const double bestnav_age = latest_bestnav_.has_value() ? age_seconds(latest_bestnav_->received_at)
                                                           : std::numeric_limits<double>::infinity();
    const double rtkstatus_age =
        last_rtkstatus_time_.has_value() ? age_seconds(*last_rtkstatus_time_)
                                         : std::numeric_limits<double>::infinity();
    const int quality = bestnav.has_value() ? bestnav->fix_quality
                                            : (rtk_status.has_value() ? rtk_status->fix_quality : 0);

    s.values.push_back(kv("status_enabled", enable_rtk_status_ ? "True" : "False"));
    s.values.push_back(
        kv("last_bestnav_age_s", std::isfinite(bestnav_age) ? to_string_or_nan(bestnav_age) : "inf"));
    s.values.push_back(kv("last_rtkstatus_age_s",
                          std::isfinite(rtkstatus_age) ? to_string_or_nan(rtkstatus_age) : "inf"));
    s.values.push_back(kv("fix_type", fix_type_from_quality(quality)));

    if (bestnav.has_value())
    {
      s.values.push_back(kv("solution_status", bestnav->solution_status));
      s.values.push_back(kv("position_type", bestnav->position_type));
      s.values.push_back(kv("diff_age_s", to_string_or_nan(bestnav->diff_age_sec)));
      s.values.push_back(kv("sol_age_s", to_string_or_nan(bestnav->sol_age_sec)));
      s.values.push_back(kv("tracked_svs", std::to_string(bestnav->satellites_tracked)));
      s.values.push_back(kv("soln_svs", std::to_string(bestnav->satellites_used)));
      s.values.push_back(
          kv("signal_mask_gal_bds3", to_hex_byte(bestnav->galileo_bds3_signal_mask)));
      s.values.push_back(kv("signal_mask_gps_glo_bds2",
                            to_hex_byte(bestnav->gps_glonass_bds2_signal_mask)));
      s.values.push_back(kv("signals_used_gal_bds3",
                            describe_galileo_bds3_signal_mask(bestnav->galileo_bds3_signal_mask)));
      s.values.push_back(
          kv("signals_used_gps_glo_bds2",
             describe_gps_glo_bds2_signal_mask(bestnav->gps_glonass_bds2_signal_mask)));
      s.values.push_back(
          kv("ext_solution_status", to_hex_byte(bestnav->extended_solution_status)));
      s.values.push_back(kv("ext_solution_detail",
                            describe_ext_solution_status(bestnav->extended_solution_status)));
    }

    if (rtk_status.has_value())
    {
      const std::size_t gps_corr = count_bits(rtk_status->gps_source_mask);
      const std::size_t bds_corr = count_bits(rtk_status->bds_source_mask_1) +
                                   count_bits(rtk_status->bds_source_mask_2);
      const std::size_t glo_corr = count_bits(rtk_status->glonass_source_mask);
      const std::size_t gal_corr = count_bits(rtk_status->galileo_source_mask_1) +
                                   count_bits(rtk_status->galileo_source_mask_2);
      const std::size_t qzss_corr = count_bits(rtk_status->qzss_source_mask);

      s.values.push_back(kv("rtkstatus_position_type", rtk_status->position_type));
      s.values.push_back(
          kv("rtk_calculate_status", describe_rtk_calculate_status(rtk_status->calculate_status)));
      s.values.push_back(kv("adr_observations", std::to_string(rtk_status->adr_observation_count)));
      s.values.push_back(kv("ion_detected", std::to_string(rtk_status->ion_detected)));
      s.values.push_back(kv("dual_rtk_flag", describe_dual_rtk_flag(rtk_status->dual_rtk_flag)));
      s.values.push_back(kv("gps_corr_sats", std::to_string(gps_corr)));
      s.values.push_back(kv("bds_corr_sats", std::to_string(bds_corr)));
      s.values.push_back(kv("glo_corr_sats", std::to_string(glo_corr)));
      s.values.push_back(kv("gal_corr_sats", std::to_string(gal_corr)));
      s.values.push_back(kv("qzss_corr_sats", std::to_string(qzss_corr)));
      s.values.push_back(kv("gps_source_mask", to_hex_word(rtk_status->gps_source_mask)));
      s.values.push_back(kv("bds_source_mask_1", to_hex_word(rtk_status->bds_source_mask_1)));
      s.values.push_back(kv("bds_source_mask_2", to_hex_word(rtk_status->bds_source_mask_2)));
      s.values.push_back(
          kv("glonass_source_mask", to_hex_word(rtk_status->glonass_source_mask)));
      s.values.push_back(
          kv("galileo_source_mask_1", to_hex_word(rtk_status->galileo_source_mask_1)));
      s.values.push_back(
          kv("galileo_source_mask_2", to_hex_word(rtk_status->galileo_source_mask_2)));
      s.values.push_back(kv("qzss_source_mask", to_hex_word(rtk_status->qzss_source_mask)));
    }

    if (!enable_rtk_status_)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = "RTK status diagnostics disabled";
    }
    else if (!bestnav.has_value())
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      s.message = "BESTNAVA stale or missing";
    }
    else if (!rtk_status.has_value())
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "RTKSTATUSA stale or missing";
    }
    else if (bestnav->solution_status != "SOL_COMPUTED")
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      s.message = "solution not computed";
    }
    else if (bestnav->diff_age_sec > max_diff_age_sec_)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "RTCM corrections too old";
    }
    else if (quality == 4)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = "RTK fixed";
    }
    else if (quality == 5)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "RTK float";
    }
    else if (quality > 0)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "non-RTK solution";
    }
    else
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      s.message = "no valid RTK solution";
    }

    return s;
  }

  diagnostic_msgs::msg::DiagnosticStatus gps_ntrip_status()
  {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "GPS: NTRIP/RTCM";
    s.hardware_id = "unicore_um982";

    constexpr double window_s = 5.0;
    const auto now_t = std::chrono::steady_clock::now();
    while (!rtcm_history_.empty() &&
           std::chrono::duration<double>(now_t - rtcm_history_.front()).count() > window_s)
    {
      rtcm_history_.pop_front();
    }
    const std::size_t n = rtcm_history_.size();
    const double rate = static_cast<double>(n) / window_s;
    const double injection_age =
        rtcm_history_.empty()
            ? std::numeric_limits<double>::infinity()
            : std::chrono::duration<double>(now_t - rtcm_history_.back()).count();
    const auto rtcm_status = active_rtcm_status();
    const double receiver_age = last_rtcmstatus_time_.has_value()
                                    ? age_seconds(*last_rtcmstatus_time_)
                                    : std::numeric_limits<double>::infinity();

    s.values.push_back(kv("msgs_per_sec", to_string_or_nan(rate)));
    s.values.push_back(
        kv("age_of_last_injected_corr_s",
           std::isfinite(injection_age) ? to_string_or_nan(injection_age) : "inf"));
    s.values.push_back(
        kv("age_of_last_rtcmstatus_s",
           std::isfinite(receiver_age) ? to_string_or_nan(receiver_age) : "inf"));
    s.values.push_back(kv("rtcm_messages_total", std::to_string(rtcm_message_count_)));
    s.values.push_back(kv("rtcm_bytes_total", std::to_string(rtcm_byte_count_)));
    s.values.push_back(kv("rtcm_status_enabled", enable_rtcm_status_ ? "True" : "False"));

    std::string recent_types = "n/a";
    if (!recent_rtcm_message_ids_.empty())
    {
      std::vector<std::string> ids;
      ids.reserve(recent_rtcm_message_ids_.size());
      for (const int message_id : recent_rtcm_message_ids_)
      {
        ids.emplace_back(std::to_string(message_id));
      }
      recent_types = join_strings(ids);
    }
    s.values.push_back(kv("recent_rtcm_types", recent_types));

    if (rtcm_status.has_value())
    {
      s.values.push_back(kv("last_rtcm_msg_id", std::to_string(rtcm_status->message_id)));
      s.values.push_back(kv("last_rtcm_counter", std::to_string(rtcm_status->message_count)));
      s.values.push_back(kv("last_rtcm_base_id", std::to_string(rtcm_status->base_station_id)));
      s.values.push_back(kv("last_rtcm_satellites", std::to_string(rtcm_status->satellite_count)));
      s.values.push_back(kv("last_rtcm_l1", std::to_string(rtcm_status->observable_count[0])));
      s.values.push_back(kv("last_rtcm_l2", std::to_string(rtcm_status->observable_count[1])));
      s.values.push_back(kv("last_rtcm_l3", std::to_string(rtcm_status->observable_count[2])));
      s.values.push_back(kv("last_rtcm_l4", std::to_string(rtcm_status->observable_count[3])));
      s.values.push_back(kv("last_rtcm_l5", std::to_string(rtcm_status->observable_count[4])));
      s.values.push_back(kv("last_rtcm_l6", std::to_string(rtcm_status->observable_count[5])));
    }

    if (rtcm_history_.empty())
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      s.message = "no RTCM in last 5 s";
    }
    else if (injection_age > rtcm_timeout_sec_)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "RTCM injection stale";
    }
    else if (enable_rtcm_status_ && !rtcm_status.has_value())
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "waiting for RTCMSTATUSA";
    }
    else if (enable_rtcm_status_ && receiver_age > rtcm_timeout_sec_)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "RTCMSTATUSA stale";
    }
    else
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = to_string_or_nan(rate) + " msg/s, last injection " +
                  to_string_or_nan(injection_age) + " s ago";
    }
    return s;
  }

  diagnostic_msgs::msg::DiagnosticStatus gps_rf_status() const
  {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "GPS: rf";
    s.hardware_id = "unicore_um982";

    const auto agc = active_agc();
    const double agc_age = latest_agc_.has_value() ? age_seconds(latest_agc_->received_at)
                                                   : std::numeric_limits<double>::infinity();
    const double main_mean = agc.has_value() ? mean_valid_agc(agc->antenna1)
                                             : std::numeric_limits<double>::quiet_NaN();
    const double aux_mean = agc.has_value() ? mean_valid_agc(agc->antenna2)
                                            : std::numeric_limits<double>::quiet_NaN();
    const int main_min = agc.has_value() ? min_valid_agc(agc->antenna1) : -1;

    // Heuristic only: the manual says weak/open-circuit conditions drive
    // AGC upward, while interference/noise-floor rise drives AGC downward.
    // We therefore flag large master-antenna gain as "signal low" and very
    // small gain as a likely saturation/jamming symptom.
    const bool rf_signal_low = std::isfinite(main_mean) && main_mean >= 80.0;
    const bool rf_saturation_suspected = main_min >= 0 && main_min <= 10;

    s.values.push_back(kv("status_enabled", enable_rf_status_ ? "True" : "False"));
    s.values.push_back(kv("agc_available", agc.has_value() ? "True" : "False"));
    s.values.push_back(
        kv("last_agc_age_s", std::isfinite(agc_age) ? to_string_or_nan(agc_age) : "inf"));
    s.values.push_back(kv("agc_main", agc.has_value() ? describe_agc_values(agc->antenna1) : "n/a"));
    s.values.push_back(kv("agc_aux", agc.has_value() ? describe_agc_values(agc->antenna2) : "n/a"));
    s.values.push_back(
        kv("agc_main_mean", std::isfinite(main_mean) ? to_string_or_nan(main_mean) : "n/a"));
    s.values.push_back(
        kv("agc_aux_mean", std::isfinite(aux_mean) ? to_string_or_nan(aux_mean) : "n/a"));
    s.values.push_back(kv("rf_signal_low", rf_signal_low ? "True" : "False"));
    s.values.push_back(
        kv("rf_saturation_suspected", rf_saturation_suspected ? "True" : "False"));

    if (agc.has_value())
    {
      s.values.push_back(kv("agc_main_l1", std::to_string(agc->antenna1[0])));
      s.values.push_back(kv("agc_main_l2", std::to_string(agc->antenna1[1])));
      s.values.push_back(kv("agc_main_l5", std::to_string(agc->antenna1[2])));
      s.values.push_back(kv("agc_aux_l1", std::to_string(agc->antenna2[0])));
      s.values.push_back(kv("agc_aux_l2", std::to_string(agc->antenna2[1])));
      s.values.push_back(kv("agc_aux_l5", std::to_string(agc->antenna2[2])));
    }

    if (!enable_rf_status_)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = "RF diagnostics disabled";
    }
    else if (!agc.has_value())
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "AGCA stale or missing";
    }
    else if (rf_saturation_suspected)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "RF saturation/interference suspected";
    }
    else if (rf_signal_low)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "RF signal weak or antenna gain high";
    }
    else
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = "RF gain nominal";
    }
    return s;
  }

  diagnostic_msgs::msg::DiagnosticStatus gps_hardware_status() const
  {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "GPS: hardware";
    s.hardware_id = "unicore_um982";

    const auto hw = active_hw_status();
    const auto agc = active_agc();
    const double hw_age = latest_hw_status_.has_value()
                              ? age_seconds(latest_hw_status_->received_at)
                              : std::numeric_limits<double>::infinity();

    const bool dc09_ok = hw.has_value() && voltage_in_range(hw->dc09_v, 0.85, 1.00);
    const bool dc10_ok = hw.has_value() && voltage_in_range(hw->dc10_v, 0.95, 1.10);
    // N4 R1.4 documents DC18 as a 1.7-1.9 V rail, but the official
    // HWSTATUSA ASCII example itself reports 0.908 V. Until verified on
    // hardware, keep this check permissive and expose the raw voltage.
    const bool dc18_ok = hw.has_value() && std::isfinite(hw->dc18_v) && hw->dc18_v > 0.0;
    const bool voltages_ok = dc09_ok && dc10_ok && dc18_ok;
    const bool clock_ok = hw.has_value() && hw->clock_flag == 1;
    const bool pll_ok = hw.has_value() && hw->pll_lock != 0;
    const bool hardware_ok = hw.has_value() && voltages_ok && clock_ok && pll_ok;

    s.values.push_back(kv("status_enabled", enable_hw_status_ ? "True" : "False"));
    s.values.push_back(kv("hwstatus_available", hw.has_value() ? "True" : "False"));
    s.values.push_back(
        kv("last_hwstatus_age_s", std::isfinite(hw_age) ? to_string_or_nan(hw_age) : "inf"));
    s.values.push_back(kv("hardware_ok", hardware_ok ? "True" : "False"));
    s.values.push_back(kv("antenna_status", describe_antenna_status(agc)));

    if (hw.has_value())
    {
      s.values.push_back(kv("dc09_v", to_string_or_nan(hw->dc09_v)));
      s.values.push_back(kv("dc10_v", to_string_or_nan(hw->dc10_v)));
      s.values.push_back(kv("dc18_v", to_string_or_nan(hw->dc18_v)));
      s.values.push_back(kv("clock_drift_mps", to_string_or_nan(hw->clock_drift_mps)));
      s.values.push_back(kv("hw_flags", to_hex_byte(hw->hw_flag)));
      s.values.push_back(kv("hw_flags_detail", describe_hw_flag_bits(hw->hw_flag)));
      s.values.push_back(kv("clock_status", describe_clock_status(hw->clock_flag, hw->hw_flag)));
      s.values.push_back(kv("pll_status", describe_pll_status(hw->pll_lock)));
      s.values.push_back(kv("pll_lock_mask", to_hex_word(static_cast<uint32_t>(hw->pll_lock))));
    }
    else
    {
      s.values.push_back(kv("hw_flags", "n/a"));
      s.values.push_back(kv("clock_status", "n/a"));
      s.values.push_back(kv("pll_status", "n/a"));
    }

    if (!enable_hw_status_)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = "hardware diagnostics disabled";
    }
    else if (!hw.has_value())
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "HWSTATUSA stale or missing";
    }
    else if (!voltages_ok)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      s.message = "supply rail out of range";
    }
    else if (!clock_ok)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "clock drift invalid";
    }
    else if (!pll_ok)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "PLL lock missing";
    }
    else
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = "hardware healthy";
    }
    return s;
  }

  diagnostic_msgs::msg::DiagnosticStatus gps_jamming_status() const
  {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "GPS: jamming";
    s.hardware_id = "unicore_um982";

    const auto jam = active_jam_status();
    const auto freq_jam = active_freq_jam_status();
    const double jam_age = latest_jam_status_.has_value()
                               ? age_seconds(latest_jam_status_->received_at)
                               : std::numeric_limits<double>::infinity();
    const double freq_age = latest_freq_jam_status_.has_value()
                                ? age_seconds(latest_freq_jam_status_->received_at)
                                : std::numeric_limits<double>::infinity();

    int strongest_flag = jam.has_value() ? jam->cw_flag : -1;
    int strongest_ratio = jam.has_value() ? jam->cw_ratio : -1;
    std::vector<std::string> jammed_frequencies;
    static const std::array<std::string, 3> kJamBands = {"L1", "L2", "L5"};
    if (freq_jam.has_value())
    {
      for (std::size_t i = 0; i < kJamBands.size(); ++i)
      {
        strongest_flag = std::max(strongest_flag, freq_jam->cw_flag[i]);
        strongest_ratio = std::max(strongest_ratio, freq_jam->cw_ratio[i]);
        if (freq_jam->cw_flag[i] > 0)
        {
          jammed_frequencies.push_back(kJamBands[i]);
        }
      }
    }
    const bool jamming_detected = strongest_flag > 0;
    const std::string jammed_frequency_text =
        freq_jam.has_value() ? (jammed_frequencies.empty() ? "none" : join_strings(jammed_frequencies))
                             : "n/a";

    s.values.push_back(kv("status_enabled", enable_jamming_status_ ? "True" : "False"));
    s.values.push_back(kv("jamstatus_available", jam.has_value() ? "True" : "False"));
    s.values.push_back(
        kv("freqjamstatus_available", freq_jam.has_value() ? "True" : "False"));
    s.values.push_back(
        kv("last_jamstatus_age_s", std::isfinite(jam_age) ? to_string_or_nan(jam_age) : "inf"));
    s.values.push_back(kv("last_freqjamstatus_age_s",
                          std::isfinite(freq_age) ? to_string_or_nan(freq_age) : "inf"));
    s.values.push_back(kv("jamming_detected", jamming_detected ? "True" : "False"));
    s.values.push_back(kv("jam_level", describe_jam_flag(strongest_flag)));
    s.values.push_back(kv("jam_ratio_max", strongest_ratio >= 0 ? std::to_string(strongest_ratio) : "n/a"));
    s.values.push_back(kv("jammed_frequencies", jammed_frequency_text));

    if (jam.has_value())
    {
      s.values.push_back(kv("global_position_type", jam->position_type));
      s.values.push_back(kv("global_cw_ratio", std::to_string(jam->cw_ratio)));
      s.values.push_back(kv("global_cw_flag", describe_jam_flag(jam->cw_flag)));
    }

    if (freq_jam.has_value())
    {
      s.values.push_back(kv("jam_l1_ratio", std::to_string(freq_jam->cw_ratio[0])));
      s.values.push_back(kv("jam_l1_flag", describe_jam_flag(freq_jam->cw_flag[0])));
      s.values.push_back(kv("jam_l2_ratio", std::to_string(freq_jam->cw_ratio[1])));
      s.values.push_back(kv("jam_l2_flag", describe_jam_flag(freq_jam->cw_flag[1])));
      s.values.push_back(kv("jam_l5_ratio", std::to_string(freq_jam->cw_ratio[2])));
      s.values.push_back(kv("jam_l5_flag", describe_jam_flag(freq_jam->cw_flag[2])));
    }

    if (!enable_jamming_status_)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = "jamming diagnostics disabled";
    }
    else if (!jam.has_value() && !freq_jam.has_value())
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "JAMSTATUSA/FREQJAMSTATUSA stale or missing";
    }
    else if (strongest_flag >= 2)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      s.message = "strong jamming detected";
    }
    else if (strongest_flag == 1)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "jamming detected";
    }
    else
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = "no jamming detected";
    }
    return s;
  }

  diagnostic_msgs::msg::DiagnosticStatus gps_parser_status()
  {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "GPS: parser";
    s.hardware_id = "unicore_um982";

    const ParserCounters current = parser_.counters();
    const std::size_t delta_parse = current.parse_errors - parser_counters_snapshot_.parse_errors;
    const std::size_t delta_nmea_crc =
        current.nmea_checksum_errors - parser_counters_snapshot_.nmea_checksum_errors;
    const std::size_t delta_unicore_crc =
        current.unicore_crc_errors - parser_counters_snapshot_.unicore_crc_errors;

    s.values.push_back(kv("parsed_sentences_total", std::to_string(current.parsed_sentences)));
    s.values.push_back(kv("parse_errors_total", std::to_string(current.parse_errors)));
    s.values.push_back(
        kv("nmea_checksum_errors_total", std::to_string(current.nmea_checksum_errors)));
    s.values.push_back(
        kv("unicore_crc_errors_total", std::to_string(current.unicore_crc_errors)));
    s.values.push_back(kv("parse_errors_delta", std::to_string(delta_parse)));
    s.values.push_back(kv("nmea_checksum_errors_delta", std::to_string(delta_nmea_crc)));
    s.values.push_back(kv("unicore_crc_errors_delta", std::to_string(delta_unicore_crc)));

    parser_counters_snapshot_ = current;

    if (delta_parse > 0U || delta_nmea_crc > 0U || delta_unicore_crc > 0U)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "new parse/CRC errors observed";
    }
    else
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = "parser healthy";
    }
    return s;
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    const auto fix = active_fix();
    array.status.push_back(gps_fix_status(fix));
    array.status.push_back(gps_satellites_status());
    if (enable_rtk_status_)
    {
      array.status.push_back(gps_rtk_status());
    }
    array.status.push_back(gps_ntrip_status());
    if (enable_rf_status_)
    {
      array.status.push_back(gps_rf_status());
    }
    if (enable_hw_status_)
    {
      array.status.push_back(gps_hardware_status());
    }
    if (enable_jamming_status_)
    {
      array.status.push_back(gps_jamming_status());
    }
    array.status.push_back(gps_parser_status());
    diagnostics_pub_->publish(array);
  }

  void handle_rtcm(const rtcm_msgs::msg::Message::SharedPtr msg)
  {
    ensure_serial_open();
    if (!serial_.is_open())
    {
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           5000,
                           "Dropping RTCM message because serial port %s is not open",
                           port_.c_str());
      return;
    }

    if (msg->message.empty())
    {
      return;
    }

    const auto written = serial_.write(msg->message.data(), msg->message.size());
    if (written < 0)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(),
                            *get_clock(),
                            5000,
                            "Failed to write RTCM message to %s: %s",
                            port_.c_str(),
                            std::strerror(errno));
      serial_.close();
      return;
    }

    ++rtcm_message_count_;
    rtcm_byte_count_ += static_cast<std::size_t>(written);
    rtcm_history_.push_back(std::chrono::steady_clock::now());
    if (rtcm_history_.size() > 1024U)
    {
      rtcm_history_.pop_front();   // bound memory under sustained 100+ Hz RTCM
    }
  }

  std::string port_;
  int baudrate_{921600};
  std::string frame_id_;
  double data_timeout_sec_{1.0};
  double reconnect_interval_sec_{1.0};
  double read_poll_hz_{200.0};
  double rtcm_timeout_sec_{5.0};
  double max_diff_age_sec_{5.0};
  double satellite_diag_timeout_sec_{5.0};
  double rf_diag_timeout_sec_{5.0};
  bool enable_rtk_status_{true};
  bool enable_rtcm_status_{true};
  bool enable_satellite_status_{true};
  bool enable_satsinfo_{true};
  bool enable_rf_status_{true};
  bool enable_hw_status_{true};
  bool enable_jamming_status_{true};
  std::string fix_topic_;
  std::string heading_topic_;
  std::string diagnostics_topic_;
  std::string rtcm_topic_;

  SerialPort serial_;
  Um982Parser parser_;
  std::string rx_buffer_;
  std::optional<SteadyTime> last_open_attempt_;

  std::optional<TimedData<FixData>> latest_gga_fix_;
  std::optional<TimedData<FixData>> latest_pvtslna_fix_;
  std::optional<TimedData<BestNavData>> latest_bestnav_;
  std::optional<TimedData<HeadingData>> latest_hdt_heading_;
  std::optional<TimedData<HeadingData>> latest_hpr_heading_;
  std::optional<TimedData<VelocityData>> latest_velocity_;
  std::optional<TimedData<RtkStatusData>> latest_rtk_status_;
  std::optional<TimedData<RtcmStatusData>> latest_rtcm_status_;
  std::optional<TimedData<BestSatData>> latest_bestsat_;
  std::optional<TimedData<SatsInfoData>> latest_satsinfo_;
  std::optional<TimedData<AgcData>> latest_agc_;
  std::optional<TimedData<HwStatusData>> latest_hw_status_;
  std::optional<TimedData<JamStatusData>> latest_jam_status_;
  std::optional<TimedData<FreqJamStatusData>> latest_freq_jam_status_;
  std::optional<SteadyTime> last_rtkstatus_time_;
  std::optional<SteadyTime> last_rtcmstatus_time_;

  std::unordered_map<std::string, std::size_t> sentence_counts_;
  std::unordered_map<std::string, TimedData<int>> gsv_counts_;
  std::size_t rtcm_message_count_{0U};
  std::size_t rtcm_byte_count_{0U};
  // Sliding window of recent RTCM injection timestamps for the
  // GPS: NTRIP/RTCM diagnostic. Bounded to ~1024 entries so a
  // chatty caster can't unbounded-grow the deque.
  std::deque<std::chrono::steady_clock::time_point> rtcm_history_;
  std::deque<int> recent_rtcm_message_ids_;
  ParserCounters parser_counters_snapshot_{};

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<compass_msgs::msg::Azimuth>::SharedPtr heading_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace mowgli_unicore_gnss

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_unicore_gnss::Um982Node>());
  rclcpp::shutdown();
  return 0;
}
