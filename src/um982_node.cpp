// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
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
                "UM982 node configured: port=%s baudrate=%d fix_topic=%s heading_topic=%s",
                port_.c_str(),
                baudrate_,
                fix_topic_.c_str(),
                heading_topic_.c_str());
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
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp).count() <=
           data_timeout_sec_;
  }

  std::optional<FixData> active_fix() const
  {
    if (latest_pvtslna_fix_.has_value() && is_fresh(latest_pvtslna_fix_->received_at))
    {
      // PVTSLNA position + covariance is what we want, but its
      // position-type string isn't always recognised by
      // position_type_to_gga_quality() on every firmware revision —
      // when that happens fix_quality lands at 0 (NONE) even though
      // the receiver clearly has a fix. Graft GGA's quality onto the
      // PVTSLNA fix so downstream NavSatStatus and the carr_soln
      // diagnostic both see the right value.
      FixData out = latest_pvtslna_fix_->data;
      if (out.fix_quality <= 0 && latest_gga_fix_.has_value() &&
          is_fresh(latest_gga_fix_->received_at) &&
          latest_gga_fix_->data.fix_quality > 0)
      {
        out.fix_quality = latest_gga_fix_->data.fix_quality;
        out.valid_fix = true;
      }
      return out;
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
      case 5:
        status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        break;
      case 4:
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

    const int q = fix.has_value() ? fix->fix_quality : 0;
    const char* carr_soln = "none";
    const char* fix_type = "no-fix";
    if (q == 4) { carr_soln = "fixed"; fix_type = "3D-RTK-Fixed"; }
    else if (q == 5) { carr_soln = "float"; fix_type = "3D-RTK-Float"; }
    else if (q == 2 || q == 9) { fix_type = "3D-DGPS"; }
    else if (q == 1) { fix_type = "3D"; }

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

    int total = 0;
    std::string per_const;
    static const std::unordered_map<std::string, std::string> kTalkerNames = {
        {"GP", "GPS"}, {"GL", "GLO"}, {"GA", "GAL"}, {"GB", "BDS"},
        {"GQ", "QZSS"}, {"GI", "NavIC"}, {"GN", "GNSS"}};
    for (const auto& [talker, timed] : gsv_counts_)
    {
      const int v = is_fresh(timed.received_at) ? timed.data : 0;
      total += v;
      const auto it = kTalkerNames.find(talker);
      const std::string label = it != kTalkerNames.end() ? it->second : talker;
      if (!per_const.empty()) per_const += ", ";
      per_const += label + "=" + std::to_string(v);
      s.values.push_back(kv("sats_" + talker, std::to_string(v)));
    }
    s.values.push_back(kv("visible", std::to_string(total)));
    s.values.push_back(kv("used", std::to_string(total)));
    s.values.push_back(kv("constellations_used", per_const));
    s.values.push_back(kv("mean_cno_db_hz", "n/a"));   // UM982 doesn't expose per-sat CN0
    s.values.push_back(kv("cno_ge_40_count", "n/a"));

    if (gsv_counts_.empty() || total == 0)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      s.message = "no satellites — receiver dead or GSV not enabled";
    }
    else if (total < 6)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = std::to_string(total) + " sats — weak for RTK";
    }
    else
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = std::to_string(total) + " sats (" + per_const + ")";
    }
    return s;
  }

  diagnostic_msgs::msg::DiagnosticStatus gps_ntrip_status()
  {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "GPS: NTRIP/RTCM";
    s.hardware_id = "ntrip_client";

    constexpr double window_s = 5.0;
    const auto now_t = std::chrono::steady_clock::now();
    while (!rtcm_history_.empty() &&
           std::chrono::duration<double>(now_t - rtcm_history_.front()).count() > window_s)
    {
      rtcm_history_.pop_front();
    }
    const std::size_t n = rtcm_history_.size();
    const double rate = static_cast<double>(n) / window_s;
    const double age =
        rtcm_history_.empty()
            ? std::numeric_limits<double>::infinity()
            : std::chrono::duration<double>(now_t - rtcm_history_.back()).count();

    s.values.push_back(kv("msgs_per_sec", to_string_or_nan(rate)));
    s.values.push_back(
        kv("age_of_last_corr_s", std::isfinite(age) ? to_string_or_nan(age) : "inf"));
    s.values.push_back(kv("rtcm_messages_total", std::to_string(rtcm_message_count_)));
    s.values.push_back(kv("rtcm_bytes_total", std::to_string(rtcm_byte_count_)));

    if (rtcm_history_.empty())
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      s.message = "no RTCM in last 5 s";
    }
    else if (age > 2.0)
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      s.message = "RTCM stalling (age " + to_string_or_nan(age) + " s)";
    }
    else
    {
      s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = to_string_or_nan(rate) + " msg/s, last " + to_string_or_nan(age) + " s ago";
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
    array.status.push_back(gps_ntrip_status());
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
  std::optional<TimedData<HeadingData>> latest_hdt_heading_;
  std::optional<TimedData<HeadingData>> latest_hpr_heading_;
  std::optional<TimedData<VelocityData>> latest_velocity_;

  std::unordered_map<std::string, std::size_t> sentence_counts_;
  std::unordered_map<std::string, TimedData<int>> gsv_counts_;
  std::size_t rtcm_message_count_{0U};
  std::size_t rtcm_byte_count_{0U};
  // Sliding window of recent RTCM injection timestamps for the
  // GPS: NTRIP/RTCM diagnostic. Bounded to ~1024 entries so a
  // chatty caster can't unbounded-grow the deque.
  std::deque<std::chrono::steady_clock::time_point> rtcm_history_;

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
