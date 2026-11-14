#pragma once

// ROS-free building blocks of the QoS experiment: the probe payload layout and
// the receive-side accounting. Kept free of rclcpp so they are unit tested
// without a ROS graph.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace qos_lab {

/// Header written into the first bytes of every probe message. The rest of the
/// payload is filler, so the message size is chosen independently.
struct ProbeHeader {
  std::uint64_t sequence{0};     ///< 0, 1, 2, ... per publisher run
  std::int64_t send_time_ns{0};  ///< publisher clock at publish, ns since epoch
};

inline constexpr std::size_t kProbeHeaderSize = sizeof(std::uint64_t) + sizeof(std::int64_t);

/// Payload of `payload_size` bytes (at least kProbeHeaderSize) carrying `header`
/// little-endian in its first 16 bytes. Returns std::nullopt if the size is
/// too small.
[[nodiscard]] std::optional<std::vector<std::uint8_t>> encodeProbe(const ProbeHeader& header,
                                                                   std::size_t payload_size);

/// Overwrites the header of an existing payload in place (no allocation on the
/// publish path). Returns false if the payload is too small.
[[nodiscard]] bool writeProbeHeader(const ProbeHeader& header, std::vector<std::uint8_t>& payload);

/// Reads the header back. Returns std::nullopt if the payload is too small.
[[nodiscard]] std::optional<ProbeHeader> decodeProbe(const std::vector<std::uint8_t>& payload);

/// Receive-side accounting over sequence numbers 0 .. expected - 1.
/// Duplicates (possible with reliable retransmission) are counted once.
class SequenceTracker {
 public:
  explicit SequenceTracker(std::uint64_t expected);

  /// Records one received sequence number. Numbers >= expected are counted as
  /// out of range and otherwise ignored.
  void record(std::uint64_t sequence);

  [[nodiscard]] std::uint64_t expected() const { return expected_; }
  [[nodiscard]] std::uint64_t uniqueReceived() const { return unique_; }
  [[nodiscard]] std::uint64_t duplicates() const { return duplicates_; }
  [[nodiscard]] std::uint64_t outOfOrder() const { return out_of_order_; }
  [[nodiscard]] std::uint64_t outOfRange() const { return out_of_range_; }
  [[nodiscard]] std::uint64_t lost() const { return expected_ - unique_; }
  /// Fraction of expected messages never received, in [0, 1]; 0 if expected is 0.
  [[nodiscard]] double lossRatio() const;

 private:
  std::uint64_t expected_;
  std::vector<bool> seen_;
  std::uint64_t unique_{0};
  std::uint64_t duplicates_{0};
  std::uint64_t out_of_order_{0};
  std::uint64_t out_of_range_{0};
  std::optional<std::uint64_t> highest_;
};

}  // namespace qos_lab
