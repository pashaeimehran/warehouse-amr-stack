#include "qos_lab/probe.hpp"

namespace qos_lab {
namespace {

void putU64(std::uint64_t value, std::uint8_t* out) {
  for (std::size_t i = 0; i < sizeof(value); ++i) {
    out[i] = static_cast<std::uint8_t>((value >> (8U * i)) & 0xFFU);
  }
}

std::uint64_t getU64(const std::uint8_t* in) {
  std::uint64_t value = 0;
  for (std::size_t i = 0; i < sizeof(value); ++i) {
    value |= static_cast<std::uint64_t>(in[i]) << (8U * i);
  }
  return value;
}

}  // namespace

std::optional<std::vector<std::uint8_t>> encodeProbe(const ProbeHeader& header,
                                                     std::size_t payload_size) {
  if (payload_size < kProbeHeaderSize) {
    return std::nullopt;
  }
  std::vector<std::uint8_t> payload(payload_size, 0xA5);
  if (!writeProbeHeader(header, payload)) {
    return std::nullopt;
  }
  return payload;
}

bool writeProbeHeader(const ProbeHeader& header, std::vector<std::uint8_t>& payload) {
  if (payload.size() < kProbeHeaderSize) {
    return false;
  }
  putU64(header.sequence, payload.data());
  putU64(static_cast<std::uint64_t>(header.send_time_ns), payload.data() + sizeof(std::uint64_t));
  return true;
}

std::optional<ProbeHeader> decodeProbe(const std::vector<std::uint8_t>& payload) {
  if (payload.size() < kProbeHeaderSize) {
    return std::nullopt;
  }
  ProbeHeader header;
  header.sequence = getU64(payload.data());
  header.send_time_ns = static_cast<std::int64_t>(getU64(payload.data() + sizeof(std::uint64_t)));
  return header;
}

SequenceTracker::SequenceTracker(std::uint64_t expected)
    : expected_(expected), seen_(static_cast<std::size_t>(expected), false) {}

void SequenceTracker::record(std::uint64_t sequence) {
  if (sequence >= expected_) {
    ++out_of_range_;
    return;
  }
  const auto index = static_cast<std::size_t>(sequence);
  if (seen_[index]) {
    ++duplicates_;
    return;
  }
  seen_[index] = true;
  ++unique_;
  if (highest_.has_value() && sequence < *highest_) {
    ++out_of_order_;
  } else {
    highest_ = sequence;
  }
}

double SequenceTracker::lossRatio() const {
  if (expected_ == 0) {
    return 0.0;
  }
  return static_cast<double>(lost()) / static_cast<double>(expected_);
}

}  // namespace qos_lab
