#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "qos_lab/probe.hpp"
#include "qos_lab/profiles.hpp"

namespace qos_lab {
namespace {

TEST(Probe, RoundTripsHeader) {
  const ProbeHeader header{123456789ULL, -42};
  const auto payload = encodeProbe(header, 64);
  ASSERT_TRUE(payload.has_value());
  EXPECT_EQ(payload->size(), 64U);
  const auto decoded = decodeProbe(*payload);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->sequence, header.sequence);
  EXPECT_EQ(decoded->send_time_ns, header.send_time_ns);
}

TEST(Probe, RoundTripsExtremeValues) {
  const ProbeHeader header{std::numeric_limits<std::uint64_t>::max(),
                           std::numeric_limits<std::int64_t>::min()};
  const auto payload = encodeProbe(header, kProbeHeaderSize);
  ASSERT_TRUE(payload.has_value());
  const auto decoded = decodeProbe(*payload);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->sequence, header.sequence);
  EXPECT_EQ(decoded->send_time_ns, header.send_time_ns);
}

TEST(Probe, LayoutIsLittleEndian) {
  const auto payload = encodeProbe({0x0102030405060708ULL, 0}, kProbeHeaderSize);
  ASSERT_TRUE(payload.has_value());
  EXPECT_EQ((*payload)[0], 0x08);
  EXPECT_EQ((*payload)[7], 0x01);
}

TEST(Probe, RejectsTooSmallPayload) {
  EXPECT_FALSE(encodeProbe({}, kProbeHeaderSize - 1).has_value());
  std::vector<std::uint8_t> small(kProbeHeaderSize - 1);
  EXPECT_FALSE(writeProbeHeader({}, small));
  EXPECT_FALSE(decodeProbe(small).has_value());
}

TEST(Probe, WriteHeaderKeepsSizeAndFiller) {
  auto payload = encodeProbe({1, 2}, 1024);
  ASSERT_TRUE(payload.has_value());
  ASSERT_TRUE(writeProbeHeader({99, 100}, *payload));
  EXPECT_EQ(payload->size(), 1024U);
  EXPECT_EQ(payload->back(), 0xA5);
  const auto decoded = decodeProbe(*payload);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->sequence, 99U);
}

TEST(SequenceTracker, CountsLossDuplicatesAndReordering) {
  SequenceTracker tracker(10);
  for (const std::uint64_t s : {0, 1, 2, 4, 3, 3, 7, 9}) {
    tracker.record(s);
  }
  EXPECT_EQ(tracker.uniqueReceived(), 7U);  // 0 1 2 3 4 7 9
  EXPECT_EQ(tracker.duplicates(), 1U);      // second 3
  EXPECT_EQ(tracker.outOfOrder(), 1U);      // 3 after 4
  EXPECT_EQ(tracker.lost(), 3U);            // 5 6 8
  EXPECT_DOUBLE_EQ(tracker.lossRatio(), 0.3);
}

TEST(SequenceTracker, OutOfRangeIsIgnored) {
  SequenceTracker tracker(2);
  tracker.record(0);
  tracker.record(5);
  EXPECT_EQ(tracker.outOfRange(), 1U);
  EXPECT_EQ(tracker.uniqueReceived(), 1U);
  EXPECT_EQ(tracker.lost(), 1U);
}

TEST(SequenceTracker, EmptyExpectationHasZeroLoss) {
  const SequenceTracker tracker(0);
  EXPECT_DOUBLE_EQ(tracker.lossRatio(), 0.0);
  EXPECT_EQ(tracker.lost(), 0U);
}

TEST(Profiles, AllNamedProfilesResolve) {
  ASSERT_EQ(profileNames().size(), 3U);
  for (const auto& name : profileNames()) {
    EXPECT_TRUE(profileByName(name).has_value()) << name;
  }
  EXPECT_FALSE(profileByName("unknown").has_value());
}

TEST(Profiles, PoliciesMatchTheirPurpose) {
  const auto reliable = profileByName("reliable_volatile");
  const auto sensor = profileByName("best_effort_volatile");
  const auto latched = profileByName("reliable_transient_local");
  ASSERT_TRUE(reliable && sensor && latched);
  EXPECT_EQ(reliable->reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(reliable->depth(), 10U);
  EXPECT_EQ(sensor->reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(latched->durability(), rclcpp::DurabilityPolicy::TransientLocal);
  EXPECT_EQ(latched->depth(), 1U);
}

}  // namespace
}  // namespace qos_lab
