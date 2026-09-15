#pragma once

#include <cstdint>

// Observation-only contract between peer communication receivers and the
// communication safety policy. Receiver implementation details stay outside
// this type.
struct CommHealthState {
    bool peer_present = false;
    bool snapshot_valid = false;
    uint64_t last_valid_rx_us = 0;
    float snapshot_rx_aoi_ms = 999999.0f;

    uint32_t last_seq = 0;
    uint32_t rx_count = 0;
    uint32_t seq_gap_count = 0;
    uint64_t missing_snapshot_count = 0;
    uint32_t current_burst_loss_count = 0;
    uint32_t max_burst_loss_count = 0;

    uint32_t invalid_count = 0;
    uint32_t crc_error_count = 0;
    uint32_t wrong_sender_count = 0;
    uint32_t wrong_source_count = 0;
    uint32_t wrong_destination_count = 0;
    uint32_t duplicate_count = 0;
    uint32_t replay_count = 0;
    uint32_t stale_timestamp_count = 0;

    // PeerHealth remains secondary evidence and diagnostic data.
    bool peer_health_valid = false;
    float peer_health_rx_aoi_ms = 999999.0f;
    float peer_reported_aoi_ms = 999999.0f;
    uint32_t peer_reported_network = 2u;
    uint32_t peer_health_seq = 0;
};
