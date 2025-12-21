#pragma once
/**
 * @file track_status.h
 * @brief Minimal track status enumeration for TrackBatch (SoA storage).
 *
 * This is intentionally small for Step 1. We can extend with more states later
 * (e.g., TENTATIVE, CONFIRMED, DELETED, etc.).
 */

#include <cstdint>

enum class TrackStatus : std::uint8_t
{
  ACTIVE   = 0,
  COASTING = 1,
  DROPPED  = 2
};
