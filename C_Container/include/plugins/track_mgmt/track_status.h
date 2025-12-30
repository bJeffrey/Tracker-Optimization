#pragma once
/**
 * @file track_status.h
 * @brief Minimal track status enumeration for tracker tracks.
 *
 * Notes:
 * - INACTIVE is a "slot unused" value. Keeping it at 0 makes default
 *   initialization safe and intentional.
 * - ACTIVE/COASTING/DROPPED are the initial live-state set; we can extend later
 *   (e.g., TENTATIVE, CONFIRMED, DELETED, etc.).
 */

#include <cstdint>

enum class TrackStatus : std::uint8_t
{
  INACTIVE = 0,  // slot unused / not a live track (default)
  ACTIVE   = 1,  // live track
  COASTING = 2,  // missed update, being predicted forward
  DROPPED  = 3   // terminal / ready for recycle (policy-dependent)
};
