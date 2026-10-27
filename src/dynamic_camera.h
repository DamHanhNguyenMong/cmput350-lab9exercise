#ifndef DYNAMIC_CAMERA_H
#define DYNAMIC_CAMERA_H

#include <iostream>
#include <limits>

#include "camera.h"

constexpr float INF = std::numeric_limits<float>::infinity();

// AABB in 1D
struct Interval {
    float min;
    float max;

    constexpr bool contains(const Interval& other) const {
        return (min <= other.min) && (max >= other.max);
    }

    constexpr bool contains(float x) const { return (min <= x) && (x <= max); }

    // signedDistance returns a positive smallest distance of x from the interval boundary
    // if it is not contained, else it returns a smallest (in magnitude) negative distance
    // from the interval boundary.
    constexpr float signedDistance(float x) const {
        if (contains(x)) {
            // 0 or negative
            return std::max(x - max, min - x);
        }
        // positive
        return std::max(x - max, min - x);
    }
};

struct DynamicCamera {
    void update(Camera& camera,                           // the camera to update
                const sf::Vector2f& trackedPosWorld,      // world coords of player to track
                const sf::Vector2f& trackedVelocityWorld  // world velocity of player to track
    ) {
        // TODO: (HINTS/outline)
        // collect statistics on the player position and velocity in (normalized, if you like)
        // window coordinates. Compute lookAhead distance in world coords, adjusting for
        // if the player is moving left or right.
        // Now update lookAhead state using an EMA over its previous state and the lookAhead world
        // vector. Then determine if the player is in the dead zone, soft rejection zone, or hard
        // rejection zone. For each, perform EMA on the delta of the camera displacement (ignoring
        // lookahead) appropriately. Next,
        // set the camera center by offsetting the cameraUnadjustedCenter, combining lookAhead.
        // Finally update cameraUnadjustedCenter, but without lookahead.

        // Get the player's normalized window position, measured relative to the camera center WITHOUT lookahead
        // trackedPosWorld - cameraUnadjustedCenter is the distance between the camera and the palyer pos in world unit
        // We convert that into window percentage
        // Then + 0.5 since center is 1/2 of the window
        float playerPercentX = 0.5f + camera.worldToWindowPercentDist(trackedPosWorld - cameraUnadjustedCenter).x;
        
        // Calculate the lookahead distance in world units
        float lookAheadWorld = camera.windowPercentToWorldDist({lookAhead, 0.f}).x;
        // Look ahead in the direction the player is moving => same sign as the velocity
        float targetLookahead = 0.f;
        if (trackedVelocityWorld.x > 0.f) {
            targetLookahead = lookAheadWorld;
        }
        else if (trackedVelocityWorld.x < 0.f) {
            targetLookahead = -lookAheadWorld;
        }
        // Smooth the lookahead offset using EMA
        // targetLookahead - lookaheadState is how much we need to move, we update the current state by the
        // difference * alpha
        lookaheadState += alpha * (targetLookahead - lookaheadState);
        // Determine the camera displacement, ignoring lookahead
        float deltaPercent = 0.f;
        // Outside the hard zone: immediately correct to its boundary
        if (playerPercentX < softZoneX.min) {
            deltaPercent = playerPercentX - softZoneX.min;
        }
        else if (playerPercentX > softZoneX.max) {
            deltaPercent = playerPercentX - softZoneX.max;
        }
        // Outside the dead zone but inside the hard zone:
        // smoothly move toward the nearest dead-zone boundary
        else if (playerPercentX < deadZoneX.min) {
            deltaPercent = alpha * (playerPercentX - deadZoneX.min);
        }
        else if (playerPercentX > deadZoneX.max) {
            deltaPercent = alpha * (playerPercentX - deadZoneX.max);
        }
        // Convert the required screen-percentage displacement to world units
        float deltaWorld = camera.windowPercentToWorldDist({deltaPercent, 0.f}).x;

        // Update the camera position without lookahead
        cameraUnadjustedCenter.x += deltaWorld;
        // Apply the unadjusted center plus the smoothed lookahead
        // only update in the x direction
        camera.setCameraCenter({ cameraUnadjustedCenter.x + lookaheadState, cameraUnadjustedCenter.y});

    }

    float alpha;  // alpha to use for damping of camera position, *as well as* for smoothing of
                  // lookahead offset.
    Interval deadZoneX;  // percentage interval along x-axis to use for dead zone in [0, 1]
    // percentage interval along x-axis to use for soft rejection zone in [0, 1]
    Interval softZoneX = {-INF, INF};
    // (everything beyond softZone is considered hard zone).
    float lookAhead;  // screen-percentage lookahead
    mutable sf::Vector2f
        cameraUnadjustedCenter;      // the camera world center from last frame, ignoring lookahead
    mutable float lookaheadState{};  // the EMA of the accumulated lookahead offset in world coords
};

#endif // DYNAMIC_CAMERA_H
