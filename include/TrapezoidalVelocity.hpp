#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <FastAccelStepper.h>

constexpr double kMaxDuration = 2000.0;
constexpr double kMinAcceleration = 1e-12;

enum class TrapezoidalVelocityError
{
    NONE = 0,
    INVALID_LIMITS,
    INVALID_INPUT,
    TARGET_UNREACHABLE
};

enum class TrapezoidalVelocityState
{
    IDLE = 0,
    IN_PROGRESS,
    DONE
};

struct TrapezoidalLimits
{
    double max_duration = 10.0;
    double max_velocity = 360.0;
    double max_acceleration = 1000.0;
};

struct TrapezoidalVelocityPoint
{
    TrapezoidalVelocityState state = TrapezoidalVelocityState::IDLE;
    double velocity = 0.0;
    double acceleration = 0.0;
};

class TrapezoidalVelocity
{
public:
    TrapezoidalVelocity() = default;

    TrapezoidalVelocityError configure_limits(const TrapezoidalLimits &limits);

    void reset(const uint32_t t_start_micros, const double velocity);
    TrapezoidalVelocityError set_target(const uint32_t t_start_micros, const double target_velocity, const double target_max_acceleration = 0.0);

    TrapezoidalVelocityPoint process(const uint32_t t_eval_micros);

private:
    TrapezoidalVelocityPoint get_point(const double t_elapsed_sec) const;

    static constexpr double kMinVelocity = 1e-12;

    uint32_t m_start_time = 0;
    double m_start_velocity = 0.0;

    double m_duration = 0;
    double m_target_velocity = 0.0;
    double m_target_acceleration = 0.0;

    TrapezoidalVelocityState m_state = TrapezoidalVelocityState::IDLE;
    TrapezoidalLimits m_limits = {};
};

inline TrapezoidalVelocityError TrapezoidalVelocity::configure_limits(const TrapezoidalLimits &limits)
{
    const auto is_finite_positive = [](double v) -> bool
    {
        return std::isfinite(v) && v > 0.0;
    };

    if (!is_finite_positive(limits.max_duration) || limits.max_duration > kMaxDuration)
    {
        return TrapezoidalVelocityError::INVALID_LIMITS;
    }
    if (!is_finite_positive(limits.max_velocity))
    {
        return TrapezoidalVelocityError::INVALID_LIMITS;
    }
    if (!is_finite_positive(limits.max_acceleration))
    {
        return TrapezoidalVelocityError::INVALID_LIMITS;
    }

    // Check if duration to go from max velocity to zero exceeds max duration
    const double time_to_stop = limits.max_velocity / limits.max_acceleration;
    if (time_to_stop > limits.max_duration)
    {
        return TrapezoidalVelocityError::INVALID_LIMITS;
    }

    m_limits = limits;

    // Reset current state to target with new limits
    reset(0.0, m_target_velocity);
    return TrapezoidalVelocityError::NONE;
}

inline void TrapezoidalVelocity::reset(const uint32_t t_start_micros, const double velocity)
{
    m_start_time = t_start_micros;
    m_start_velocity = std::clamp(velocity, -m_limits.max_velocity, m_limits.max_velocity);

    m_duration = 0.0;
    m_target_velocity = m_start_velocity;
    m_target_acceleration = 0.0;

    m_state = TrapezoidalVelocityState::IDLE;
}

inline TrapezoidalVelocityPoint TrapezoidalVelocity::get_point(const double t_elapsed_sec) const
{
    TrapezoidalVelocityPoint point = {};
    point.state = TrapezoidalVelocityState::IN_PROGRESS;

    if (t_elapsed_sec <= 0.0)
    {
        point.velocity = m_start_velocity;
        point.acceleration = 0.0;
    }
    else if (t_elapsed_sec >= m_duration)
    {
        point.velocity = m_target_velocity;
        point.acceleration = 0.0;
        point.state = TrapezoidalVelocityState::DONE;
    }
    else
    {
        point.velocity = m_start_velocity + (m_target_acceleration * t_elapsed_sec);
        point.acceleration = m_target_acceleration;
    }

    return point;
}

inline TrapezoidalVelocityError TrapezoidalVelocity::set_target(const uint32_t t_start_micros, double target_velocity, double target_max_acceleration)
{

    if (!std::isfinite(target_velocity))
    {
        return TrapezoidalVelocityError::INVALID_INPUT;
    }

    if (!std::isfinite(target_max_acceleration))
    {
        return TrapezoidalVelocityError::INVALID_INPUT;
    }

    // clamp target velocity to limits
    target_velocity = std::clamp(target_velocity, -m_limits.max_velocity, m_limits.max_velocity);

    // Use max acceleration if none specified
    if (std::abs(target_max_acceleration) < kMinAcceleration)
    {
        target_max_acceleration = m_limits.max_acceleration;
    }
    else
    {
        target_max_acceleration = std::min(target_max_acceleration, m_limits.max_acceleration);
    }

    // calculate start velocity if in progress
    double start_velocity = m_target_velocity;
    if (m_state == TrapezoidalVelocityState::IN_PROGRESS)
    {
        const double t_elapsed_sec = static_cast<double>(t_start_micros - m_start_time) * 1e-6;
        start_velocity = get_point(t_elapsed_sec).velocity;
    }

    // duration to reach new target
    const double delta_v = target_velocity - start_velocity;
    const double required_duration = std::abs(delta_v) / target_max_acceleration;
    if (required_duration > m_limits.max_duration)
    {
        return TrapezoidalVelocityError::TARGET_UNREACHABLE;
    }

    // Apply new trajectory
    const double sign = (delta_v >= 0.0) ? 1.0 : -1.0;
    m_start_time = t_start_micros;
    m_start_velocity = start_velocity;
    m_duration = required_duration;
    m_target_velocity = target_velocity;
    m_target_acceleration = sign * target_max_acceleration;
    m_state = TrapezoidalVelocityState::IN_PROGRESS;

    return TrapezoidalVelocityError::NONE;
}

inline TrapezoidalVelocityPoint TrapezoidalVelocity::process(uint32_t t_eval_micros)
{
    TrapezoidalVelocityPoint point = {};
    if (m_state == TrapezoidalVelocityState::IN_PROGRESS)
    {
        const double t_elapsed_sec = static_cast<double>(t_eval_micros - m_start_time) * 1e-6;
        point = get_point(t_elapsed_sec);
        m_state = point.state;
    }
    else
    {
        point.velocity = m_target_velocity;
        point.acceleration = 0.0;
        point.state = m_state;
    }
    return point;
}
