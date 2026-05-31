#include "booster_k1_controller/interpolator.hpp"
#include <stdexcept>
#include <cmath>

void Interpolator::load(const action_interface::msg::JointTrajectory& trajectory)
{
    end_position.reserve(Joint::kJointCnt);
    last_sent_position.reserve(Joint::kJointCnt);
    steps.clear();

    if (trajectory.points.empty())
        throw std::invalid_argument("Trajectory has no points");

    double cursor = 0.0;

    for (size_t i = 0; i < trajectory.points.size(); i++) {
        const auto& point = trajectory.points[i];         

        if (point.positions.size() != Joint::kJointCnt)    
            throw std::invalid_argument("Number of joints don't match joint count");

        std::vector<double> from =
            (i == 0) ? std::vector<double>(Joint::kJointCnt, 0.0)
                     : trajectory.points[i - 1].positions;

        cursor += point.delay_before_seconds;
        double t_start = cursor;
        cursor += point.duration_seconds;
        double t_end = cursor;

        steps.push_back({ t_start, t_end, from, point.positions });
    }

    total_duration = cursor;
    end_position = trajectory.points.back().positions;    
}

void Interpolator::set_start_positions(const std::vector<double>& current_joint_q)
{
    if (!steps.empty())
        steps[0].from = current_joint_q;
}

std::optional<std::vector<double>> Interpolator::sample(double t)  
{
    for (const auto& step : steps) {
        if (t < step.t_start)
            return step.from;

        if (t <= step.t_end) {
            double alpha = (t - step.t_start) / (step.t_end - step.t_start);
            alpha = std::clamp(alpha, 0.0, 1.0);
            last_sent_position = lerp(step.from, step.to, alpha);
            return last_sent_position;                     
        }

        if (!all_reached(last_sent_position, step.to, Joint::kMaxJointDelta)) {
            last_sent_position = lerp(last_sent_position, step.to, 1.0);
            return last_sent_position;
        }
    }

    return std::nullopt;
}

bool Interpolator::is_done(double t) const
{
    if (t < total_duration) return false;
    return all_reached(last_sent_position, end_position, Joint::kMaxJointDelta);
}

bool Interpolator::all_reached(                            
    const std::vector<double>& current,
    const std::vector<double>& target,
    double tolerance)
{
    for (size_t i = 0; i < current.size(); i++) {
        if (std::abs(current[i] - target[i]) > tolerance)
            return false;
    }
    return true;
}

const std::vector<double>& Interpolator::end_position() const  
{
    return end_position;
}

std::vector<double> Interpolator::lerp(
    const std::vector<double>& a,
    const std::vector<double>& b,
    double alpha)
{
    std::vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); i++) {
        result[i] = a[i] + alpha * (b[i] - a[i]);
        result[i] = std::clamp(result[i],                 
            a[i] - Joint::kMaxJointDelta,
            a[i] + Joint::kMaxJointDelta);
    }
    return result;
}