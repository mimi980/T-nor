#include <cmath> // Pour std::abs et d'autres fonctions mathématiques

// Assurez-vous que les classes Pose2d, Rotation2d, Twist2d et DriveSignal sont définies quelque part dans votre projet C++.
// #include "Pose2d.h"
// #include "Rotation2d.h"
#include "geometry/Twist2d.h"
// #include "DriveSignal.h"
#include "Constants.h"

class Kinematics
{
public:
    static constexpr double kEpsilon = 1E-9;

    static Twist2d forwardKinematics(double left_wheel_delta, double right_wheel_delta)
    {
        double delta_rotation = (right_wheel_delta - left_wheel_delta) / (DRIVE_WHEEL_TRACK_WIDTH_INCHES * TRACK_SCRUB_FACTOR);
        return forwardKinematics(left_wheel_delta, right_wheel_delta, delta_rotation);
    }

    static Twist2d forwardKinematics(double left_wheel_delta, double right_wheel_delta, double delta_rotation_rads)
    {
        double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        return Twist2d(dx, 0.0, delta_rotation_rads);
    }

    static Twist2d forwardKinematics(const Rotation2d &prev_heading, double left_wheel_delta, double right_wheel_delta, const Rotation2d &current_heading)
    {
        double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        double dy = 0.0;
        return Twist2d(dx, dy, prev_heading.inverse().rotateBy(current_heading).getRadians());
    }

    static Pose2d integrateForwardKinematics(const Pose2d &current_pose, const Twist2d &forward_kinematics)
    {
        return current_pose.transformBy(Pose2d::exp(forward_kinematics));
    }

    static DriveSignal inverseKinematics(const Twist2d &velocity)
    {
        if (std::abs(velocity.dtheta) < kEpsilon)
        {
            return DriveSignal(velocity.dx, velocity.dx);
        }
        double delta_v = DRIVE_WHEEL_TRACK_WIDTH_INCHES * velocity.dtheta / (2 * TRACK_SCRUB_FACTOR);
        return DriveSignal(velocity.dx - delta_v, velocity.dx + delta_v);
    }
};
