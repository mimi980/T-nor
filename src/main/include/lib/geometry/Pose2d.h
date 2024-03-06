#ifndef POSE2D_H
#define POSE2D_H

#include "Translation2d.h"
#include "Rotation2d.h"
#include "Twist2d.h"
#include "lib/utils.h" // Assurez-vous que cette classe contient une constante kEpsilon
#include <cmath>

class Pose2d
{
public:
    static const Pose2d kIdentity;
    static const double kEpsilon = 1e-12;

    Translation2d translation_;
    Rotation2d rotation_;

    Pose2d() : translation_(Translation2d()), rotation_(Rotation2d()) {}

    Pose2d(double x, double y, const Rotation2d &rotation)
        : translation_(Translation2d(x, y)), rotation_(rotation) {}

    Pose2d(const Translation2d &translation, const Rotation2d &rotation)
        : translation_(translation), rotation_(rotation) {}

    Pose2d(const Pose2d &other) = default;

    static Pose2d fromTranslation(const Translation2d &translation)
    {
        return Pose2d(translation, Rotation2d());
    }

    static Pose2d fromRotation(const Rotation2d &rotation)
    {
        return Pose2d(Translation2d(), rotation);
    }

    static Pose2d exp(const Twist2d &delta)
    {
        double sin_theta = std::sin(delta.dtheta);
        double cos_theta = std::cos(delta.dtheta);
        double s, c;
        if (std::abs(delta.dtheta) < kEpsilon)
        {
            s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
            c = 0.5 * delta.dtheta;
        }
        else
        {
            s = sin_theta / delta.dtheta;
            c = (1.0 - cos_theta) / delta.dtheta;
        }
        return Pose2d(Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
                      Rotation2d(cos_theta, sin_theta));
    }

    static Twist2d log(const Pose2d &transform)
    {
        double dtheta = transform.rotation_.getRadians();
        double half_dtheta = 0.5 * dtheta;
        double cos_minus_one = transform.rotation_.cos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (std::abs(cos_minus_one) < utils::kEpsilon)
        {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        }
        else
        {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.rotation_.sin()) / cos_minus_one;
        }
        Translation2d translation_part = transform.translation_
                                             .rotateBy(Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        return Twist2d(translation_part.x(), translation_part.y(), dtheta);
    }

    Pose2d transformBy(const Pose2d &other) const
    {
        return Pose2d(translation_.translateBy(other.translation_.rotateBy(rotation_)),
                      rotation_.rotateBy(other.rotation_));
    }

    Translation2d getTranslation() const { return translation_; }
    Rotation2d getRotation() const { return rotation_; }

    Pose2d inverse() const
    {
        Rotation2d rotation_inverted = rotation_.inverse();
        return Pose2d(translation_.inverse().rotateBy(rotation_inverted), rotation_inverted);
    }

    Pose2d normal() const
    {
        return Pose2d(translation_, rotation_.normal());
    }

    Translation2d intersection(const Pose2d &other) const
    {
        const Rotation2d &other_rotation = other.rotation_;
        if (rotation_.isParallel(other_rotation))
        {
            return Translation2d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
        }
        if (std::abs(rotation_.cos()) < std::abs(other_rotation.cos()))
        {
            return intersectionInternal(*this, other);
        }
        else
        {
            return intersectionInternal(other, *this);
        }
    }

    bool isColinear(const Pose2d &other) const
    {
        if (!rotation_.isParallel(other.rotation_))
            return false;
        Twist2d twist = Pose2d::log(inverse().transformBy(other));
        return utils::epsilonEquals(twist.dy, 0.0) && utils::epsilonEquals(twist.dtheta, 0.0);
    }

    bool epsilonEquals(const Pose2d &other, double epsilon) const
    {
        return translation_.epsilonEquals(other.translation_, epsilon) && rotation_.isParallel(other.rotation_);
    }

    Pose2d interpolate(const Pose2d &other, double x) const
    {
        if (x <= 0)
        {
            return Pose2d(*this);
        }
        else if (x >= 1)
        {
            return Pose2d(other);
        }
        Twist2d twist = Pose2d::log(inverse().transformBy(other));
        return transformBy(Pose2d::exp(twist.scaled(x)));
    }

    double distance(const Pose2d &other) const
    {
        return Pose2d::log(inverse().transformBy(other)).norm();
    }

    bool equals(const Pose2d &other) const
    {
        return epsilonEquals(other, utils::kEpsilon);
    }

    Pose2d mirror() const
    {
        return Pose2d(Translation2d(translation_.x(), -translation_.y()), rotation_.inverse());
    }

    // Implementation privée
private:
    static Translation2d intersectionInternal(const Pose2d &a, const Pose2d &b)
    {
        const Rotation2d &a_r = a.rotation_;
        const Rotation2d &b_r = b.rotation_;
        const Translation2d &a_t = a.translation_;
        const Translation2d &b_t = b.translation_;

        double tan_b = b_r.tan();
        double t = ((a_t.x() - b_t.x()) * tan_b + b_t.y() - a_t.y()) / (a_r.sin() - a_r.cos() * tan_b);
        if (std::isnan(t))
        {
            return Translation2d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
        }
        return a_t.translateBy(a_r.toTranslation().scale(t));
    }

        static constexpr double kEps = 1E-9;

};

#endif // POSE2D_H
