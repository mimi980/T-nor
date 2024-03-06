#ifndef TWIST2D_H
#define TWIST2D_H

#include <cmath>   // Pour std::hypot et std::abs
#include <string>  // Pour std::string
#include <sstream> // Pour std::ostringstream
#include <iomanip> // Pour std::setprecision

// Assurez-vous que Util.h est inclus et défini quelque part dans votre projet
#include "lib/utils.h"
#include "lib/RblUtils.h"


class Twist2d
{
public:
    static const Twist2d kIdentity;
    static const double kEpsilon = 1e-12;
    const double dx;
    const double dy;
    const double dtheta; // En radians !

    Twist2d(double dx, double dy, double dtheta) : dx(dx), dy(dy), dtheta(dtheta) {}

    Twist2d scaled(double scale) const
    {
        return Twist2d(dx * scale, dy * scale, dtheta * scale);
    }

    double norm() const
    {
        if (dy == 0.0)
            return std::abs(dx);
        return std::hypot(dx, dy);
    }

    double curvature() const
    {
        if (std::abs(dtheta) < kEpsilon && norm() < kEpsilon)
            return 0.0;
        return dtheta / norm();
    }

    std::string toString() const
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        oss << "(" << dx << "," << dy << "," << dtheta * 180.0 / NF64_PI << " deg)";
        return oss.str();
    }

    static Twist2d identity()
    {
        return kIdentity;
    }
};

const Twist2d Twist2d::kIdentity(0.0, 0.0, 0.0);

#endif // TWIST2D_H
