#ifndef ROTATION2D_H
#define ROTATION2D_H

#include <cmath>
#include <limits>
#include <lib/RblUtils.h>

class Rotation2d {
public:
    static const Rotation2d kIdentity;

private:
    double cos_angle_ = std::numeric_limits<double>::quiet_NaN();
    double sin_angle_ = std::numeric_limits<double>::quiet_NaN();
    double radians_ = std::numeric_limits<double>::quiet_NaN();

public:
    Rotation2d(double x, double y, bool normalize) {
        if (normalize) {
            double magnitude = std::hypot(x, y);
            if (magnitude > kEpsilon) {
                sin_angle_ = y / magnitude;
                cos_angle_ = x / magnitude;
            } else {
                sin_angle_ = 0.0;
                cos_angle_ = 1.0;
            }
        } else {
            cos_angle_ = x;
            sin_angle_ = y;
        }
        radians_ = std::atan2(sin_angle_, cos_angle_);
    }

    Rotation2d() : Rotation2d(1.0, 0.0, false) {}

    Rotation2d(double radians, bool normalize) {
        if (normalize) {
            radians = WrapRadians(radians);
        }
        radians_ = radians;
        cos_angle_ = std::cos(radians_);
        sin_angle_ = std::sin(radians_);
    }

    Rotation2d(const Rotation2d& other) = default;

    static Rotation2d identity() {
        return kIdentity;
    }

    static Rotation2d fromRadians(double angle_radians) {
        return Rotation2d(angle_radians, true);
    }

    double cos() const {
        return cos_angle_;
    }

    double sin() const {
        return sin_angle_;
    }

    double tan() const {
        if (std::abs(cos_angle_) < kEpsilon) {
            return std::copysign(std::numeric_limits<double>::infinity(), sin_angle_);
        }
        return sin_angle_ / cos_angle_;
    }

    double getRadians() const {
        return radians_;
    }


Rotation2d fromDegrees(double angle_degrees) {
    return fromRadians(angle_degrees * NF64_PI / 180.0); // Conversion degrés en radians
}

double cos() const {
    // Dans cette implémentation, nous supposons que cos_angle_ et sin_angle_ sont toujours calculés
    return cos_angle_;
}

double sin() const {
    // Dans cette implémentation, nous supposons que cos_angle_ et sin_angle_ sont toujours calculés
    return sin_angle_;
}

double tan() const {
    if (std::abs(cos_angle_) < kEpsilon) {
        return std::copysign(std::numeric_limits<double>::infinity(), sin_angle_);
    }
    return sin_angle_ / cos_angle_;
}

double getRadians() const {
    // Dans cette implémentation, nous supposons que radians_ est toujours calculé
    return radians_;
}

double getDegrees() const {
    return getRadians() * 180.0 / NF64_PI; // Conversion radians en degrés
}

Rotation2d rotateBy(const Rotation2d& other) const {
    // Multiplier cette rotation par une autre rotation
    return Rotation2d(cos_angle_ * other.cos_angle_ - sin_angle_ * other.sin_angle_,
                      cos_angle_ * other.sin_angle_ + sin_angle_ * other.cos_angle_, true);
}

Rotation2d normal() const {
    // Rotation de 90 degrés dans le sens anti-horaire
    return Rotation2d(-sin_angle_, cos_angle_, false);
}

// Suite de la classe Rotation2d en C++

Rotation2d inverse() const {
    // Utilisation de la trigonométrie calculée si disponible, sinon calcul direct à partir de radians
    return Rotation2d(cos_angle_, -sin_angle_, true);
}

bool isParallel(const Rotation2d& other) const {
    // Vérifie si deux rotations sont parallèles en comparant leurs angles
    double angleDiff = std::fabs(radians_ - other.radians_);
    angleDiff = fmod(angleDiff, 2 * NF64_PI);
    return utils::epsilonEquals(angleDiff, 0.0) || utils::epsilonEquals(angleDiff, NF64_PI);
}

Translation2d toTranslation() const {
    // Convertit la rotation en translation en utilisant le cosinus et le sinus comme composants x et y
    return Translation2d(cos_angle_, sin_angle_);
}

static double WrapRadians(double radians) {
    // Normalise les radians pour être entre -PI et PI
    radians = fmod(radians, 2.0 * NF64_PI);
    if (radians < -NF64_PI) {
        radians += 2.0 * NF64_PI;
    } else if (radians > NF64_PI) {
        radians -= 2.0 * NF64_PI;
    }
    return radians;
}

Rotation2d interpolate(const Rotation2d& other, double x) const {
    // Interpolation entre cette rotation et une autre
    if (x <= 0.0) {
        return *this;
    } else if (x >= 1.0) {
        return other;
    }
    double angleDiff = inverse().rotateBy(other).getRadians();
    return this->rotateBy(Rotation2d(angleDiff * x, true));
}

std::string toString() const {
    // Représentation en chaîne de caractères de la rotation en degrés
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << getDegrees() << " deg";
    return ss.str();
}

std::string toCSV() const {
    // Format CSV de la rotation en degrés
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << getDegrees();
    return ss.str();
}

double distance(const Rotation2d& other) const {
    // Calcule la distance angulaire entre deux rotations
    return inverse().rotateBy(other).getRadians();
}

bool equals(const Rotation2d& other) const {
    // Comparaison basée sur la distance angulaire
    return distance(other) < utils::kEpsilon;
}

Rotation2d getRotation() const {
    // Retourne cette rotation
    return *this;
}

private:
    static constexpr double kEpsilon = 1E-9;

    static double WrapRadians(double radians) {
        // Implementation to normalize angle in radians
    }
};

const Rotation2d Rotation2d::kIdentity = Rotation2d();

#endif // ROTATION2D_H
