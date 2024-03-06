#ifndef TRANSLATION2D_H
#define TRANSLATION2D_H

#include <cmath>
#include "Rotation2d.h" // Assurez-vous d'inclure votre classe Rotation2d
#include <iomanip> // Pour std::setprecision
#include <sstream> // Pour std::stringstream
#include <lib/utils.h> // Assurez-vous que cette classe contient une constante kEpsilon


class Translation2d {
public:
    static const Translation2d kIdentity; // Identité de translation

    // Constructeurs
    Translation2d() : x_(0.0), y_(0.0) {}
    Translation2d(double x, double y) : x_(x), y_(y) {}
    Translation2d(const Translation2d& other) : x_(other.x_), y_(other.y_) {}
    Translation2d(const Translation2d& start, const Translation2d& end) : x_(end.x_ - start.x_), y_(end.y_ - start.y_) {}

    // Norme de la translation (distance Euclidienne)
    double norm() const {
        return std::hypot(x_, y_);
    }

    // Norme au carré pour éviter la racine carrée lorsque ce n'est pas nécessaire
    double norm2() const {
        return x_ * x_ + y_ * y_;
    }

    // Accesseurs
    double x() const { return x_; }
    double y() const { return y_; }

    // Composition de translations
    Translation2d translateBy(const Translation2d& other) const {
        return Translation2d(x_ + other.x_, y_ + other.y_);
    }

    // Rotation de la translation
    Translation2d rotateBy(const Rotation2d& rotation) const {
        return Translation2d(
            x_ * rotation.cos() - y_ * rotation.sin(),
            x_ * rotation.sin() + y_ * rotation.cos()
        );
    }

    // Direction de la translation comme rotation
    Rotation2d direction() const {
        return Rotation2d(x_, y_, true);
    }
    Translation2d inverse() const {
        return Translation2d(-x_, -y_);
    }

    Translation2d interpolate(const Translation2d& other, double x) const {
        if (x <= 0.0) {
            return *this;
        } else if (x >= 1.0) {
            return other;
        }
        return extrapolate(other, x);
    }

    Translation2d extrapolate(const Translation2d& other, double x) const {
        return Translation2d(x * (other.x_ - x_) + x_, x * (other.y_ - y_) + y_);
    }

    Translation2d scale(double s) const {
        return Translation2d(x_ * s, y_ * s);
    }

    bool epsilonEquals(const Translation2d& other, double epsilon) const {
        return utils::epsilonEquals(x_, other.x_, epsilon) && utils::epsilonEquals(y_, other.y_, epsilon);
    }

    std::string toString() const {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(3) << "(" << x_ << "," << y_ << ")";
        return stream.str();
    }

    std::string toCSV() const {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(3) << x_ << "," << y_;
        return stream.str();
    }

    static double dot(const Translation2d& a, const Translation2d& b) {
        return a.x_ * b.x_ + a.y_ * b.y_;
    }

    static Rotation2d getAngle(const Translation2d& a, const Translation2d& b) {
        double cos_angle = dot(a, b) / (a.norm() * b.norm());
        if (std::isnan(cos_angle)) {
            return Rotation2d();
        }
        return Rotation2d::fromRadians(std::acos(utils::limit(cos_angle, -1.0, 1.0)));
    }

    static double cross(const Translation2d& a, const Translation2d& b) {
        return a.x_ * b.y_ - a.y_ * b.x_;
    }

    double distance(const Translation2d& other) const {
        return inverse().translateBy(other).norm();
    }

    bool equals(const Translation2d& other) const {
        return distance(other) < utils::kEpsilon;
    }

    Translation2d getTranslation() const {
        return *this;
    }

private:
    double x_, y_;
};

// Initialisation de la constante kIdentity
const Translation2d Translation2d::kIdentity = Translation2d();

#endif // TRANSLATION2D_H
