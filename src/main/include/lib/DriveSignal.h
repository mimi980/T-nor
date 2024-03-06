#pragma once

#include <string> // Pour std::string
#include <sstream> // Pour std::stringstream

class DriveSignal {
private:
    double mLeftMotor;
    double mRightMotor;
    bool mBrakeMode;

public:
    // Constructeur par défaut désactivé
    DriveSignal() = delete;

    // Constructeur principal
    DriveSignal(double left, double right, bool brakeMode = false)
        : mLeftMotor(left), mRightMotor(right), mBrakeMode(brakeMode) {}

    // Générateur de signal à partir de contrôles
    static DriveSignal fromControls(double throttle, double turn) {
        return DriveSignal(throttle - turn, throttle + turn);
    }

    // Signaux statiques
    static const DriveSignal NEUTRAL;
    static const DriveSignal BRAKE;

    // Accesseurs
    double getLeft() const { return mLeftMotor; }
    double getRight() const { return mRightMotor; }
    bool getBrakeMode() const { return mBrakeMode; }

    // Conversion en chaîne de caractères pour l'affichage
    std::string toString() const {
        std::stringstream ss;
        ss << "L: " << mLeftMotor << ", R: " << mRightMotor;
        if (mBrakeMode) {
            ss << ", BRAKE";
        }
        return ss.str();
    }
};

// Initialisation des membres statiques
const DriveSignal DriveSignal::NEUTRAL(0, 0);
const DriveSignal DriveSignal::BRAKE(0, 0, true);
