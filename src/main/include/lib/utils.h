#pragma once

namespace utils
{
    inline bool epsilonEquals(double a, double b, double epsilon)
    {
        return ((a - epsilon <= b) and (a + epsilon >= b));
    }
}