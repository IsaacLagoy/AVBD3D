#include "includes.h"

// copy operator
vec6& vec6::operator=(const vec6& vec) {
    if (&vec == this) return *this;

    linear = vec.linear;
    angular = vec.angular;

    if (hasNaN(linear)) throw std::runtime_error("vec6 = Linear component of copy has NaN");
    if (hasNaN(angular)) throw std::runtime_error("vec6 = Angular component of copy has NaN");

    return *this;
}

// inplace arithmetic
vec6& vec6::operator+=(const vec6& rhs) {

    print(linear);

    linear += rhs.linear;
    angular += rhs.angular;

    if (hasNaN(linear)) throw std::runtime_error("vec6 += Linear component of copy has NaN");
    if (hasNaN(angular)) throw std::runtime_error("vec6 += Angular component of copy has NaN");

    return *this;
}

// get item operators
float& vec6::operator[](int i) {
    if (i < 0 || i >= 6) {
        throw std::out_of_range("vec6 index out of range");
    }
    return i < 3 ? linear[i] : angular[i - 3];
}

const float& vec6::operator[](int i) const {
    if (i < 0 || i >= 6) {
        throw std::out_of_range("vec6 index out of range");
    }
    return i < 3 ? linear[i] : angular[i - 3];
}

// arithmetic operators
vec6 vec6::operator+(const vec6& rhs) const {
    return { linear + rhs.linear, angular + rhs.angular };
}

vec6 vec6::operator-(const vec6& rhs) const {
    return { linear - rhs.linear, angular - rhs.angular };
}

vec6 vec6::operator*(float rhs) const {
    return { rhs * linear, rhs * angular };
}

vec6 vec6::operator/(float rhs) const {
    if (rhs == 0.0f) throw std::runtime_error("Cannot divide by 0.");
    return { linear / rhs, angular / rhs };
}