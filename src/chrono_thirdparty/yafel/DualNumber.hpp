#ifndef YAFEL_DUALNUMBER_HPP
#define YAFEL_DUALNUMBER_HPP

#include <cmath>

namespace yafel {

template<typename T>
class DualNumber
{
public:
    T first;
    T second;

    DualNumber() : DualNumber(0, 0)
    {}

    DualNumber(T v1) : DualNumber(v1, 0)
    {}

    DualNumber(T v1, T v2) : first(v1), second(v2)
    {}

    // arithmetic operator overloading (+, -, *, /)
    DualNumber<T> &operator+=(const DualNumber<T> &rhs)
    {
        second += rhs.second;
        first += rhs.first;
        return *this;
    }

    DualNumber<T> &operator-=(const DualNumber<T> &rhs)
    {
        second -= rhs.second;
        first -= rhs.first;
        return *this;
    }

    DualNumber<T> &operator*=(const DualNumber<T> &rhs)
    {
        second = second * rhs.first + first * rhs.second;
        first = first * rhs.first;
        return *this;
    }

    DualNumber<T> &operator/=(const DualNumber<T> &rhs)
    {
        second = (second * rhs.first - first * rhs.second) / (rhs.first * rhs.first);
        first = first / rhs.first;
        return *this;
    }

    DualNumber<T> operator+(DualNumber<T> rhs) const
    {
        return (rhs += *this);
    }

    DualNumber<T> operator-(const DualNumber<T> &rhs) const
    {
        DualNumber<T> copy(*this);
        return (copy -= rhs);
    }

    DualNumber<T> operator*(DualNumber<T> rhs) const
    {
        return (rhs *= *this);
    }

    DualNumber<T> operator/(const DualNumber<T> &rhs) const
    {
        DualNumber<T> copy(*this);
        return (copy /= rhs);
    }

    // comparison operators
    bool operator>(const DualNumber<T> &rhs) const
    {
        return (first > rhs.first);
    }

    bool operator<(const DualNumber<T> &rhs) const
    {
        return (rhs > *this);
    }

    // unary operator-()
    DualNumber<T> operator-() const
    {
        return DualNumber<T>(-first, -second);
    }
};

template<typename T>
DualNumber<T> make_dual(T lhs)
{
    return DualNumber<T>(lhs);
}

template<typename T, typename L>
DualNumber<T> operator+(L lhs, DualNumber<T> rhs)
{
    return (make_dual(static_cast<T>(lhs)) + rhs);
}

template<typename T, typename L>
DualNumber<T> operator-(L lhs, DualNumber<T> rhs)
{
    return (make_dual(static_cast<T>(lhs)) - rhs);
}

template<typename T, typename L>
DualNumber<T> operator*(L lhs, DualNumber<T> rhs)
{
    return (make_dual(static_cast<T>(lhs)) * rhs);
}

template<typename T, typename L>
DualNumber<T> operator/(L lhs, DualNumber<T> rhs)
{
    return (make_dual(static_cast<T>(lhs)) / rhs);
}

// More Useful Functions
template<typename T>
DualNumber<T> sin(DualNumber<T> x)
{
    using std::sin;
    using std::cos;
    return DualNumber<T>(sin(x.first), x.second * cos(x.first));
}

template<typename T>
DualNumber<T> cos(DualNumber<T> x)
{
    using std::sin;
    using std::cos;
    return DualNumber<T>(cos(x.first), -x.second * sin(x.first));
}

template<typename T>
DualNumber<T> exp(DualNumber<T> x)
{
    using std::exp;
    return DualNumber<T>(exp(x.first), x.second * exp(x.first));
}


} //end namespace yafel

#endif
