#include "collision/Manifold.hpp"

#include <ostream>

namespace stw
{
Manifold::Manifold(const Vector2& a, const Vector2& b, const Vector2& normal, const float depth)
    : a(a), b(b), normal(normal), depth(depth), hasCollision(true) {}

Manifold::Manifold(const Vector2& normal, const float depth)
    : Manifold(Vector2(), Vector2(), normal, depth) {}
Manifold::Manifold() {}

Manifold Manifold::Swaped() const
{
    Manifold copy = *this;
    copy.a = b;
    copy.b = a;
    copy.normal = -normal;
    return copy;
}
bool Manifold:: operator ==(const Manifold& Manifold) const
{
    return double((a - Manifold.a).SqrMagnitude()) + (b - Manifold.b).SqrMagnitude() + (normal - Manifold.normal).SqrMagnitude() < 0.01 ||
        double((a - Manifold.b).SqrMagnitude()) + (b - Manifold.a).SqrMagnitude() + (normal + Manifold.normal).SqrMagnitude() < 0.01;
}
std::ostream& operator<<(std::ostream& os, const Manifold& manifold)
{
    os << "{a: " << manifold.a << ", b: " << manifold.b <<
        ", normal: " << manifold.normal << ", depth: " <<
        manifold.depth << ", hasCollision: " << manifold.hasCollision << "}";
    return os;
}
}
