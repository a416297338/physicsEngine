#include "AabbBox.hpp"

#include "MathUtils.hpp"

#include "math/Vector2.hpp"

AabbBox::AabbBox(stw::DynamicsWorld& dynWorld, const stw::Vector2 size, const stw::Vector2 pos, const bool takesGravity)
    : Entity(dynWorld, pos),
      _shape(SpeVecToSfml(size)),
      _collider(std::make_unique<stw::AabbCollider>())
{
    const stw::Vector2 halfSize = size / 2.0f;
    _collider->halfWidth = halfSize.x;
    _collider->halfHeight = halfSize.y;
    _rb->SetCollider(_collider.get());
    _rb->SetTakesGravity(takesGravity);
    if (takesGravity)
        _rb->SetRotation(0.0f);
    _shape.setOrigin(halfSize.x, halfSize.y);
    _dynWorld.AddRigidbody(RigidBody());
    _shape.setFillColor(sf::Color::Black);
}

sf::RectangleShape& AabbBox::Shape()
{
    return _shape;
}

void AabbBox::SetColor(const sf::Color color)
{
    _shape.setFillColor(color);
}


void AabbBox::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    states.transform *= getTransform();
    target.draw(_shape, states);
}

ConvexBox::ConvexBox(stw::DynamicsWorld& dynWorld, float r ,int vec, const stw::Vector2 pos, const bool takesGravity)
    : Entity(dynWorld, pos),
    _shape(vec),
    _collider(std::make_unique<stw::RegularPolygonCollider>())
{
    _collider->r = r;
    _collider->side = vec;
    _collider->invInertia = 12 * (1 - std::cos(2 * PI / vec)) / (r * std::sin(PI / vec) * (2 + std::cos(2 * PI / vec)));
    _rb->SetCollider(_collider.get());
    _rb->SetTakesGravity(takesGravity);
    if (takesGravity)
        _rb->SetRotation(0.0f);
    std::vector<stw::Vector2> Out;
    _collider->GetTransformedVertices(stw::Transform(), Out);
    for (int i = 0; i < vec; i++)
    {
        _shape.setPoint(i, sf::Vector2f(Out[i].x - Out[0].x, -(Out[i].y- Out[0].y)));
    }
    _shape.setOrigin(r*std::sin(PI / vec),-r*std::cos(PI / vec));
    
    _dynWorld.AddRigidbody(RigidBody());
    _shape.setFillColor(sf::Color::Black);
}

sf::ConvexShape& ConvexBox::Shape()
{
    return _shape;
}

void ConvexBox::SetColor(const sf::Color color)
{
    _shape.setFillColor(color);
}


void ConvexBox::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    states.transform *= getTransform();
    target.draw(_shape, states);
}
