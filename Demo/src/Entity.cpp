#include "Entity.hpp"

#include <memory>

#include "MathUtils.hpp"

Entity::Entity(stw::DynamicsWorld& dynWorld)
    : Entity(dynWorld, stw::Vector2(0, 0)) {}

Entity::Entity(stw::DynamicsWorld& dynWorld, const stw::Vector2 pos)
    : _dynWorld(dynWorld), _rb(std::make_unique<stw::Rigidbody>())
{
    _rb->SetTransform({pos, stw::Vector2(1, 1), 0});
    // Makes the objects affected by gravity
    _rb->SetTakesGravity(true);
    // Indicates that the body is affected by collision responses
    _rb->SetIsKinematic(true);
    _rb->SetRestitution(0.5f);
}

Entity::~Entity()
{
    _dynWorld.RemoveCollisionBody(RigidBody());
}

stw::Rigidbody* Entity::RigidBody() const
{
    return _rb.get();
}

void Entity::Push(const stw::Vector2 force) const
{
    _rb->ApplyForce(force);
}

void Entity::Update(float deltaTime)
{
    setPosition(SpePosToSfml(_rb->Trans()->position));
    setScale(SpeVecToSfml(_rb->Trans()->scale));
    setRotation(SpeRotToSfml(_rb->Trans()->rotation));
}
