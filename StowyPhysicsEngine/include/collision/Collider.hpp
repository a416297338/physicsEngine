/**
 * @file Collider.hpp
 * @author Fabian Huber (fabian.hbr@protonmail.ch)
 * @brief Contains all the Colliders classes.
 * @version 1.0
 * @date 05.07.2022
 *
 * @copyright SAE (c) 2022
 *
 */
#pragma once

#include <array>
#include <vector>
#include "Manifold.hpp"
#include "Projection.hpp"
#include "Transform.hpp"

#include "math/Vector2.hpp"

namespace stw
{
class CircleCollider;
class BoxCollider;
class AabbCollider;

/**
 * \brief A pure virtual struct that represents colliders.
 */
enum ColliderType { None, Box, Aabb, Circle};
class Collider
{
public:
	Collider() = default;
	Collider(Collider&& col) noexcept = default;
	virtual ~Collider() = default;
	Collider& operator=(Collider&& col) = default;
	Collider& operator=(const Collider& col) = default;
	Collider(const Collider& col) = default;
	std::vector<Vector2> vertices;
	/*
	* \brief The center of the collider.
	*/
	Vector2 center{};
	
	/**
	 * \brief Tests the collision against a generic collider.
	 * \param transform The transform of this collider.
	 * \param collider The collider to collide with.
	 * \param colliderTransform The transform of the collider to collide with.
	 * \return The manifold of that collision.
	 */
	virtual Manifold TestCollision(
		const Transform* transform,
		const Collider* collider,
		const Transform* colliderTransform
	)  const { return Manifold(); };

	/**
	 * \brief Tests the collision against a box collider.
	 * \param transform The transform of this collider.
	 * \param collider The box collider to collide with.
	 * \param boxTransform The transform of the collider to collide with.
	 * \return The manifold of that collision.
	 */
	virtual Manifold TestCollision(
		const Transform* transform,
		const BoxCollider* collider,
		const Transform* boxTransform
	) const {
		return Manifold();
	};

	/**
	 * \brief Tests the collision against a circle collider.
	 * \param transform The transform of this collider.
	 * \param collider The circle collider to collide with.
	 * \param circleTransform The transform of the collider to collide with.
	 * \return The manifold of that collision.
	 */
	virtual Manifold TestCollision(
		const Transform* transform,
		const CircleCollider* collider,
		const Transform* circleTransform
	) const {
		return Manifold();
	};

	/**
	 * \brief Tests the collision against a aabb collider.
	 * \param transform The transform of this collider.
	 * \param collider The circle collider to collide with.
	 * \param aabbTransform The transform of the collider to collide with.
	 * \return The manifold of that collision.
	 */
	virtual Manifold TestCollision(
		const Transform* transform,
		const AabbCollider* collider,
		const Transform* aabbTransform
	) const {
		return Manifold();
	};

	/**
	 * \brief Find the furthest point in the specified direction.
	 * \param transform The transform of this collider.
	 * \param direction Direction in which to find the furthest point.
	 * \return The furthest point.
	 */
	[[nodiscard]] virtual Vector2 FindFurthestPoint(
		const Transform* transform,
		const Vector2& direction
	) const {
		return Vector2();
	};

	/**
	 * \brief Gets the size of the box that surrounds the collider.
	 * \return The bounding box of the collider.
	 */
	[[nodiscard]] virtual Vector2 GetBoundingBoxSize() const {
		return Vector2();
	};
	/*
	* ... kun
	*/
	inline virtual Vector2 GetPositivePerpendicular(const int idx) const { return (vertices[(long long (idx) + 1) % vertices.size()] - vertices[idx]).PositivePerpendicular().Normalized(); }
	inline virtual Vector2 GetEdge(const int idx) const { return (vertices[(long long(idx) + 1) % vertices.size()] - vertices[idx]); }
	virtual std::array<Vector2, 4> GetTransformedVertices(const Transform& transform) const { return {}; }
	virtual void GetTransformedVertices(const Transform& transform, std::vector<Vector2>& Out) const  {}
	virtual float GetInvInertia() const { return 0; }

};

/**
 * \brief A rotatable box collider.
 */
class BoxCollider final : public Collider
{
public:
	/**
	 * \brief Half of the width of the box.
	 */
	float halfWidth = 0;
	/**
	 * \brief Half of the height of the box.
	 */
	float halfHeight = 0;

	Manifold TestCollision(
		const Transform* transform,
		const Collider* collider,
		const Transform* colliderTransform
	) const override;

	Manifold TestCollision(
		const Transform* transform,
		const BoxCollider* collider,
		const Transform* boxTransform
	) const override;

	Manifold TestCollision(
		const Transform* transform,
		const CircleCollider* collider,
		const Transform* circleTransform
	) const override;

	Manifold TestCollision(const Transform* transform, const AabbCollider* collider,
		const Transform* circleTransform) const override;

	[[nodiscard]] Vector2 FindFurthestPoint(
		const Transform* transform,
		const Vector2& direction
	) const override;

	/**
	 * \brief Gets the transformed vertices of the box.
	 * \param transform The transform to apply to the collider.
	 * \return The transform vertices of the box.
	 */
	[[nodiscard]] std::array<Vector2, 4> GetTransformedVertices(const Transform& transform) const;

	/**
	 * \brief Gets the untransformed vertices of the box.
	 * \return The vertices of the box.
	 */
	[[nodiscard]] std::array<Vector2, 4> GetVertices() const;

	[[nodiscard]] static float Project(
		const Vector2& axis,
		const std::array<Vector2, 4>& vertices);
	[[nodiscard]] static std::array<Vector2, 4> GetAxes(const std::array<Vector2, 4>& vertices);

	[[nodiscard]] Vector2 GetBoundingBoxSize() const override;
};


/**
 * \brief A circle collider.
 */
class CircleCollider final : public Collider
{
public:
	/**
	 * \brief Radius of the circle.
	 */
	float radius = 0;

	Manifold TestCollision(
		const Transform* transform,
		const Collider* collider,
		const Transform* colliderTransform
	) const override;

	Manifold TestCollision(
		const Transform* transform,
		const BoxCollider* collider,
		const Transform* boxTransform
	) const override;

	Manifold TestCollision(
		const Transform* transform,
		const CircleCollider* collider,
		const Transform* circleTransform
	) const override;

	Manifold TestCollision(const Transform* transform, const AabbCollider* collider,
		const Transform* aabbTransform) const override;

	[[nodiscard]] Vector2 FindFurthestPoint(
		const Transform* transform,
		const Vector2& direction
	) const override;
	void GetTransformedVertices(const Transform& transform, std::vector<Vector2>& Out) const;
	[[nodiscard]] Vector2 GetBoundingBoxSize() const override;

};

/**
 * \brief An Axis Aligned Bounding Box collider.
 */
class AabbCollider final : public Collider
{
public:
	/**
	 * \brief Half of the width of the box.
	 */
	float halfWidth = 0;
	/**
	 * \brief Half of the height of the box.
	 */
	float halfHeight = 0;
	
	Manifold TestCollision(const Transform* transform, const Collider* collider,
		const Transform* colliderTransform) const override;
	Manifold TestCollision(const Transform* transform, const BoxCollider* collider,
		const Transform* boxTransform) const override;
	Manifold TestCollision(const Transform* transform, const CircleCollider* collider,
		const Transform* circleTransform) const override;
	Manifold TestCollision(const Transform* transform, const AabbCollider* collider,
		const Transform* aabbTransform) const override;
	std::array<Vector2, 4> GetTransformedVertices(const Transform& transform) const;
	void GetTransformedVertices(const Transform& transform, std::vector<Vector2>& Out) const;
	virtual float GetInvInertia() const;
	[[nodiscard]] Vector2 FindFurthestPoint(const Transform* transform, const Vector2& direction) const override;
	[[nodiscard]] Vector2 GetBoundingBoxSize() const override;
};
class RegularPolygonCollider final : public Collider
{
public:
	/**
	 * \brief Half of the width of the box.
	 */
	float r = 0;
	/**
	 * \brief Half of the height of the box.
	 */
	int side = 0;
	float invInertia = 0;
	void GetTransformedVertices(const Transform& transform, std::vector<Vector2>& Out) const;
	virtual float GetInvInertia() const;
	[[nodiscard]] Vector2 FindFurthestPoint(const Transform* transform, const Vector2& direction) const override;
	[[nodiscard]] Vector2 GetBoundingBoxSize() const override;
};
}
