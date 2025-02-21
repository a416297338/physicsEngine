#include "collision/ManifoldFactory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "collision/Collider.hpp"
#include "collision/Manifold.hpp"
#include "collision/Simplex.hpp"

namespace stw
{
Manifold algo::FindCircleCircleManifold(
	const CircleCollider* a, const Transform* ta,
	const CircleCollider* b, const Transform* tb)
{
	Vector2 aPos = a->center + ta->position;
	Vector2 bPos = b->center + tb->position;

	const float aRadius = a->radius * ta->scale.Major();
	const float bRadius = b->radius * tb->scale.Major();

	const Vector2 aToB = bPos - aPos;
	const Vector2 bToA = aPos - bPos;

	if (aToB.Magnitude() > aRadius + bRadius) return Manifold::Empty();

	aPos += aToB.NewMagnitude(aRadius);
	bPos += bToA.NewMagnitude(bRadius);

	const Vector2 collisionPointsDistance = bPos - aPos;

	return {
		aPos,
		bPos,
		collisionPointsDistance.Normalized(),
		collisionPointsDistance.Magnitude()
	};
}

Manifold algo::FindCircleBoxManifold(
	const CircleCollider* a,
	const Transform* ta,
	const BoxCollider* b,
	const Transform* tb
)
{
	return FindBoxCircleManifold(b, tb, a, ta);
}

Manifold algo::FindBoxCircleManifold(const BoxCollider* a, const Transform* ta, const CircleCollider* b,
	const Transform* tb)
{
	return Gjk(a, ta, b, tb);
}

Manifold algo::FindBoxBoxManifold(
	const BoxCollider* a,
	const Transform* ta,
	const BoxCollider* b,
	const Transform* tb
)
{
	return Gjk(a, ta, b, tb);
	//return Sat(a, ta, b, tb);
}

Manifold algo::FindAabbAabbManifold(
	const AabbCollider* a, const Transform* ta,
	const AabbCollider* b, const Transform* tb)
{
	const Vector2 transformedCenterA = ta->position + a->center;
	const float aScaledHWidth = a->halfWidth * ta->scale.x;
	const float aScaledHHeight = a->halfHeight * ta->scale.y;

	const Vector2 transformedCenterB = tb->position + b->center;
	const float bScaledHWidth = b->halfWidth * tb->scale.x;
	const float bScaledHHeight = b->halfHeight * tb->scale.y;

	const Vector2 aToB = transformedCenterB - transformedCenterA;
	const float xOverlap = aScaledHWidth + bScaledHWidth - std::abs(aToB.x);

	// Overlap test on x axis
	if (xOverlap <= 0.0f) return Manifold::Empty();

	const float yOverlap = aScaledHHeight + bScaledHHeight - std::abs(aToB.y);

	// Overlap test on y axis
	if (yOverlap <= 0.0f) return Manifold::Empty();

	// Find out which axis is axis of least penetration
	if (xOverlap > yOverlap)
	{
		// Point towards B knowing that aToB points from A to B
		Vector2 normal;

		float aY;
		float bY;
		if (aToB.y > 0.0f)
		{
			normal = Vector2(0.0f, -1.0f);
			aY = transformedCenterA.y + aScaledHHeight;
			bY = transformedCenterB.y - bScaledHHeight;
		}
		else
		{
			normal = Vector2(0.0f, 1.0f);
			aY = transformedCenterA.y - aScaledHHeight;
			bY = transformedCenterB.y + bScaledHHeight;
		}

		float x;
		if (aToB.x > 0.0f)
		{
			x = transformedCenterA.x + aScaledHWidth - xOverlap / 2.0f;
		}
		else
		{
			x = transformedCenterA.x - aScaledHWidth + xOverlap / 2.0f;
		}

		return { {x, aY},{x, bY}, normal, yOverlap };
	}

	// Point towards B knowing that aToB points from A to B
	Vector2 normal;

	float y;
	if (aToB.y > 0.0f)
	{
		y = transformedCenterA.x + aScaledHWidth - xOverlap / 2.0f;
	}
	else
	{
		y = transformedCenterA.x - aScaledHWidth + xOverlap / 2.0f;
	}

	float aX;
	float bX;
	if (aToB.x > 0.0f)
	{
		normal = Vector2(-1.0f, 0.0f);
		aX = transformedCenterA.x + aScaledHWidth;
		bX = transformedCenterB.x - bScaledHWidth;
	}
	else
	{
		normal = Vector2(1.0f, 0.0f);
		aX = transformedCenterA.x - aScaledHWidth;
		bX = transformedCenterB.x + bScaledHWidth;
	}
	return { {aX, y}, {bX, y}, normal, xOverlap };
}

Manifold algo::FindAabbCircleManifold(
	const AabbCollider* a, const Transform* ta,
	const CircleCollider* b, const Transform* tb)
{
	// Apply the transform to the AabbCollider
	const Vector2 aabbCenter = ta->position + a->center;
	const float scaledHWidth = a->halfWidth * ta->scale.x;
	const float scaledHHeight = a->halfHeight * ta->scale.y;

	// Apply the transform to the circle collider
	const Vector2 circleCenter = tb->position + b->center;
	const float scaledRadius = b->radius * tb->scale.Major();

	const Vector2 aabbToCircle = circleCenter - aabbCenter;

	Vector2 clampedPoint;

	// Clamp point to the edge of the AABB
	clampedPoint.x = std::clamp(aabbToCircle.x, -scaledHWidth, scaledHWidth);
	clampedPoint.y = std::clamp(aabbToCircle.y, -scaledHHeight, scaledHHeight);

	bool isCircleCenterInside = false;

	// If they're equal, the center of the circle is inside the AABB
	if (clampedPoint == aabbToCircle)
	{
		isCircleCenterInside = true;

		// We still want one point on the side of the AABB, so we find the nearest border, and clamp on it
		const float distToPosWidth = std::abs(scaledHWidth - clampedPoint.x);
		const float distToNegWidth = std::abs(-scaledHWidth - clampedPoint.x);
		const float distToPosHeight = std::abs(scaledHHeight - clampedPoint.y);
		const float distToNegHeight = std::abs(-scaledHHeight - clampedPoint.y);

		const float smallest = std::min({ distToPosWidth, distToNegWidth, distToPosHeight, distToNegHeight });

		if (smallest == distToPosWidth) // NOLINT(clang-diagnostic-float-equal)
		{
			clampedPoint.x = scaledHWidth;
		}
		else if (smallest == distToNegWidth) // NOLINT(clang-diagnostic-float-equal)
		{
			clampedPoint.x = -scaledHWidth;
		}
		else if (smallest == distToPosHeight) // NOLINT(clang-diagnostic-float-equal)
		{
			clampedPoint.y = scaledHHeight;
		}
		else if (smallest == distToNegHeight) // NOLINT(clang-diagnostic-float-equal)
		{
			clampedPoint.y = -scaledHHeight;
		}
	}

	// Put the point in "world space" because it was relative to the center
	const Vector2 closestPointOnAabb = aabbCenter + clampedPoint;

	const Vector2 circleToClosestPoint = closestPointOnAabb - circleCenter;

	// Distance between the circle center and the clamped point
	const float squaredDistance = circleToClosestPoint.SqrMagnitude();

	if (!isCircleCenterInside && squaredDistance >= scaledRadius * scaledRadius) return Manifold::Empty();

	// This is the collision point around the circle
	Vector2 aroundCirclePoint = circleToClosestPoint.NewMagnitude(scaledRadius);
	if (isCircleCenterInside)
	{
		aroundCirclePoint = -aroundCirclePoint;
	}

	Vector2 worldAroundCirclePoint = aroundCirclePoint + circleCenter;

	const Vector2 diff = worldAroundCirclePoint - closestPointOnAabb;

	//return { closestPointOnAabb, worldAroundCirclePoint, circleToClosestPoint, depth };
	return { closestPointOnAabb, worldAroundCirclePoint, diff.Normalized(), diff.Magnitude() };
}

Manifold algo::FindCircleAabbManifold(const CircleCollider* a, const Transform* ta, const AabbCollider* b,
	const Transform* tb)
{
	return FindAabbCircleManifold(b, tb, a, ta).Swaped();
}

Vector2 algo::Support(
	const Collider* colliderA,
	const Transform* transformA,
	const Collider* colliderB,
	const Transform* transformB,
	const Vector2& direction
)
{
	return colliderA->FindFurthestPoint(transformA, direction) -
		colliderB->FindFurthestPoint(transformB, -direction);
}

Manifold algo::Gjk(
	const Collider* colliderA,
	const Transform* transformA,
	const Collider* colliderB,
	const Transform* transformB
)
{
	Vector2 direction = Vector2::Normalize(transformB->position - transformA->position);

	Vector2 support = Support(
		colliderA,
		transformA,
		colliderB,
		transformB,
		direction);

	Simplex points;
	points.PushFront(support);

	// New direction is towards the origin
	direction = Vector2::Normalize(-support);

	while (true)
	{
		support = Support(colliderA, transformA, colliderB, transformB, direction);

		if (support.Dot(direction) <= 0)
		{
			return Manifold::Empty();
		}

		points.PushFront(support);

		if (NextSimplex(points, direction))
		{
			return Epa(points, colliderA, transformA, colliderB, transformB);
		}
	}
}

bool algo::NextSimplex(const Simplex& points, Vector2& direction)
{
	switch (points.Size())
	{
	case 2:
		return Line(points, direction);
	case 3:
		return Triangle(points, direction);
	default:
		return false;
	}
}

bool algo::SameDirection(const Vector2 direction, const Vector2 ao)
{
	return direction.Dot(ao) > 0;
}

bool algo::Line(const Simplex& points, Vector2& direction)
{
	const Vector2 a = points[0];
	const Vector2 b = points[1];
	const Vector2 ab = Vector2::Normalize(b - a);
	const Vector2 ao = Vector2::Normalize(-a);
	direction = Vector2::TripleProduct(ab, ao, ab);

	return false;
}

bool algo::Triangle(const Simplex& points, Vector2& direction)
{
	const Vector2 a = points[0];
	const Vector2 b = points[1];
	const Vector2 c = points[2];

	const Vector2 ab = Vector2::Normalize(b - a);
	const Vector2 ac = Vector2::Normalize(c - a);
	const Vector2 ao = Vector2::Normalize(-a);

	//const Vector2 tripple = TripleProduct(ac, ac, ab);
	const Vector2 abf = Vector2::TripleProduct(ac, ab, ab);
	const Vector2 acf = Vector2::TripleProduct(ab, ac, ac);

	if (SameDirection(abf, ao))
	{
		direction = abf;
		return false;
		//return Line(points = { a, b }, direction);
	}
	if (SameDirection(acf, ao))
	{
		direction = acf;
		return false;
		//return Line(points = { a, c }, direction);
	}
	return true;
}

Manifold algo::Epa(
	const Simplex& simplex,
	const Collider* colliderA,
	const Transform* transformA,
	const Collider* colliderB,
	const Transform* transformB
)
{
	std::vector polytope(simplex.Begin(), simplex.End());

	Vector2 minNormal;
	float minDistance = std::numeric_limits<float>::infinity();
	std::size_t minIndex = 0;

	std::size_t iterations = 0;
	constexpr std::size_t maxIter = 30;
	while (minDistance == std::numeric_limits<float>::infinity()) // NOLINT(clang-diagnostic-float-equal)
	{
		if (iterations++ > maxIter)
		{
			break;
		}

		for (std::size_t i = 0; i < polytope.size(); i++)
		{
			const std::size_t j = (i + 1) % polytope.size();

			Vector2 vertexI = polytope[i];
			Vector2 vertexJ = polytope[j];

			Vector2 ij = vertexJ - vertexI;

			Vector2 normal = ij.NegativePerpendicular().Normalized();
			float distance = normal.Dot(vertexI);

			if (distance < 0.0f)
			{
				distance *= -1.0f;
				normal *= -1.0f;
			}

			if (distance < minDistance)
			{
				minDistance = distance;
				minNormal = normal;
				minIndex = i;
			}
		}

		Vector2 support = Support(colliderA, transformA, colliderB, transformB, minNormal);
		const float sDistance = minNormal.Dot(support);

		if (std::abs(sDistance - minDistance) > 0.001f)
		{
			minDistance = std::numeric_limits<float>::infinity();
			const auto convIndex = static_cast<long long>(minIndex);
			polytope.insert(polytope.begin() + convIndex + 1ll, support);
		}
	}

	if (minDistance == std::numeric_limits<float>::infinity()) // NOLINT(clang-diagnostic-float-equal)
	{
		return Manifold::Empty();
	}

	return { minNormal, minDistance };
}
struct Axis {
	Axis(Vector2 A, Vector2 B, Vector2 C) {
		posA = A; posB = B; normal = C;
	}
	Vector2 posA;
	Vector2 posB;
	Vector2 normal;
};
struct VclipData{
	Vector2 reference_edge_posA;
	Vector2 reference_edge_posB;
	Vector2 incident_edge_posA;
	Vector2 incident_edge_posB;
	Vector2 axisnomal;
	float depth;
};

std::vector<Axis> GetAxes(const std::vector<Vector2>& vertices)
{
	std::vector<Axis> axes;

	for (std::size_t i = 0; i < vertices.size(); ++i)
	{
		Vector2 p1 = vertices[i];
		Vector2 p2 = vertices[(i + 1) % vertices.size()];
		Vector2 edge = p1 - p2;
		Vector2 normal = edge.PositivePerpendicular().Normalized();
		axes.push_back(Axis(p1, p2, normal));
	}
	return axes;
}
bool Project(Vector2 pos, float r, const std::vector<Vector2>& vertices, VclipData& ref)
{
	float p_max_depth = -10000;
	int p_max_ver_idx = 0;
	int p_vertices_size = vertices.size();
	for (int i = 0; i < p_vertices_size; i++)
	{
		float depth = r - (vertices[i] - pos).Magnitude();
		if (depth > p_max_depth)
		{
			p_max_depth = depth;
			p_max_ver_idx = i;
		}
	}

	ref.depth = p_max_depth;
	ref.axisnomal = (pos - vertices[p_max_ver_idx]).Normalized();
	ref.incident_edge_posA = vertices[p_max_ver_idx];
	return p_max_depth > 0;
}
bool Project(const std::vector<Axis>& axises, Vector2 pos, float r, VclipData& ref)
{
	float g_min_depth = 10000;
	for (const auto& axis : axises)
	{
		int p_max_ver_idx = 0;
		float depth = (pos - axis.posA).Dot(axis.normal) + r ;
		if (depth < g_min_depth)
		{
			if (depth < 0)
				return false;
			g_min_depth = depth;
			ref.depth = depth;
			ref.axisnomal = axis.normal;
			ref.incident_edge_posA = pos- axis.normal * r;
		}

	}

	return g_min_depth > 0 && g_min_depth != 10000;;
}

bool Project(const std::vector<Axis>& axises, const std::vector<Vector2>& vertices, VclipData& ref)
{
	float g_min_depth = 10000;
	for (const auto& axis : axises)
	{
		float p_max_depth = -10000;
		int p_max_ver_idx = 0;
		int p_vertices_size = vertices.size();
		for (int i = 0; i < p_vertices_size; i++)
		{
			float depth = (vertices[i] - axis.posA).Dot(axis.normal);
			if (depth > p_max_depth)
			{
				p_max_depth = depth;
				p_max_ver_idx = i;
			}
		}
		if (p_max_depth < g_min_depth)
		{
			if (p_max_depth < 0)
				return false;
			g_min_depth = p_max_depth;
			ref.reference_edge_posA = axis.posA;
			ref.reference_edge_posB = axis.posB;
			ref.axisnomal = axis.normal;
			ref.depth = g_min_depth;
			if (fabs((vertices[p_max_ver_idx] - vertices[(p_max_ver_idx - 1 + p_vertices_size) % p_vertices_size]).Dot(axis.normal)) < \
				fabs((vertices[p_max_ver_idx] - vertices[(p_max_ver_idx + 1) % p_vertices_size]).Dot(axis.normal)))
			{
				ref.incident_edge_posA = vertices[p_max_ver_idx];
				ref.incident_edge_posB = vertices[(p_max_ver_idx - 1 + p_vertices_size) % p_vertices_size];
			}
			else
			{
				ref.incident_edge_posA = vertices[(p_max_ver_idx + 1) % p_vertices_size];
				ref.incident_edge_posB = vertices[p_max_ver_idx];
			}
		}
	}
	return g_min_depth > 0 && g_min_depth != 10000;
}
bool Project(Vector2 pos1, float r1,Vector2 pos2, float r2, VclipData& ref)
{
	ref.depth = r1 + r2 - (pos1 - pos2).Magnitude() ;
	ref.axisnomal = (pos2-pos1).Normalized();
	ref.incident_edge_posA = pos2 - ref.axisnomal*r2;
	return ref.depth > 0;
}
bool algo::SatForCircle(
	const Collider* colliderA, const Transform* transformA,
	const Collider* colliderB, const Transform* transformB,
	std::vector <Manifold>& foldVector, bool swap
)
{
	RegularPolygonCollider* colliderAR = (RegularPolygonCollider*)colliderA;
	RegularPolygonCollider* colliderBR = (RegularPolygonCollider*)colliderB;
	std::vector<Vector2> verticesA;
	std::vector<Vector2> verticesB;
	colliderA->GetTransformedVertices(*transformA, verticesA);
	colliderB->GetTransformedVertices(*transformB, verticesB);
	VclipData refData;
	bool swap2 = false;
	if (verticesA.size() == 1 && verticesB.size() == 1)
	{
		RegularPolygonCollider* colliderAR = (RegularPolygonCollider*)colliderA;
		RegularPolygonCollider* colliderBR = (RegularPolygonCollider*)colliderB;
		bool collisionA = Project(verticesA[0], colliderAR->r, verticesB[0], colliderBR->r, refData);
		if (!collisionA)
			return false;
	}
	else{
	VclipData refDataA;
	VclipData refDataB;
	const auto axesB = GetAxes(verticesB);
	bool collisionA = Project(verticesA[0], colliderAR->r, verticesB, refDataA);
	bool collisionB = Project(axesB, verticesA[0], colliderAR->r, refDataB);
	if (!collisionA && !collisionB)
		return false;
	refData = refDataA.depth > refDataB.depth ? refDataA : refDataB;
	swap2 = refDataA.depth > refDataB.depth;
	}
	if (swap2 ^ swap)
		foldVector.push_back(Manifold(refData.incident_edge_posA, refData.incident_edge_posA - refData.axisnomal * refData.depth, -refData.axisnomal, refData.depth));
	else
		foldVector.push_back(Manifold(refData.incident_edge_posA - refData.axisnomal * refData.depth, refData.incident_edge_posA, refData.axisnomal, refData.depth));
	return true;

}
bool algo::Sat(
	const Collider* colliderA, const Transform* transformA,
	const Collider* colliderB, const Transform* transformB, 
	std::vector <Manifold>& foldVector
)
{
	Vector2 smallestAxis;
	std::vector<Vector2> verticesA;
	std::vector<Vector2> verticesB;
	colliderA->GetTransformedVertices(*transformA, verticesA);
	colliderB->GetTransformedVertices(*transformB, verticesB);
	if(verticesB.size() == 1 or verticesA.size() == 1)
	{
		if(verticesB.size() == 1)
			return SatForCircle(colliderB, transformB, colliderA, transformA, foldVector ,true);
		else
			return SatForCircle(colliderA, transformA, colliderB, transformB, foldVector, false);
	}

	const auto axesA = GetAxes(verticesA);
	const auto axesB = GetAxes(verticesB);
	VclipData refDataA;
	VclipData refDataB;
	bool collisionA = Project(axesA, verticesB, refDataA);
	bool collisionB = Project(axesB, verticesA, refDataB);
	if (!collisionA || !collisionB)
		return false;
	VclipData refData = refDataA.depth < refDataB.depth ? refDataA : refDataB;
	Vector2 ref_edge_array[2]{ refData.reference_edge_posA, refData.reference_edge_posB };
	Vector2 colVector[2]{ refData.incident_edge_posA , refData.incident_edge_posB };
	for (int i = 0; i < 2; i++)
	{
		Vector2 ref_edge_A = ref_edge_array[i];
		Vector2 ref_edge_B = ref_edge_array[i^1];
		float sinProjectionR = (ref_edge_B - ref_edge_A).Cross(refData.axisnomal);
		float sinProjectionA = (refData.incident_edge_posA - ref_edge_A).Cross(refData.axisnomal);
		float sinProjectionB = (refData.incident_edge_posB - ref_edge_A).Cross(refData.axisnomal);
		float Da = fabs(sinProjectionA);
		float Db = fabs(sinProjectionB);
		if (sinProjectionA * sinProjectionB < 0)
		{
			Vector2 intersection = (Da) / (Db + Da) * refData.incident_edge_posB + (Db) / (Db + Da) * refData.incident_edge_posA;
			if (sinProjectionA * sinProjectionR < 0)
				colVector[0] = intersection;
			else
				colVector[1] = intersection;
		}
	}
	float depthA = (colVector[0] - refData.reference_edge_posA).Dot(refData.axisnomal);
	float depthB = (colVector[1] - refData.reference_edge_posA).Dot(refData.axisnomal);
	float dpe = std::max(depthA, depthB);
	for(auto pcol : colVector)
	{
		float depth = (pcol - refData.reference_edge_posA).Dot(refData.axisnomal);
		if (depth > 0 )
		{
			if(refDataA.depth < refDataB.depth)
				foldVector.push_back(Manifold(pcol, pcol - refData.axisnomal * depth, -refData.axisnomal, depth));
			else
				foldVector.push_back(Manifold(pcol - refData.axisnomal * depth, pcol, refData.axisnomal, depth));
		}
		if(depth>5)
		{
			int w = 100;
		}
	}
	return true;
}

/*
std::vector<Axis> GetAxes(const std::array<Vector2, 4>& vertices ,int n)
{
	std::vector<Axis> axes;

	for (std::size_t i = 0; i < n; ++i)
	{
		Vector2 p1 = vertices[i];
		Vector2 p2 = vertices[(i + 1) % vertices.size()];
		Vector2 edge = p1 - p2;
		Vector2 normal = edge.PositivePerpendicular().Normalized();
		axes.push_back(Axis(p1, p2, normal));
	}

	return axes;
}
bool Project(const std::vector<Axis>& axises, const std::array<Vector2, 4>& vertices , VclipData& ref)
{
	float g_min_depth = 10000;
	for (const auto& axis : axises)
	{
		float p_max_depth = -10000;
		int p_max_ver_idx = 0;
		for(int i = 0 ;i< vertices.size();i++)
		{
			float depth = (vertices[i] - axis.posA).Dot(axis.normal);
			if (depth > p_max_depth)
			{
				p_max_depth = depth;
				p_max_ver_idx = i;
			}
		}
		if (p_max_depth < g_min_depth)
		{
			if (p_max_depth < 0)
				return false;
			g_min_depth = p_max_depth;
			ref.reference_edge_posA = axis.posA;
			ref.reference_edge_posB = axis.posB;
			ref.axisnomal = axis.normal;
			ref.depth = g_min_depth;
			if (fabs((vertices[p_max_ver_idx] - vertices[(p_max_ver_idx - 1 + 4) % 4]).Dot(axis.normal)) < \
				fabs((vertices[p_max_ver_idx] - vertices[(p_max_ver_idx + 1) % 4]).Dot(axis.normal)))
			{
				ref.incident_edge_posA = vertices[p_max_ver_idx];
				ref.incident_edge_posB = vertices[(p_max_ver_idx - 1 + 4) % 4];
			}
			else
			{
				ref.incident_edge_posA = vertices[(p_max_ver_idx + 1) % 4];
				ref.incident_edge_posB = vertices[p_max_ver_idx];
			}
		}
	}

	return g_min_depth > 0 && g_min_depth != 10000;
}
bool algo::Sat(
	const Collider* colliderA, const Transform* transformA,
	const Collider* colliderB, const Transform* transformB,
	std::vector <Manifold>& foldVector
)
{
	Vector2 smallestAxis;
	const auto verticesA = colliderA->GetTransformedVertices(*transformA);
	const auto verticesB = colliderB->GetTransformedVertices(*transformB);
	const auto axesA = GetAxes(verticesA, 4);
	const auto axesB = GetAxes(verticesB, 4);
	VclipData refDataA;
	VclipData refDataB;
	bool collisionA = Project(axesA, verticesB, refDataA);
	bool collisionB = Project(axesB, verticesA, refDataB);
	if (!collisionA || !collisionB)
		return false;
	collisionA = Project(axesA, verticesB, refDataA);
	collisionB = Project(axesB, verticesA, refDataB);
	VclipData refData = refDataA.depth < refDataB.depth ? refDataA : refDataB;
	Vector2 ref_edge_array[2]{ refData.reference_edge_posA, refData.reference_edge_posB };
	Vector2 colVector[2]{ refData.incident_edge_posA , refData.incident_edge_posB };
	for (int i = 0; i < 2; i++)
	{
		Vector2 ref_edge_A = ref_edge_array[i];
		Vector2 ref_edge_B = ref_edge_array[i ^ 1];
		float sinProjectionR = (ref_edge_B - ref_edge_A).Cross(refData.axisnomal);
		float sinProjectionA = (refData.incident_edge_posA - ref_edge_A).Cross(refData.axisnomal);
		float sinProjectionB = (refData.incident_edge_posB - ref_edge_A).Cross(refData.axisnomal);
		float Da = fabs(sinProjectionA);
		float Db = fabs(sinProjectionB);
		if (sinProjectionA * sinProjectionB < 0)
		{
			Vector2 intersection = (Da) / (Db + Da) * refData.incident_edge_posB + (Db) / (Db + Da) * refData.incident_edge_posA;
			if (sinProjectionA * sinProjectionR < 0)
				colVector[0] = intersection;
			else
				colVector[1] = intersection;
		}
	}
	float depthA = (colVector[0] - refData.reference_edge_posA).Dot(refData.axisnomal);
	float depthB = (colVector[1] - refData.reference_edge_posA).Dot(refData.axisnomal);
	float dpe = std::max(depthA, depthB);
	for (auto pcol : colVector)
	{
		float depth = (pcol - refData.reference_edge_posA).Dot(refData.axisnomal);
		if (depth > 0)
		{
			if (refDataA.depth < refDataB.depth)
				foldVector.push_back(Manifold(pcol, pcol - refData.axisnomal * depth, -refData.axisnomal, depth));
			else
				foldVector.push_back(Manifold(pcol - refData.axisnomal * depth, pcol, refData.axisnomal, depth));
		}
		if (depth > 5)
		{
			int w = 100;
		}
	}
	return true;
}
*/
}
