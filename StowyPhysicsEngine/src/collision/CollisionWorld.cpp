#include "collision/CollisionWorld.hpp"

#include <algorithm>

#include "collision/Collision.hpp"
#include <ranges>
#include <collision/ManifoldFactory.hpp>
#include <map>
namespace stw
{
CollisionWorld::CollisionWorld()
	: CollisionWorld({}, {}) {}

CollisionWorld::CollisionWorld(
	std::unordered_map<std::uint64_t, CollisionBody*> bodies,
	std::vector<Solver*> solvers
)
	: _bodies(std::move(bodies)),
	_solvers(std::move(solvers)),
	_grid(-100, 100, -100, 100, 10) {}

void CollisionWorld::AddCollisionBody(CollisionBody* body)
{
	if (!body) return;

	_bodies.insert({ body->id, body });
}

void CollisionWorld::RemoveCollisionBody(const CollisionBody* body)
{
	if (!body) return;

	_bodies.erase(body->id);
}

void CollisionWorld::AddSolver(Solver* solver)
{
	_solvers.push_back(solver);
}

void CollisionWorld::RemoveSolver(Solver* solver)
{
	if (!solver) return;

	const auto itr = std::ranges::find(_solvers, solver);

	if (itr == _solvers.end()) return;

	_solvers.erase(itr);
}

void CollisionWorld::SetCollisionCallback(const std::function<void(Collision&, float)>& callback)
{
	_onCollision = callback;
}

void CollisionWorld::SolveCollisions(std::vector<Collision>& collisions, const float deltaTime) const
{
	for (Solver* solver : _solvers)
	{
		solver->Solve(collisions, deltaTime);
	}
}

void CollisionWorld::SendCollisionCallbacks(std::vector<Collision>& collisions, const float deltaTime) const
{
	for (Collision& collision : collisions)
	{
		if (_onCollision)
			_onCollision(collision, deltaTime);

		collision.bodyA->OnCollision(collision, deltaTime);
		collision.bodyB->OnCollision(collision, deltaTime);
	}
}

void CollisionWorld::ResolveCollisions(const float deltaTime)
{
	// Vector for the collisions that have been detected
	std::vector<Collision> collisions;

	// Vector for the collisions that have been caused by trigger colliders
	std::vector<Collision> triggers;

	if (useSpacePartitioning)
	{
		// Update the grid and find the object that can collide together
		_grid.Update(_bodies);
		const auto collisionPairs = _grid.GetCollisionPairs();
		for (auto& [firstId, secondId] : collisionPairs)
		{
			FindCollisions(firstId, secondId, collisions, triggers);
		}
	}
	else
	{
		std::unordered_multimap<std::uint64_t, std::uint64_t> checkedCollisions;
		for (auto firstId : _bodies | std::views::keys)
		{
			for (auto secondId : _bodies | std::views::keys)
			{
				if (firstId == secondId) continue;

				auto [firstChecked, secondChecked] = checkedCollisions.equal_range(firstId);
				bool isContained = false;
				for (auto& i = firstChecked; i != secondChecked; ++i)
				{
					if (i->second == secondId)
					{
						isContained = true;
						break;
					}
				}

				if (!isContained)
				{
					checkedCollisions.emplace(firstId, secondId);
					FindCollisions(firstId, secondId, collisions, triggers);
				}
			}
		}
	}
	//WarmStaring(collisions, deltaTime);
	SolveCollisions(collisions, deltaTime);
	SendCollisionCallbacks(collisions, deltaTime);
	
	SendCollisionCallbacks(triggers, deltaTime);
	//old_collisions = collisions;

}

void CollisionWorld::FindCollisions(const std::uint64_t firstId, const std::uint64_t secondId,
	std::vector<Collision>& collisions, std::vector<Collision>& triggers)
{
	CollisionBody* a = _bodies[firstId];
	CollisionBody* b = _bodies[secondId];

	if (!a->Col() || !b->Col()) return;
	std::vector<Manifold> foldVector;
	bool isCollision = stw::algo::Sat(a->Col(),
		a->Trans(),
		b->Col(),
		b->Trans(), foldVector);

	if (!isCollision) return;
	if (a->IsTrigger() || b->IsTrigger())
	{
		triggers.emplace_back(a, b, foldVector, std::tuple(Vector2(0, 0), 0, 0));
	}
	else
	{
		collisions.emplace_back(a, b, foldVector, std::tuple(Vector2(0, 0),0, 0));
	}
}
bool Find(Collision& A, Collision& B)
{
	if (A.bodyA->id == B.bodyA->id && A.bodyB->id == B.bodyB->id)
	{
		if (A.manifold.size() == B.manifold.size())
		{
			for (int i = 0; i < A.manifold.size(); i++)
			{
				if ((A.manifold[i].a - B.manifold[i].a).Magnitude() + (A.manifold[i].b - B.manifold[i].b).Magnitude() + (A.manifold[i].normal - B.manifold[i].normal).Magnitude() > 0.0001)
					return false;
			}
			return true;
		}
	}
	return false;

}
void CollisionWorld::WarmStaring(std::vector<Collision>& collisions,float deltaTime)
{
	for (auto &collision : collisions)
	{
		for (auto& collision_old : old_collisions)
		{
			if (Find(collision, collision_old))
				collision.impluse = collision_old.impluse;
		}
	}
	for (Solver* solver : _solvers)
	{
		solver->WarmingStartingSolve(collisions, deltaTime);
	}

}
}
