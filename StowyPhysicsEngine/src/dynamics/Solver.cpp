#include "dynamics/Solver.hpp"

#include "collision/Collision.hpp"

#include "dynamics/Rigidbody.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
namespace stw
{
	void ImpulseSolver::WarmingStartingSolve(std::vector<Collision>& collisions, float deltaTime)
	{
		for (auto& [bodyA, bodyB, manifolds, old_impulse] : collisions)
		{
			Rigidbody* aBody = (Rigidbody*)bodyA;
			Rigidbody* bBody = (Rigidbody*)bodyB;
			std::get<0>(old_impulse) = std::get<0>(old_impulse) * 0.5;
			std::get<1>(old_impulse) = std::get<1>(old_impulse) * 0.5;
			std::get<2>(old_impulse) = std::get<2>(old_impulse) * 0.5;
			if (aBody->IsDynamic())
			{
				aBody->UpdateImpulse(-std::get<0>(old_impulse), -std::get<1>(old_impulse));
			}
			if (bBody->IsDynamic())
			{
				bBody->UpdateImpulse(std::get<0>(old_impulse), std::get<2>(old_impulse));
			}
		}
	}
	void ImpulseSolver::Solve(std::vector<Collision>& collisions, float deltaTime)
	{
		int Times = 100;
		for (int i = 1; i <= Times; i++)
		{
			for (auto& [bodyA, bodyB, manifolds, worked_impulse] : collisions)
			{
				for(const auto& manifold: manifolds)
				{
					Rigidbody* aBody = (Rigidbody*)bodyA;
					Rigidbody* bBody = (Rigidbody*)bodyB;
					// manifold 上一步求出的数据
					Vector2 ra = manifold.a - aBody->GetCenter();
					Vector2 rb = manifold.b - bBody->GetCenter();
					Vector2 aVel = aBody->Velocity();
					Vector2 bVel = bBody->Velocity();

					Vector2 relativeVelocity = bVel - aVel + rb.Cross(bBody->AngularVelocity()) - ra.Cross(aBody->AngularVelocity());

					float Penetration = -manifold.depth;

					// JV1
					float velocityAlongNormal = relativeVelocity.Dot(manifold.normal);

					// Do not resolve if velocities are separating
					const float aInvMass = aBody->IsDynamic() ? aBody->InvMass() : 0.0f;
					const float bInvMass = bBody->IsDynamic() ? bBody->InvMass() : 0.0f;
					const float aInvInertia = aBody->IsDynamic() ? aBody->InvInertia() : 0.0f;
					const float bInvInertia = bBody->IsDynamic() ? bBody->InvInertia() : 0.0f;
					//float effectMass = aInvMass + bInvMass; 
					float effectMass = aInvMass + bInvMass + fabs((manifold.normal * ra.Cross(manifold.normal)).Cross(ra)) * aInvInertia +
						fabs((manifold.normal * rb.Cross(manifold.normal)).Cross(rb)) * bInvInertia;
					// Impulse
					const float baumgarte = 5 * Penetration + 0.01 * (rb.Cross(bBody->AngularVelocity()) - ra.Cross(aBody->AngularVelocity())).Dot(manifold.normal);
					//const float e = std::min(aBody ? aBody->Restitution() : 1.0f, bBody ? bBody->Restitution() : 1.0f);
					float lambda = -(float(i) / Times) * (velocityAlongNormal + baumgarte) / (effectMass);
					if (lambda < 0)
						continue;
					//float all_lambda = lambda + std::get<0>(worked_impulse).Dot(manifold.normal.Normalized());
					
					Vector2 impulse = lambda * manifold.normal;
					
					/*
					if (aBody ? aBody->IsKinematic() : false)
					{
						aBody->UpdateImpulse(-impulse, ra.Cross(-impulse));
					}
					if (bBody ? bBody->IsKinematic() : false)
					{
						bBody->UpdateImpulse(impulse, rb.Cross(impulse));
					}
					impulse = Vector2(0, 0);*/
					//relativeVelocity = bVel - aVel + rb.Cross(bBody->AngularVelocity()) - ra.Cross(aBody->AngularVelocity());
					
					const auto tangent = manifold.normal.PositivePerpendicular();
					effectMass = aInvMass + bInvMass + fabs((tangent * ra.Cross(tangent)).Cross(ra)) * aInvInertia +
						fabs((tangent * rb.Cross(tangent)).Cross(rb)) * bInvInertia;
					const float velocityAlongTangent = relativeVelocity.Dot(tangent);
					float lambda_T = -(float(i) / Times) * (velocityAlongTangent) / effectMass;
					lambda_T = fmin(lambda * 0.3f, fmax(lambda * -0.3f, lambda_T));
					impulse += lambda_T * tangent;
					//std::get<0>(worked_impulse) += impulse;
					//std::get<1>(worked_impulse) += ra.Cross(impulse);
					//std::get<2>(worked_impulse) += rb.Cross(impulse);
					if (aBody->IsDynamic())
					{
						aBody->UpdateImpulse(-impulse, ra.Cross(-impulse));
					}
					if (bBody->IsDynamic())
					{
						bBody->UpdateImpulse(impulse, rb.Cross(impulse));
					}
				}
			}
		}
	}

	void SmoothPositionSolver::Solve(std::vector<Collision>& collisions, float deltaTime)
	{
		return;
		/*
		for (const auto& [bodyA, bodyB, points] : collisions)
		{
			// ReSharper disable CppCStyleCast
			Rigidbody* aBody = bodyA->IsDynamic() ? (Rigidbody*)bodyA : nullptr;
			Rigidbody* bBody = bodyB->IsDynamic() ? (Rigidbody*)bodyB : nullptr;
			// ReSharper restore CppCStyleCast

			const float aInvMass = aBody ? aBody->InvMass() : 0.0f;
			const float bInvMass = bBody ? bBody->InvMass() : 0.0f;

			Vector2 resolution = points.b - points.a;

			constexpr float slop = 0.01f;
			constexpr float percent = 0.8f;

			const Vector2 correction = points.normal * percent
				* std::max(resolution.Magnitude() - slop, 0.0f)
				/ (aInvMass + bInvMass);

			if (aBody ? aBody->IsKinematic() : false)
			{
				const Vector2 deltaA = aInvMass * correction;
				aBody->Trans()->position -= deltaA;
			}

			if (bBody ? bBody->IsKinematic() : false)
			{
				const Vector2 deltaB = bInvMass * correction;
				bBody->Trans()->position += deltaB;
			}
		}
		*/
	}
}