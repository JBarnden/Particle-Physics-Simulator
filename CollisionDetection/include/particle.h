    /**
     * A particle is the simplest object that can be simulated in the
     * physics system.

     */

#ifndef PARTICLE_H
#define PARTICLE_H

#include "coreMath.h"

    class Particle
    {
    protected:
	
	// Storing the inverse mass as opposed to mass is a convenient way of simulating
	// objects of infinite mass (set inverse mas to 0), as well as preventing the simulation of
	// objects of 0 mass (which would have an infinite inverse mass).
	float inverseMass;
	float damping;
	float radius;

    Vector2 position;
	Vector2 velocity;
	Vector2 acceleration;

	// Force accumulator vector
	Vector2 forceAccum;
    
	public:
		void integrate(float duration);
		void setMass(const float mass);
		float getMass() const;
		void setInverseMass(const float inverseMass);
		float getInverseMass() const;
		bool hasFiniteMass() const;

	    void setDamping(const float damping);
        float getDamping() const;


        void setPosition(const float x, const float y);
		void setPosition(const Vector2 &position);
		Vector2 getPosition() const;
		void getPosition(Vector2 *position) const;
		
		void setRadius(const float r);
		float getRadius() const;

		void setVelocity(const Vector2 &velocity);
		void setVelocity(const float x, const float y);
		Vector2 getVelocity() const;
		void getVelocity(Vector2 *velocity) const;

		void setAcceleration(const Vector2 &acceleration);
		void setAcceleration(const float x, const float y);
		Vector2 getAcceleration() const;

		void clearAccumulator();
		/*
			Add force to be applied to next iteration only.
		*/
		void addForce(const Vector2 &force);
	
       };

	#endif // 


