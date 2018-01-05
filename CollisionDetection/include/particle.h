    /**
     * A particle is the simplest object that can be simulated in the
     * physics system.
	 *
     **/

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
	// Damping applied to linear motion.
	float damping;
	// Damping applied to angular motion.
	float angularDamping;
	float radius;

	// Position of the particle in world space
    Vector2 position;
	// Velocity of the particle with respect to the x and y axes.
	Vector2 velocity;
	// Acceleration of the particle with respect to axes.
	Vector2 acceleration;

	float angularAcceleration;
	// Angular velocity determines the rate of change of orientation.
	float angularVelocity;


	// Orientation of the object (which way it's facing) in radians.
	// Where 0 orients the object directly up the y axis.
	// positive orientation suggests clockwise rotation, and negative suggests
	// counter-clockwise rotation.
	float orientation;

	// Force accumulator vector
	Vector2 forceAccum;

	/*
		Torque accumulator vector

		Torque can be thought of as the rotational equivalent to force,
		which impacts the change in angular velocity.

		Like the force accumulator, the cumulative torque is only applied
		to the next iteraction
	*/
	float torqueAccum;
    
	public:
		void integrate(float duration);
		void setMass(const float mass);
		float getMass() const;
		void setInverseMass(const float inverseMass);
		float getInverseMass() const;
		bool hasFiniteMass() const;

	    void setDamping(const float damping);
        float getDamping() const;

		void setAngularDamping(const float damping);
		float getAngularDamping() const;


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

		void setAngularVelocity(const float &velocity);
		float getAngularVelocity() const;

		void setAcceleration(const Vector2 &acceleration);
		void setAcceleration(const float x, const float y);
		Vector2 getAcceleration() const;

		void setAngularAcceleration(const float &acceleration);
		float getAngularAcceleration() const;

		void setOrientation(const float &orientation);
		float getOrientation() const;

		void clearAccumulators();
		/*
			Add force to be applied to next iteration only.
		*/
		void addForce(const Vector2 &force);

		/*
			Add torque to be applied to next iteration only.
		*/
		void addTorque(const float &torque);
	
       };

	#endif // 


