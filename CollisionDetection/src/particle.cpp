
#include "particle.h"
#include <math.h>
#include <assert.h>
#include <float.h>

// Update position and velocity of the particle based on the given duration.
void Particle::integrate(float duration)
{

	// We don't integrate things with zero mass.
   if (inverseMass <= 0.0f) return;

	assert(duration > 0.0);
	// update position based on linear velocity
	position.addScaledVector(velocity, duration);
	// update orientation based on angular velocity
	orientation += angularVelocity * duration;

	// Work out the linear acceleration from force
    Vector2 resultingAcc = acceleration;
    resultingAcc.addScaledVector(forceAccum, inverseMass);

	// Work out angular acceleration from force
	float angularAcc = angularAcceleration;
	angularAcc += torqueAccum;

	// Update linear velocity from the acceleration and impulse.
    velocity.addScaledVector(resultingAcc, duration);

	// Update angular velocity from impulse
	angularVelocity += angularAcc * duration;

	// Impose drag.
	velocity *= pow(damping, duration);
	angularVelocity *= pow(angularDamping, duration);

	// Clear accumulated forces and torques now they have been integrated.
    clearAccumulators();
}

void Particle::setMass(const float mass)
{
    assert(mass != 0);
    Particle::inverseMass = ((float)1.0)/mass;
}

float Particle::getMass() const
{
    if (inverseMass == 0) {
        return DBL_MAX;
    } else {
        return ((float)1.0)/inverseMass;
    }
}

void Particle::setInverseMass(const float inverseMass)
{
    Particle::inverseMass = inverseMass;
}

float Particle::getInverseMass() const
{
    return inverseMass;
}

bool Particle::hasFiniteMass() const
{
    return inverseMass >= 0.0f;
}


void Particle::setDamping(const float damping)
{
    Particle::damping = damping;
}

float Particle::getDamping() const
{
    return damping;
}

void Particle::setAngularDamping(const float damping) {
	angularDamping = damping;
}

float Particle::getAngularDamping() const {
	return angularDamping;
}

void Particle::setPosition(const float x, const float y)
{
    position.x = x;
    position.y = y;
}

void Particle::setPosition(const Vector2 &position)
{
	 Particle::position = position;
}


Vector2 Particle::getPosition() const
{
    return position;
}

void Particle::getPosition(Vector2 *position) const
{
    *position = Particle::position;
}

void Particle::setRadius(const float r)
{
    radius = r;
}

float Particle::getRadius() const
{
    return radius;
}


void Particle::setVelocity(const float x, const float y)
{
    velocity.x = x;
    velocity.y = y;
}

void Particle::setVelocity(const Vector2 &velocity)
{
    Particle::velocity = velocity;
}

Vector2 Particle::getVelocity() const
{
    return velocity;
}

void Particle::getVelocity(Vector2 *velocity) const
{
    *velocity = Particle::velocity;
}

void Particle::setAngularVelocity(const float &velocity) {
	Particle::angularVelocity = velocity;
}

float Particle::getAngularVelocity() const {
	return angularVelocity;
}

void Particle::setAcceleration(const Vector2 &acceleration)
{
    Particle::acceleration = acceleration;
}

void Particle::setAcceleration(const float x, const float y)
{
    acceleration.x = x;
    acceleration.y = y;
}

Vector2 Particle::getAcceleration() const
{
    return acceleration;
}

void Particle::setAngularAcceleration(const float &acceleration) {
	angularAcceleration = acceleration;
}

float Particle::getAngularAcceleration() const {
	return angularAcceleration;
}

void Particle::setOrientation(const float &orientation) {
	Particle::orientation = orientation;
}

float Particle::getOrientation() const {
	return orientation;
}

void Particle::clearAccumulators()
{
    forceAccum.clear();
	torqueAccum = 0;
}

void Particle::addForce(const Vector2 &force)
{
    forceAccum += force;
}

void Particle::addTorque(const float &torque) {
	torqueAccum += torque;
}
