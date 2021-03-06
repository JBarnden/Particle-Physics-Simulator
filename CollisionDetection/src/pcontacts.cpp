
#include <float.h>
#include <pcontacts.h>
#include <iostream>


// Contact implementation
void ParticleContact::resolve(float duration)
{
	// Resolve velocities for the pair of particles (or single particle)
    resolveVelocity(duration);
	resolveAngularVelocity(duration);
	// Resolve interpenetration for the pair of particles (or single particle)
	resolveInterpenetration();
}

// Returns the velocity in the direction of the contact
float ParticleContact::calculateSeparatingVelocity() const
{
    Vector2 relativeVelocity = particle[0]->getVelocity();
    if (particle[1]) relativeVelocity -= particle[1]->getVelocity();
    return relativeVelocity * contactNormal;
}

void ParticleContact::resolveVelocity(float duration)
{
    // Find the velocity in the direction of the contact
    float separatingVelocity = calculateSeparatingVelocity();

    // Check if it needs to be resolved
    if (separatingVelocity > 0)
    {
        // The contact is either separating, or stationary.
        return;
    }

    // Calculate the new separating velocity
    float newSepVelocity = -separatingVelocity * restitution;


	// Check velocity build-up due to acceleration only.
	// (this is to do with better handling resting particles making contact),
	// Trying to prevent them from "vibrating" and potentially jumping.
	Vector2 velocityAccel = particle[0]->getAcceleration();
	// If another particle is involved (i.e. not a collision with the outer scene border)
	if (particle[1]) velocityAccel -= particle[1]->getAcceleration();
	float accCausedSepVel = velocityAccel * contactNormal * duration;

	// If we have a closing velocity due to acceleration build-up,
	// we need to discount this from separating velocity.
	if (accCausedSepVel < 0){
		newSepVelocity += restitution * accCausedSepVel;
		// Ensure we didn't remove more than we should have.
		if (newSepVelocity < 0) newSepVelocity = 0;
	}

	// Doing this we're essentially applying a small change in velocity at each frame
	// to prevent the increase in velocity that can cause particles to settle in to each other
	// over time.
	float deltaVelocity = newSepVelocity - separatingVelocity;

    // We apply the change in velocity to each object in proportion to
    // their inverse mass (i.e. those with lower inverse mass [higher
    // actual mass] get less change in velocity)..
    float totalInverseMass = particle[0]->getInverseMass();
    if (particle[1]) totalInverseMass += particle[1]->getInverseMass();

    // If all particles have infinite mass, then impulses have no effect
    if (totalInverseMass <= 0) return;

    // Calculate the impulse to apply
    float impulse = deltaVelocity / totalInverseMass;

    // Find the amount of impulse per unit of inverse mass
    Vector2 impulsePerIMass = contactNormal * impulse;

    // Apply impulses: they are applied in the direction of the contact,
    // and are proportional to the inverse mass.
    particle[0]->setVelocity(particle[0]->getVelocity() +
        impulsePerIMass * particle[0]->getInverseMass()
        );
    if (particle[1])
    {
        // Particle 1 goes in the opposite direction
        particle[1]->setVelocity(particle[1]->getVelocity() +
            impulsePerIMass * -particle[1]->getInverseMass()
            );
    }
}

void ParticleContact::resolveAngularVelocity(float duration) {
	// Check if angular veolcity needs to be resolved.
	if (particle[0])

	// Resolve angular velocity by reversing it.
	particle[0]->setAngularVelocity(particle[0]->getAngularVelocity()*-1);

	if (particle[1]) {
		particle[1]->setAngularVelocity(particle[1]->getAngularVelocity()*-1);
	}
	
}

void ParticleContact::resolveInterpenetration(){
	// If the objects aren't penetrating, skip.
	if (penetration <= 0) return;

	// Movement of the objects is based on their inverse mass.
	float totalInverseMass = particle[0]->getInverseMass();
	// If we're checking two particles (as upposed to a window boundary check),
	// then we must account for the mass of the other object
	if (particle[1]) totalInverseMass += particle[1]->getInverseMass();

	// We don't touch particles of infinite mass
	if (totalInverseMass <= 0) return;

	Vector2 movePerIMass = contactNormal * (penetration / totalInverseMass);

	// Apply the penetration resolution.
	//particle[0]->setPosition(particle[0]->getPosition() +
	//	movePerIMass * particle[0]->getInverseMass());
	//if (particle[1])
	//{
	//	particle[1]->setPosition(particle[1]->getPosition() +
	//		movePerIMass * particle[1]->getInverseMass());
	//}

	particleMovements[0] = movePerIMass * particle[0]->getInverseMass();
	// If we're not just checking against boundaries (not 2 particles involved)
	if (particle[1]){
		particleMovements[1] = movePerIMass * -particle[1]->getInverseMass();
	}
	else{
		particleMovements[1].clear();
	}

	// Resolve interpenetration
	particle[0]->setPosition(particle[0]->getPosition() + particleMovements[0]);
	if (particle[1]){
		particle[1]->setPosition(particle[1]->getPosition() + particleMovements[1]);
	}

	// Set penetration to 0 now we've resolved it.
	penetration = 0;
}

ParticleContactResolver::ParticleContactResolver(unsigned iterations)
:
iterations(iterations)
{
}

void ParticleContactResolver::setIterations(unsigned iterations)
{
    ParticleContactResolver::iterations = iterations;
}

void ParticleContactResolver::resolveContacts(ParticleContact *contactArray,
                                              unsigned numContacts,
                                              float duration)
{
    unsigned i;


	/*
		We apply a limit to the number of iterations (collision resolutions) to prevent the same
		contacts being resolved over and over again (e.g. for resting objects with similtaneous contacts).

		Each contact resolved one at a time (simplification of real world).  As opposed to honouring sequential series,
		or performing simultaneous ressolution of multiple contacts.  This approach is far less complex and less error prone,
		although order of contact resolution is important for believable results.

		This implementation prioritises the resolution of contacts with the lowest separating velocity first
	*/
    iterationsUsed = 0;
    while(iterationsUsed < iterations)
    {
		/*
			Order of the algorithm:
			1. Calculate separating velocity of each contact, keeping track of the contact with
			the lowest (most negative value).

			2. If lowest sepVel >= 0 , we're done, exit.

			3. Process the collision response algorithm for contact with lowest sepVel

			4. If we still have iterations left return to step 1.
		*/

        // Find the contact with the largest closing velocity (speed);
        float max = DBL_MAX;
        unsigned maxIndex = numContacts;
        for (i = 0; i < numContacts; i++)
        {
			// Get the separating velocity of this contact
            float sepVel = contactArray[i].calculateSeparatingVelocity();
            // If separating velocity is less than the maximum separating velocity,
			// and the separating velocity is less than 0 or penetration > 0
			if (sepVel < max &&
                (sepVel < 0 || contactArray[i].penetration > 0))
            {
				// Store this separating velocity and the index of the contact
				// in the array.
                max = sepVel;
                maxIndex = i;
            }
        }
         //Do we have anything worth resolving?
        if (maxIndex == numContacts) break;

        // Resolve this contact
        contactArray[maxIndex].resolve(duration);

        iterationsUsed++;
    }
	if (iterationsUsed == iterations) std::cout << "Resolver: All iterations used." << std::endl;

}