/*
 * Interface file for the contact resolution system for particles.
 *
 */

#ifndef PCONTACTS_H
#define PCONTACTS_H

#include "particle.h"

	// Forward declaration for use as friend class
    class ParticleContactResolver;

    /**
     * A Contact represents two objects in contact (in this case
     * ParticleContact representing two Particles). 
     */
    class ParticleContact
    {
        /**
         * The contact resolver object needs access to the contact to
         * manipulate it.
         */
        friend ParticleContactResolver;

    public:
        /**
         * Holds the particles that are involved in the contact. The
         * second of these can be NULL, for contacts with the scenery.
         */
        Particle* particle[2];

        /**
         * Holds the normal restitution coefficient at the contact.
         */
        float restitution;

        /**
         * Holds the direction of the contact in world coordinates.
         */
        Vector2 contactNormal;

        /**
         * Holds the depth of penetration at the contact.
		   Negative value: No interpenetration
		   Zero value: Objects are just barely touching
		   >0: Interpenetration needs to be resolved.

		   This value is calculated by the collision detection system,
		   as the calculation requires information specific to the geometries,
		   which is not needed at this level.
         */
        float penetration;


    protected:
        /**
         * Resolves this contact, for both velocity and interpenetration.
         */
        void resolve(float duration);

        /**
         * Calculates the separating velocity at this contact.
         */
        float calculateSeparatingVelocity() const;

    private:
        /**
         * Handles the impulse (change of momentum) calculations for this collision.
         */
        void resolveVelocity(float duration);

		
		//  Resolves interpenetration between the contacts
		void resolveInterpenetration();

		// Used to calculate how much we move each particle in the interpenetration resolution step.
		Vector2 particleMovements[2];

    };

    /**
     * The contact resolution routine for particle contacts. One
     * resolver instance can be shared for the whole simulation.
     */
    class ParticleContactResolver
    {
    protected:
        /**
         * Holds the number of iterations allowed.
         */
        unsigned iterations;

        /**
         * This is a performance tracking value - we keep a record
         * of the actual number of iterations used.
         */
        unsigned iterationsUsed;

    public:
        /**
         * Creates a new contact resolver.
         */
        ParticleContactResolver(unsigned iterations);

        /**
         * Sets the number of iterations that can be used.
         */
        void setIterations(unsigned iterations);

        /**
         * Resolves a set of particle contacts for both penetration
         * and velocity.
		 *
		 * Case of one object:
		 * Move the object "penetrationDepth" ammount.
		 *
		 * Case of two objects:
		 * Contact is resolved taking in to account the mass of the objects
		 * e.g. objects with large mass barely move, objects with small mass will be
		 * moved more.
		 *
		 * While the looping is likely to eventually settle into a correct answer,
		 * there's no telling how long this will take.
         *
        */
        void resolveContacts(ParticleContact *contactArray,
            unsigned numContacts,
            float duration);
    };

    /**
     * This is the basic polymorphic interface for contact generators
     * applying to particles.
     */
    class ParticleContactGenerator
    {
    public:
        /**
         * Fills the given contact structure with the generated
         * contact. 
         */
        virtual unsigned addContact(ParticleContact *contact,
                                    unsigned limit) const = 0;
    };



	

#endif // CONTACTS_H