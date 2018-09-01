/*
 * The Blob demo.
 *
 */
#include <gl/glut.h>
#include "app.h"
#include "coreMath.h"
#include "pcontacts.h"
#include "pworld.h"
#include <stdio.h>
#include <cassert>
#include <vector>
// Required for particle vector iterator
#include "particle.h"
#include <iostream>

// Number of platforms in environment (doesn't include border platforms)
#define numPlatforms 2
// Number of blobs to add to the environment.
#define numBlobs 25


// Definition of acceleration due to gravity
const Vector2 Vector2::GRAVITY = Vector2(0,-9.81);

class Sphere : public ParticleContactGenerator
{
public:
	// Particle this generator is associated with
	Particle* thisParticle;
	float radius;

	// Particles potentially colliding with this one;
	std::vector<Particle*> particles;

	// Method to determine how contact structures will be filled.
	virtual unsigned addContact(
		ParticleContact* contact,
		unsigned limit) const;
};

unsigned Sphere::addContact(ParticleContact* contact, unsigned limit) const{
	const static float restitution = 1.0f;
	unsigned used = 0;

	// Get position of this particle
	Vector2 position = thisParticle->getPosition();

	for (std::vector<Particle*>::const_iterator it = particles.begin(); it != particles.end(); it++){
		if (used > limit) {
			std::cout << "Sphere-Sphere contact generator: Contact limit used.";
			break;
		}

		Vector2 candidatePos = (*it)->getPosition();
		float candidateRadius = (*it)->getRadius();

		// Check for interpenetration:
		float spheresPosXSqrd = position.x - candidatePos.x;
		spheresPosXSqrd *= spheresPosXSqrd;
		float spheresPosYSqrd = position.y - candidatePos.y;
		spheresPosYSqrd *= spheresPosYSqrd;

		// Calculate and square the sum of the radii
		float sumRadiiSquared = radius + candidateRadius;
		// Square sum of radii to avoid square root (computationally cheaper)
		sumRadiiSquared *= sumRadiiSquared;

		// If distance between centres <= sum of radii, we have a collision (face-face contact).
		// Test 1: are spheres touching or intersecting?
		if ((spheresPosXSqrd + spheresPosYSqrd) <= sumRadiiSquared){
			std::cout << "Sphere-Sphere: Touch/intersection detected\n";
			// find vector between this object and the candidate object
			Vector2 dist = position - candidatePos;
			// Take magnitude of this distance (giving us the size vector between these objects)
			float size = dist.magnitude();
			// Get the contact normal (normal of the vector from centre of first object through centre of second)
			Vector2 normal = dist * (((float)1.0) / size);
			
			// The intersection may be noticable, populate contact structure
			// for collision processing
			contact->contactNormal = normal;
			contact->restitution = restitution;
			contact->particle[0] = thisParticle;
			contact->particle[1] = (*it);
			// Calculate and populate contact structure penetration value.
			contact->penetration = (radius + candidateRadius - size);
			contact->contactPoint = position + dist * (float)0.5;

			// Increment number of used contacts
			used++;	
			// Move to the next available contact object
			contact++;
		}
	}

	return used;
}

/**
 * Platforms are two dimensional lines on which 
 * particles can rest. Platforms are also contact generators for the physics.
 */

class Platform : public ParticleContactGenerator
{
public:
    Vector2 start;
    Vector2 end;
    /**
     * Holds a pointer to the particles we're checking for collisions with. 
     */
    Particle *particles;

    virtual unsigned addContact(
        ParticleContact *contact, 
        unsigned limit
        ) const;
};

unsigned Platform::addContact(ParticleContact *contact, 
                              unsigned limit) const
{
    
	const static float restitution = 0.8f;
	//const static float restitution = 1.0f;
	unsigned used = 0;
    
	for (int i = 0; i < numBlobs; i++){
		if (used > limit) {
			std::cout << "Platform contact generator: Contact limit used." << std::endl;
			break;
		}

		// Check for penetration
		// Get distance to particle from start point
		Vector2 toParticle = particles[i].getPosition() - start;
		Vector2 lineDirection = end - start;

		// Get projected distance from distance to particle multiplied
		// by the direction of the line
		float projected = toParticle * lineDirection;
		float platformSqLength = lineDirection.squareMagnitude();
		// Get the squared radius of the particle to aviod using square root in calculations
		float squareRadius = particles[i].getRadius()*particles[i].getRadius();

		if (projected <= 0)
		{
			// The blob is nearest to the start point
			if (toParticle.squareMagnitude() < squareRadius)
			{
				// We have a collision, populate the contact structure accordingly.
				contact->contactNormal = toParticle.unit();
				contact->restitution = restitution;
				contact->particle[0] = particles + i;
				contact->particle[1] = 0;
				contact->penetration = particles[i].getRadius() - toParticle.magnitude();
				used++;
				contact++;
			}

		}
		else if (projected >= platformSqLength)
		{

			// Update particle position with respect to the end point of the platform
			toParticle = particles[i].getPosition() - end;
			
			// The blob is nearest to the end point
			if (toParticle.squareMagnitude() < squareRadius)
			{
				// We have a collision, populate the contact structure accordingly.
				contact->contactNormal = toParticle.unit();
				contact->restitution = restitution;
				contact->particle[0] = particles + i;
				contact->particle[1] = 0;
				contact->penetration = particles[i].getRadius() - toParticle.magnitude();
				used++;
				contact++;
			}
		}
		else
		{
			// the blob is between the start and end points.
			float distanceToPlatform = toParticle.squareMagnitude() - projected*projected / platformSqLength;
			if (distanceToPlatform < squareRadius)
			{
				// We have a collision, populate the contact structure accordingly.
				Vector2 closestPoint = start + lineDirection*(projected / platformSqLength);

				contact->contactNormal = (particles[i].getPosition() - closestPoint).unit();
				contact->restitution = restitution;
				contact->particle[0] = particles + i;
				contact->particle[1] = 0;
				contact->penetration = particles[i].getRadius() - sqrt(distanceToPlatform);
				used++;
				contact++;
			}
		}
	}

    return used;
}

class NonConvexPoly : public ParticleContactGenerator
{
public:

	virtual unsigned addContact(
		ParticleContact *contact,
		unsigned limit
	) const;
};

unsigned NonConvexPoly::addContact(ParticleContact *contact,
									unsigned limit) const 
{
	const static float restitution = 0.8f;
	//const static float restitution = 1.0f;
	unsigned used = 0;



	return used;
}


class BlobDemo : public Application
{
	Particle *blobs;

    Platform *platforms;
	Sphere *spheres;

	// Holds all contact generators and particle contacts in this
	// simulation world
    ParticleWorld world;

public:
    /** Creates a new demo object. */
    BlobDemo();
    virtual ~BlobDemo();

    /** Returns the window title for the demo. */
    virtual const char* getTitle();

    /** Display the particles. */
    virtual void display();

    /** Update the particle positions. */
    virtual void update();
	
};

// Method definitions
BlobDemo::BlobDemo():world(10, 10)
{
	width = 400; height = 400; 
	nRange = 100.0;

    // Create the blob storage
    //blob = new Particle;
	blobs = new Particle[numBlobs];

	// Create the platforms (4 for box + user given numPlatforms)
	platforms = new Platform[4+numPlatforms];

	// Create platform box
	// numPos determines height and width of the box.
	float numPos = 98.0f;
	float numNeg = numPos*-1.0f;

	platforms[0].start = Vector2(numNeg, numPos);
	platforms[0].end = Vector2(numPos, numPos);

	platforms[1].start = platforms[0].end;
	platforms[1].end = Vector2(numPos, numNeg);

	platforms[2].start = platforms[1].end;
	platforms[2].end = Vector2(numNeg, numNeg);

	platforms[3].start = platforms[2].end;
	platforms[3].end = Vector2(numNeg, numPos);

	if (numPlatforms > 0) {
		platforms[4].start = Vector2(-80, 30);
		platforms[4].end = Vector2(-10, 15);
		if (numPlatforms > 1) {
			platforms[5].start = Vector2(80, 30);
			platforms[5].end = Vector2(10, 15);
		}
	}

	// Register all blobs with all platform contact generators,
	// and all contact generators with the particle world.
	for (int i = 0; i < 4+numPlatforms; i++){
		platforms[i].particles = blobs;
		world.getContactGenerators().push_back(platforms + i);
	}

	// One sphere contact generator per blob.
	spheres = new Sphere[numBlobs];

	// Make blobs
	float mass = 1.0f;
	float radius = 2.0f;
	float offset = radius;

	for (int i = 0; i < numBlobs; i++){
		blobs[i].setPosition(-80.0f+offset, 70.0f);
		blobs[i].setVelocity(80.0f, 0.0f);
		// Use damping to simulate drag force (cheaper
		// than calculating a force)
		blobs[i].setDamping(0.8f);
		blobs[i].setAngularDamping(0.8f);
		// Apply acceleration due to gravity directly
		// (This could be added as a force)
		blobs[i].setAcceleration(Vector2::GRAVITY * 20.0f);
		blobs[i].setAngularVelocity(10.0f);
		blobs[i].setAngularAcceleration(0.0f);
		blobs[i].setMass(mass);
		blobs[i].setRadius(radius);
		blobs[i].setOrientation(0);
		blobs[i].clearAccumulators();

		// Set position and radius for this blob's contact generator
		//spheres[i].position = blobs[i].getPosition();
		spheres[i].radius = blobs[i].getRadius();

		world.getParticles().push_back(blobs + i);
		offset += 5.0f;
		//mass += mass+mass;
		//radius += radius;
	}

	// Register contacts between spheres
	for (int i = 0; i < numBlobs; i++){
		for (int j = 0; j < numBlobs; j++){
			if (i == j){
				// Pass particle reference to sphere
				spheres[i].thisParticle = &blobs[i];
			}
			// Don't have a sphere check for contacts against its self!
			if (j != i){

				spheres[i].particles.push_back(blobs + j);
			}
		}
		// Add this contact generator to world's contact generators.
		world.getContactGenerators().push_back(spheres + i);
	}
}


BlobDemo::~BlobDemo()
{
    delete blobs;
}

void BlobDemo::display()
{
  Application::display();
  glBegin(GL_LINES);
  glColor3f(0, 1, 1);

  for (int i = 0; i < 4+numPlatforms; i++){
	  const Vector2 &p0 = platforms[i].start;
	  const Vector2 &p1 = platforms[i].end;

	  glVertex2f(p0.x, p0.y);
	  glVertex2f(p1.x, p1.y);
  }
   glEnd();

   int r = 1.0f;
   int g = 0.0f;
   int b = 00.0f;

   for (int i = 0; i < numBlobs; i++){

	   const Vector2 &p = blobs[i].getPosition();
	   const float radius = blobs[i].getRadius();
	   // Convert orientation to degrees and store it
	   const float &orientation = blobs[i].getOrientation() * 180/3.1459;
	   glPushMatrix();
	   glLoadIdentity();
	   // Position the sphere
	   glTranslatef(p.x, p.y, 0);
	   glRotatef(orientation, 0, 0, 1.0f);

	   //glColor3f(r, g, b);
	   //glBegin(GL_POLYGON);
	   //glVertex3f(2.0, 4.0, 0.0);
	   //glVertex3f(8.0, 4.0, 0.0);
	   //glVertex3f(8.0, 6.0, 0.0);
	   //glVertex3f(2.0, 6.0, 0.0);
	   //glEnd();

	   // Draw the sphere
	   glColor3f(r, g, b);
	   glutSolidSphere(blobs[i].getRadius(), 12, 12);

	   // Draw a black line from sphere centre to top of sphere
	   glColor3f(0.0f, 0.0f, 0.0f);
	   glBegin(GL_LINES);
	   glVertex2f(p.x, p.y);
	   glVertex2f(p.x, p.y + radius);
	   glEnd();

	   glPopMatrix();

	   if (r != 0){
		   r = 0;
		   g = 1;
	   }
	   else if (g != 0){
		   g = 0;
		   b = 1;
	   }
	   else if (b != 0){
		   b = 0;
		   r = 1;
	   }
   }

	glutSwapBuffers();
}

void BlobDemo::update()
{
    // Recenter the axes
	float duration = timeinterval/1000;
    // Run the simulation
    world.runPhysics(duration);

    Application::update();
}

const char* BlobDemo::getTitle()
{
    return "Blob Demo";
}

/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
    return new BlobDemo();
}
