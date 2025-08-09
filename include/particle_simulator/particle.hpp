#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <vector>

// particle class

class Particle {
public:
    /** @brief Current position of the particle in 2D space (x, y). */
    std::vector<float> position;
    /** @brief Current velocity of the particle in 2D space (vx, vy). */
    std::vector<float> velocity;
    /** @brief Current acceleration of the particle in 2D space (ax, ay). */
    std::vector<float> acceleration;
    /** @brief Particle radius in pixels. */
    float particle_radius;
    /** @brief Mass of particle. */
    float mass = 1.0; // hard coded but could be updated in the future, physics is not the main focus on this project.
    /** @brief Gravity applied to particle. */
    float gravity = 9.81;

    /**
     * @brief Default constructor sets all values to zero (except gravity and mass hardcoded for now).
     */
    Particle();

    /**
     * @brief Constructs a Particle with specified initial position, velocity, and radius.
     * 
     * @param initial_position Initial (x, y) coordinates of the particle.
     * @param initial_velocity Initial (x, y) velocity of the particle.
     * @param particle_radius Radius of the particle.
     */
    Particle(const std::vector<float> initial_position, const std::vector<float> initial_velocity, float particle_radius);

    /**
     * @brief Updates the particle position and velocity based on acceleration and elapsed time.
     * 
     * @param dt Time step (in seconds) to advance the simulation.
     */
    void update(const float dt);

    /**
     * @brief Checks and handles collisions with window boundary.
     * 
     * @param particle_radius Radius of the particle.
     * @param width Width of the bounding area.
     * @param height Height of the bounding area.
     * @param coef_restitution Coefficient of restitution.
     */
    void check_boundary(const float particle_radius, const float width, const float height,const float coef_restitution);

    


};

#endif 
