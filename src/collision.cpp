#include "particle_simulator/particle.hpp"
#include <cmath>
#include <memory>


/**
 * @brief Checks if Particle A and B are in collision (overlapping) and returns True if in collision.
 * 
 * @param A address to particle which is to be checked in collision with B.
 * @param B address to particle which is to be checked in collision with A.
 */
bool check_collision(Particle& A, Particle& B){

    float dx = B.position[0]- A.position[0];
    float dy = B.position[1]- A.position[1];

    float distance_between_centers = sqrt( (dx*dx + dy*dy) );

    bool collision = distance_between_centers <= (A.particle_radius + B.particle_radius);

    return collision;

}

/**
 * @brief Updates dynamics of 2 particles in collision.
 * 
 * Uses 2D impulse physics to change velocity of 2 particles based on collision with one another.
 * 
 * @param A address to particle is in collision with particle B.
 * @param B address to particle is in collision with particle A.
 * @param coef_restitution Coefficient of restitution defining the transfer of momentum in the collision.
 */
void apply_collision(Particle& A, Particle& B, const float coef_restitution){

    // calculate collision normal vector
    std::vector<float> collision_normal(2);

    collision_normal[0] = B.position[0] - A.position[0];
    collision_normal[1] = B.position[1] - A.position[1];

    float normal_length = sqrt( collision_normal[0]*collision_normal[0] + collision_normal[1]*collision_normal[1]);
    // division by 0.0 leads to inf so just extra check
    if (normal_length == 0.0f) return;
    //normalize to unit vector
    collision_normal[0] /= normal_length;
    collision_normal[1] /= normal_length;

    std::vector<float> relative_velocity(2);

    relative_velocity[0] = B.velocity[0] - A.velocity[0];
    relative_velocity[1] = B.velocity[1] - A.velocity[1];
    // calculate relative velocity along normal vector using dot product n . v
    float velocity_along_normal = (relative_velocity[0]*collision_normal[0]) + (relative_velocity[1]*collision_normal[1]);
    // if collision update was already applied velocity is >= 0.0
    if (velocity_along_normal >= 0.0f) return;

    // calculate impulse and update velocities due to collision
    float impulse = -(1.0 + coef_restitution)* velocity_along_normal / (1.0/A.mass + 1.0/B.mass);

    for(size_t i = 0; i < A.position.size(); ++i){
        A.velocity[i] = A.velocity[i] - (impulse/A.mass)*collision_normal[i];
        B.velocity[i] = B.velocity[i] + (impulse/B.mass)*collision_normal[i];
    }

    // shift both particles along collision normal to prevent overlapping particles
    float overlap = (A.particle_radius + B.particle_radius) - normal_length;

    if (overlap > 0.0f){
        A.position[0] -= collision_normal[0] * 0.5f*overlap;
        A.position[1] -= collision_normal[1] * 0.5f*overlap;
        B.position[0] += collision_normal[0] * 0.5f*overlap;
        B.position[1] += collision_normal[1] * 0.5f*overlap;
    }


}

