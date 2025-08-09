#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <vector>

class Particle {
public:
    std::vector<float> position;
    std::vector<float> velocity;
    float particle_radius;

    Particle();

    Particle(const std::vector<float> initial_position, const std::vector<float> initial_velocity, float particle_radius);

    void update(const float dt);
    void check_boundary(const float particle_radius, const float width, const float height,const float coef_restitution);

    std::vector<float> acceleration;
    float              mass = 1.0;
    float              gravity = 9.81;



};

#endif 
