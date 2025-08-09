#include "particle_simulator/particle.hpp"

Particle::Particle() {
    position = {0.0f, 0.0f};
    velocity = {0.0f, 0.0f};
    acceleration = {0.0f, 0.0f};
    particle_radius = 60.0f;
}

Particle::Particle(const std::vector<float> initial_position, const std::vector<float> initial_velocity, float particle_r) {
    position = initial_position;
    velocity = initial_velocity;
    acceleration = {0.0f, 0.0f};
    particle_radius = particle_r;
}


void Particle::update(const float dt){

    this->acceleration[1] = this->mass*this->gravity;
    std::vector<float> new_velocity(2);
    std::vector<float> new_position(2);

    for (int i = 0; i < 2; i++) {
        new_velocity[i] = this->velocity[i] + this->acceleration[i]*dt;
        new_position[i] = this->position[i] + new_velocity[i]*dt;
    }

    this->position = new_position;
    this->velocity = new_velocity;
    

}


void Particle::check_boundary(const float particle_radius, const float width, const float height, const float coef_restitution){
    if((this->position[0] + particle_radius) > width){
        this->position[0] = width - particle_radius;
        this->velocity[0] = -(this->velocity[0]*coef_restitution);
    }

    if((this->position[0] - particle_radius) < 0.0f){
        this->position[0] = particle_radius;
        this->velocity[0] = -(this->velocity[0]*coef_restitution);
    }

    if((this->position[1] + particle_radius) > height){
        this->position[1] = height - particle_radius;
        this->velocity[1] = -(this->velocity[1]*coef_restitution);
    }

    if((this->position[1] - particle_radius) < 0.0f){
        this->position[1] = particle_radius;
        this->velocity[1] = -(this->velocity[1]*coef_restitution);
    }
}

