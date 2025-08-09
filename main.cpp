#include <SFML/Graphics.hpp>
#include <particle_simulator/particle.hpp>
#include "src/collision.cpp"
#include <iostream>
#include <vector>
#include <memory>
#include <random>


sf::Color getRandomColor() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, 255); 
    
    return sf::Color(dis(gen), dis(gen), dis(gen)); 
}

int main()
{   
    const int substeps = 8;
    const float dt = 0.1f; //sim time step
    const float subdt = dt / 4;
    const float width = 640.0f; //smfl window width
    const float height = 480.0f; //smfl window height
    float particle_radius = 10.0f; //particle radius in pixels
    const float coef_restitution = 0.9; //coefficient of restituion for collisions
    const int   n_particles = 100; // number of particles

    sf::RenderWindow window(sf::VideoMode(width,height),"Particle Simulation");
    window.setFramerateLimit(60);
    
    std::vector<std::unique_ptr<Particle>> particles;
    std::vector<std::unique_ptr<sf::CircleShape>> shapes;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> init_pos_x(0.0f, 640.0f);  
    std::uniform_real_distribution<float> init_pos_y(0.0f, 480.0f); 
    std::uniform_real_distribution<float> init_vel(-50.0f, 50.0f);

    for (int i = 0; i < n_particles; ++i) {
        std::vector<float> initial_position = {init_pos_x(gen), init_pos_y(gen)};
        std::vector<float> initial_velocity = {init_vel(gen), init_vel(gen)};
        particles.push_back(std::make_unique<Particle>(initial_position, initial_velocity, particle_radius));

        auto shape = std::make_unique<sf::CircleShape>(particle_radius);
        shape->setOrigin(particle_radius, particle_radius);
        shape->setFillColor(getRandomColor());
        shapes.push_back(std::move(shape));

    }
    
    // MAIN LOOP
    while (window.isOpen()) 
    {
        sf::Event event;
        while (
            window.pollEvent(event))
            if (event.type ==
            sf::Event::Closed)
                window.close();


        for (int step = 0; step < substeps; ++step){

            for (auto& p : particles){
                p->update(subdt);
            }
            for (size_t i = 0; i < particles.size(); ++i){
                Particle& p = *particles[i];

                for (size_t j = i+1; j < particles.size(); ++j){
                    Particle& p_collision = *particles[j];

                    if (check_collision(p , p_collision)){
                        apply_collision(p,p_collision, coef_restitution);
                    }

                }

                p.check_boundary(particle_radius, width, height,coef_restitution);

            }
        }

        window.clear();
        for (size_t i = 0; i < particles.size(); ++i){
            Particle& p = *particles[i];
            sf::Vector2f pos = {p.position[0], p.position[1]};
            sf::CircleShape& shape = *shapes[i];
            shape.setPosition(pos);
            window.draw(shape); 
            // p.update(dt);

            // for (size_t j = i+1; j < particles.size(); ++j){
            //     Particle& p_collision = *particles[j];

            //     if (check_collision(p , p_collision)){
            //         apply_collision(p,p_collision, coef_restitution);
            //     }


            // }
            // p.check_boundary(particle_radius, width, height,coef_restitution);
        }
        window.display();
    }
    return 0;
}