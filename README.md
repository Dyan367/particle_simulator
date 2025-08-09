# particle_simulator
Simple particle simulator with 2D impulse collision physics and a bounding volume hierarchy (BVH) tree for fast collision detection in C++. This project from a physics point of view is quite simple but was an attempt to develop more ability with C++.

# Dependencies
This repository uses SFML library to visualize the simulation. The library can be installed on linux using the following commands:
```bash
sudo apt update
sudo apt install libsfml-dev
```

# Examples
## Without BVH Tree acceleration (n_particles = 400, coef_restitution = 0.9)
![No BVH Demo](examples/no_bvh.gif)


# TO DO
- add BVH tree accelerator.
- load parameters from json file.
- change spawning of particles to something more interesting.