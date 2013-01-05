//
//  Particle.h
//  MPM3D
//
//  Created by Ruby on 1/5/13.
//
//

#ifndef MPM3D_Particle_h
#define MPM3D_Particle_h

using namespace Eigen;

struct Particle {
    Vector4f x;
    Vector4f u;
    Vector4f phi[8];
    int c;
    Particle() {
        x << 0,0,0,0;
        u << 0,0,0,0;
    }
    Particle(float x0, float y0, float z0) {
        x << x0,y0,z0,0;
        u << .1,0,0,0;
    }
};

#endif
