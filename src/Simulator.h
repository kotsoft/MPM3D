//
//  Simulator.h
//  MPM3D
//
//  Created by Ruby on 1/5/13.
//
//

#ifndef MPM3D_Simulator_h
#define MPM3D_Simulator_h

#include <Eigen>
#include <vector>

#include "Particle.h"
#include "Node.h"

using namespace Eigen;
using namespace std;

class Simulator {
    int gSizeX, gSizeY, gSizeZ, gSizeY_2, gSizeZ_2, gSize;
    Vector4i cmul;
public:
    vector<Particle> particles;
    Node *grid;
    
    Simulator() {}
    
    Simulator(int gridSizeX, int gridSizeY, int gridSizeZ) {
        gSizeX = gridSizeX;
        gSizeY = gridSizeY;
        gSizeZ = gridSizeZ;
        gSizeY_2 = gSizeY*gSizeZ-2*gSizeZ;
        gSizeZ_2 = gSizeZ-2;
        gSize = gSizeX*gSizeY*gSizeZ;
        cmul << gSizeY*gSizeZ, gSizeZ, 1, 0;
        grid = (Node*)malloc(gSize*sizeof(Node));
        for (int i = 0; i < gSize; i++) {
            grid[i] = Node();
        }
    }
    
    void AddParticle(float x, float y, float z) {
        for (int i = 0; i < 125; i++) {
            for (int j = 0; j < 40; j++) {
                for (int k = 0; k < 20; k++) {
                    particles.push_back(Particle(x+i*.5, y+j*.5, z+k*.5));
                }
            }
        }
        
    }
    
    void Update() {
        for (int i = 0; i < gSize; i++) {
            Node &n = grid[i];
            if (n.m > 0) {
                n.m = 0;
                n.u.setZero();
                n.a.setZero();
            }
        }
        
        Vector4f u[2], v[2], w[2], splat;
        for (int i = 0; i < particles.size(); i++) {
            Particle &p = particles[i];
            
            Vector4i cx = p.x.cast<int>();
            
            // calculate 1d index of cell
            p.c = cx.dot(cmul);
            
            Vector4f dx = p.x-cx.cast<float>();
            float *dxData = dx.data();
            
            Vector4f one = Vector4f::Ones();
            u[1] = splat = Vector4f::Constant(dxData[0]);
            u[0] = one-splat;
            v[1] = splat = Vector4f::Constant(dxData[1]);
            v[0] = one-splat;
            w[1] = splat = Vector4f::Constant(dxData[2]);
            w[0] = one-splat;
            
            u[0][0] = 1;
            u[1][0] = -1;
            v[0][1] = 1;
            v[1][1] = -1;
            w[0][2] = 1;
            w[1][2] = -1;
            
            Vector4f *phiPtr = &p.phi[0];
            Node *nodePtr = &grid[p.c];
            for (int x = 0; x < 2; x++, nodePtr += gSizeY_2) {
                for (int y = 0; y < 2; y++, nodePtr += gSizeZ_2) {
                    for (int z = 0; z < 2; z++, phiPtr++, nodePtr++) {
                        Vector4f &phi = *phiPtr = u[x].cwiseProduct(v[y].cwiseProduct(w[z]));
                        Node &n = *nodePtr;
                        
                        float ws = phi.w();
                        Vector4f w = Vector4f::Constant(ws);
                        n.m += ws;
                        n.u += w.cwiseProduct(p.u);
                        n.a += phi*.01f;
                    }
                }
            }
        }
        
        for (int i = 0; i < gSize; i++) {
            Node &n = grid[i];
            if (n.m > 0) {
                n.mdiv = Vector4f::Constant(1/n.m);
                n.mdiv[3] = 0;
                n.u = n.u.cwiseProduct(n.mdiv);
                n.a = n.a.cwiseProduct(n.mdiv);
                n.u += n.a;
            }
        }
        
        for (int i = 0; i < particles.size(); i++) {
            Particle &p = particles[i];
            
            Vector4f gu = Vector4f::Zero();
            
            Vector4f *phiPtr = &p.phi[0];
            Node *nodePtr = &grid[p.c];
            for (int x = 0; x < 2; x++, nodePtr += gSizeY_2) {
                for (int y = 0; y < 2; y++, nodePtr += gSizeZ_2) {
                    for (int z = 0; z < 2; z++, phiPtr++, nodePtr++) {
                        Vector4f &phi = *phiPtr;
                        Node &n = *nodePtr;
                        
                        Vector4f w = Vector4f::Constant(phi.w());
                        gu += w.cwiseProduct(n.u);
                    }
                }
            }
            
            p.x += gu;
        }
    }
};

#endif
