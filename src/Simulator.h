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
#include <Poco/ThreadPool.h>

#include "Particle.h"
#include "Node.h"

#define NUM_THREADS 4

using namespace Poco;

class ClearGridCells : public Runnable {
    Node *grid;
    int start, end;
public:
    ClearGridCells(Node *grid, int start, int end) : grid(grid), start(start), end(end) {}
    virtual void run() {
        for (int i = start; i < end; i++) {
            Node &n = grid[i];
            if (n.m > 0) {
                n.m = 0;
                n.a.setZero();
                n.gx.setZero();
            }
        }
    }
};

using namespace Eigen;
using namespace std;

class Simulator : public ofThread {
    int gSizeX, gSizeY, gSizeZ, gSizeY_2, gSizeZ_2, gSize;
    Vector4i cmul;
    Vector4f dxSub[8];
    Vector4f lowBound;
    Vector4f highBound;
    Vector4f lowBoundS;
    Vector4f highBoundS;
    Vector4f gravity;
    
    ClearGridCells *clearGridCells[NUM_THREADS];
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
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
                for (int z = 0; z < 2; z++) {
                    dxSub[x*4+y*2+z] = Vector4f(x, y, z, -1);
                }
            }
        }
        
        lowBound = Vector4f(1.0f, 1.0f, 1.0f, 0.0f);
        highBound = Vector4f(gSizeX-2, gSizeY-2, gSizeZ-2, 0);
        lowBoundS = Vector4f(3.0f, 3.0f, 3.0f, 0.0f);
        highBoundS = Vector4f(gSizeX-4, gSizeY-4, gSizeZ-4, 0);
        
        gravity = Vector4f(0, .01f, 0, 0);
        
        int gSize_8 = gSize/NUM_THREADS;
        for (int i = 0; i < NUM_THREADS; i++) {
            int start = i*gSize_8;
            int end = i < (NUM_THREADS-1) ? (i+1)*gSize_8 : gSize;
            clearGridCells[i] = new ClearGridCells(grid, start, end);
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
    
    void threadedFunction() {
        Update();
    }
    void Update() {
        /*
        for (int i = 0; i < gSize; i++) {
            Node &n = grid[i];
            if (n.m > 0) {
                n.m = 0;
                n.a.setZero();
                n.gx.setZero();
            }
        }*/
        
        for (int i = 0; i < NUM_THREADS; i++) {
            ThreadPool::defaultPool().start(*clearGridCells[i]);
        }
        ThreadPool::defaultPool().joinAll();
        
        Vector4f u[2], v[2], w[2], splat;
        for (int i = 0; i < particles.size(); i++) {
            Particle &p = particles[i];
            
            Vector4i cx = p.x.cast<int>();
            
            // calculate 1d index of cell
            p.c = cx.dot(cmul);
            
            p.dx = p.x-cx.cast<float>();
            float *dxData = p.dx.data();
            
            u[1] = splat = Vector4f::Constant(dxData[0]);
            u[0] = Vector4f::Ones()-splat;
            v[1] = splat = Vector4f::Constant(dxData[1]);
            v[0] = Vector4f::Ones()-splat;
            w[1] = splat = Vector4f::Constant(dxData[2]);
            w[0] = Vector4f::Ones()-splat;
            
            u[0][0] = 1;
            u[1][0] = -1;
            v[0][1] = 1;
            v[1][1] = -1;
            w[0][2] = 1;
            w[1][2] = -1;
            
            Vector4f *phiPtr = &p.phi[0];
            float *wPtr = &p.w[0];
            Node *nodePtr = &grid[p.c];
            for (int x = 0; x < 2; x++, nodePtr += gSizeY_2) {
                for (int y = 0; y < 2; y++, nodePtr += gSizeZ_2) {
                    for (int z = 0; z < 2; z++, phiPtr++, wPtr++, nodePtr++) {
                        Vector4f &phi = *phiPtr = u[x].cwiseProduct(v[y].cwiseProduct(w[z]));
                        Node &n = *nodePtr;
                        
                        *wPtr = phi.w();
                        n.gx += phi;
                        phi.w() = 0;
                    }
                }
            }
        }
        
        for (int i = 0; i < particles.size(); i++) {
            Particle &p = particles[i];
            
            float density = 0;
            
            Vector4f *phiPtr = &p.phi[0];
            float *wPtr = &p.w[0];
            Node *nodePtr = &grid[p.c];
            Vector4f *dxPtr = &dxSub[0];
            for (int x = 0; x < 2; x++, nodePtr += gSizeY_2) {
                for (int y = 0; y < 2; y++, nodePtr += gSizeZ_2) {
                    for (int z = 0; z < 2; z++, phiPtr++, wPtr++, nodePtr++, dxPtr++) {
                        Vector4f &phi = *phiPtr;
                        Node &n = *nodePtr;
                        
                        density += *wPtr * (n.gx.dot(p.dx - *dxPtr));
                    }
                }
            }
            
            float pressure = .25*(density-2);
            
            if (pressure > .5) {
                pressure = .5;
            }
            
            Vector4f wallforce = Vector4f::Zero();
            
            auto comparisonl = (p.x.array() < lowBoundS.array());
            if (comparisonl.any()) {
                Vector4f compMul = comparisonl.cast<float>();
                wallforce += compMul.cwiseProduct(lowBoundS-p.x);
            }
            auto comparisonh = (p.x.array() > highBoundS.array());
            if (comparisonh.any()) {
                Vector4f compMul = comparisonh.cast<float>();
                wallforce += compMul.cwiseProduct(highBoundS-p.x);
            }
            
            phiPtr = &p.phi[0];
            wPtr = &p.w[0];
            nodePtr = &grid[p.c];
            
            Matrix4f elasticForce = p.strain*0+p.stress*.1;
            pressure -= elasticForce.trace()/3-.5*p.stress.trace()/3;
            
            for (int x = 0; x < 2; x++, nodePtr += gSizeY_2) {
                for (int y = 0; y < 2; y++, nodePtr += gSizeZ_2) {
                    for (int z = 0; z < 2; z++, phiPtr++, wPtr++, nodePtr++) {
                        Vector4f &phi = *phiPtr;
                        Node &n = *nodePtr;
                        
                        n.a -= phi*pressure - *wPtr*wallforce + elasticForce*phi;
                    }
                }
            }
        }
        
        for (int i = 0; i < gSize; i++) {
            Node &n = grid[i];
            n.m = n.gx.w();
            if (n.m > 0) {
                n.a = n.a/n.m + gravity;
                n.u.setZero();
            }
        }
        
        for (int i = 0; i < particles.size(); i++) {
            Particle &p = particles[i];
            
            Vector4f ga = Vector4f::Zero();
            
            float *wPtr = &p.w[0];
            Node *nodePtr = &grid[p.c];
            for (int x = 0; x < 2; x++, nodePtr += gSizeY_2) {
                for (int y = 0; y < 2; y++, nodePtr += gSizeZ_2) {
                    for (int z = 0; z < 2; z++, wPtr++, nodePtr++) {
                        Node &n = *nodePtr;
                        
                        ga += *wPtr * n.a;
                    }
                }
            }

            p.u += ga;
            
            wPtr = &p.w[0];
            nodePtr = &grid[p.c];
            for (int x = 0; x < 2; x++, nodePtr += gSizeY_2) {
                for (int y = 0; y < 2; y++, nodePtr += gSizeZ_2) {
                    for (int z = 0; z < 2; z++, wPtr++, nodePtr++) {
                        Node &n = *nodePtr;
                        
                        n.u += *wPtr * p.u;
                    }
                }
            }
        }
        
        for (int i = 0; i < gSize; i++) {
            Node &n = grid[i];
            if (n.m > 0) {
                n.u /= n.m;
            }
        }
        
        for (int i = 0; i < particles.size(); i++) {
            Particle &p = particles[i];
            
            Vector4f gu = Vector4f::Zero();
            Matrix4f L = Matrix4f::Zero();
            
            Vector4f *phiPtr = &p.phi[0];
            float *wPtr = &p.w[0];
            Node *nodePtr = &grid[p.c];
            for (int x = 0; x < 2; x++, nodePtr += gSizeY_2) {
                for (int y = 0; y < 2; y++, nodePtr += gSizeZ_2) {
                    for (int z = 0; z < 2; z++, phiPtr++, wPtr++, nodePtr++) {
                        Vector4f &phi = *phiPtr;
                        Node &n = *nodePtr;
                        
                        gu += *wPtr * n.u;
                        
                        L += phi*n.u.transpose();
                    }
                }
            }
            
            
            
            Matrix4f LT = L.transpose();
            
            p.strain += p.stress = (L+L.transpose())*.5f;
            float norm = p.strain.squaredNorm();
            if (norm > 1) {
                norm = sqrtf(norm);
                p.strain -= .5*(norm-1)/norm*p.strain;
            }
            
            p.x += gu;
            p.u += .1*(gu-p.u);
            
            auto comparisonl = (p.x.array() < lowBound.array());
            if (comparisonl.any()) {
                Vector4f compMul = comparisonl.cast<float>();
                p.x += compMul.cwiseProduct(lowBound-p.x+.001f*Vector4f::Random());
                p.u -= compMul.cwiseProduct(p.u);
            }
            auto comparisonh = (p.x.array() > highBound.array());
            if (comparisonh.any()) {
                Vector4f compMul = comparisonh.cast<float>();
                p.x += compMul.cwiseProduct(highBound-p.x-.001f*Vector4f::Random());
                p.u -= compMul.cwiseProduct(p.u);
            }
        }
    }
};

#endif
