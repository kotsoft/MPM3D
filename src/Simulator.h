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
#include "Region.h"
#include "GridWorker.h"
#include <Poco/ThreadPool.h>

using namespace Eigen;
using namespace std;
using namespace Poco;

class Simulator : public ofThread {
    int gSizeX, gSizeY, gSizeZ, gSizeY_2, gSizeZ_2, gSize;
    Vector4i cmul;
    Vector4f dxSub[8];
    Vector4f lowBound;
    Vector4f highBound;
    Vector4f lowBoundS;
    Vector4f highBoundS;
    Vector4f gravity;
    
    ThreadPool *threadpool;
public:
    vector<Particle> particles;
    static const int nRegions = 16;
    Region *regions[nRegions];
    
    static const int nGWorkers = 8;
    GridWorker *gridWorkers[nGWorkers];
    
    //Region region;
    Node *grid;
    vector<Node*> active;
    
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
        
        
        lowBound = Vector4f(1.0f, 1.0f, 1.0f, 0.0f);
        highBound = Vector4f(gSizeX-2, gSizeY-2, gSizeZ-2, 0);
        lowBoundS = Vector4f(3.0f, 3.0f, 3.0f, 0.0f);
        highBoundS = Vector4f(gSizeX-4, gSizeY-4, gSizeZ-4, 0);
        
        gravity = Vector4f(0, .01f, 0, 0);
        
        for (int i = 0; i < nRegions; i++) {
            regions[i] = new Region(grid, lowBound, highBound, lowBoundS, highBoundS, gSizeY_2, gSizeZ_2, cmul);
        }
        
        for (int i = 0; i < nGWorkers; i++) {
            gridWorkers[i] = new GridWorker(active);
        }
        
        threadpool = new ThreadPool();
    }
    
    void AddParticle(float x, float y, float z) {
        for (int i = 0; i < 20; i++) {
            for (int j = 0; j < 40; j++) {
                for (int k = 0; k < 125; k++) {
                    particles.push_back(Particle(x+i*.5, y+j*.5, z+k*.5));
                }
            }
        }
    }
    
    void threadedFunction() {
        Update();
    }
    
    void Update() {
        for (int i = 0; i < nRegions; i++) {
            regions[i]->particles.clear();
        }
        for (int i = 0; i < particles.size(); i++) {
            Particle &p = particles[i];
            int region = p.x[0]-3;
            if (region < 0) {
                region = 0;
            } else if (region > nRegions-1) {
                region = nRegions-1;
            }
            regions[region]->particles.push_back(&p);
        }
        
        
        for (int i = 0; i < active.size(); i++) {
            Node &n = *active[i];
            n.m = 0;
            n.a.setZero();
            n.gx.setZero();
        }
        /*
        int block = active.size()/nGWorkers;
        for (int i = 0; i < nGWorkers; i++) {
            gridWorkers[i]->action = 0;
            gridWorkers[i]->active = active;
            gridWorkers[i]->start = i*block;
            gridWorkers[i]->end = i == nGWorkers-1 ? active.size() : (i+1)*block;
            threadpool->start(*gridWorkers[i]);
        }
        threadpool->joinAll();
        */
        active.clear();
        
        // Add particle mass to grid
        for (int i = 0; i < nRegions/2; i++) {
            regions[i*2]->currentFunction = 0;
            threadpool->start(*regions[i*2]);
        }
        threadpool->joinAll();
        
        for (int i = 0; i < nRegions/2; i++) {
            regions[i*2+1]->currentFunction = 0;
            threadpool->start(*regions[i*2+1]);
        }
        threadpool->joinAll();
        
        // Add forces to grid
        for (int i = 0; i < nRegions/2; i++) {
            regions[i*2]->currentFunction = 1;
            threadpool->start(*regions[i*2]);
        }
        threadpool->joinAll();
        for (int i = 0; i < nRegions/2; i++) {
            regions[i*2+1]->currentFunction = 1;
            threadpool->start(*regions[i*2+1]);
        }
        threadpool->joinAll();
        
        for (int i = 0; i < gSize; i++) {
            Node &n = grid[i];
            if (n.m > 0) {
                n.a = n.a/n.m + gravity;
                n.u.setZero();
                
                active.push_back(&n);
            }
        }
        
        // Particle velocities
        for (int i = 0; i < nRegions/2; i++) {
            regions[i*2]->currentFunction = 2;
            threadpool->start(*regions[i*2]);
        }
        threadpool->joinAll();
        for (int i = 0; i < nRegions/2; i++) {
            regions[i*2+1]->currentFunction = 2;
            threadpool->start(*regions[i*2+1]);
        }
        threadpool->joinAll();
        
        for (int i = 0; i < active.size(); i++) {
            Node &n = *active[i];
            if (n.m > 0) {
                n.u /= n.m;
            }
        }
        
        // Advance Particles
        for (int i = 0; i < nRegions; i++) {
            regions[i]->currentFunction = 3;
            threadpool->start(*regions[i]);
        }
        threadpool->joinAll();
    }
};

#endif
