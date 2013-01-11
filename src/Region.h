//
//  Region.h
//  MPM3D
//
//  Created by Ruby on 1/10/13.
//
//

#ifndef MPM3D_Region_h
#define MPM3D_Region_h

struct Region : public ofThread {
    vector<Particle*> particles;
    Node *grid;
    Vector4f lowBound;
    Vector4f highBound;
    int gSizeY_2;
    int gSizeZ_2;
    Vector4i cmul;
    
    int currentFunction;
    
    Region(Node *grid, Vector4f lowBound, Vector4f highBound, int gSizeY_2, int gSizeZ_2, Vector4i cmul) : grid(grid), lowBound(lowBound), highBound(highBound), gSizeY_2(gSizeY_2), gSizeZ_2(gSizeZ_2), cmul(cmul) {}
    
    void threadedFunction() {
        switch (currentFunction) {
            case 0:
                ParticlesToGrid1();
                break;
            case 3:
                AdvanceParticles();
                break;
        }
        AdvanceParticles();
    }
    
    void ParticlesToGrid1() {
        Vector4f u[2], v[2], w[2], splat;
        for (int i = 0; i < particles.size(); i++) {
            Particle &p = *particles[i];
            
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
                        
                        float &ws = *wPtr = phi.w();
                        phi.w() = 0;
                        
                        n.m += ws;
                        n.gx += phi;
                    }
                }
            }
        }
    }
    
    void AdvanceParticles() {
        for (int i = 0; i < particles.size(); i++) {
            Particle &p = *particles[i];
            
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
            //Matrix4f W = (L-LT)*.5f;
            
            p.stress = (L+LT)*.5f;
            //p.strain += p.stress + p.strain*W - W*p.strain;
            p.strain += p.stress;
            float norm = p.strain.squaredNorm();
            if (norm > 9) {
                norm = sqrtf(norm);
                p.strain -= .5*(norm-3)/norm*p.strain;
            }
            
            p.x += gu;
            p.u += 1*(gu-p.u);
            
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
