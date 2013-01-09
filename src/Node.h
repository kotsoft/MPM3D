//
//  Node.h
//  MPM3D
//
//  Created by Ruby on 1/5/13.
//
//

#ifndef MPM3D_Node_h
#define MPM3D_Node_h

using namespace Eigen;

struct Node {
    Vector4f u, a, gx;
    float m;
    Node() {
        u.setZero();
        a.setZero();
        gx.setZero();
        m = 0;
    }
};

#endif
