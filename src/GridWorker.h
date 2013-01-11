//
//  GridWorker.h
//  MPM3D
//
//  Created by Ruby on 1/11/13.
//
//

#ifndef MPM3D_GridWorker_h
#define MPM3D_GridWorker_h

class GridWorker : public Poco::Runnable {
public:
    GridWorker(vector<Node*> &active) : active(active) {}
    
    vector<Node*> &active;
    int start, end, action;
    virtual void run() {
        switch (action) {
            case 0:
            {
                for (int i = start; i < end; i++) {
                    Node &n = *active[i];
                    n.m = 0;
                    n.a.setZero();
                    n.gx.setZero();
                }
            }
                break;
                
            case 1:
            {
                for (int i = start; i < end; i++) {
                    
                }
            }
                break;
                
            default:
                break;
        }
    }
};

#endif
