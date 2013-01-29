#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    simulator = new Simulator(150, 100, 60);
    simulator->AddParticle(5.5, 5.5, 5.5);
    
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
}

//--------------------------------------------------------------
void testApp::update(){
}

//--------------------------------------------------------------
void testApp::draw(){
    simulator->startThread();
    /*
    ofFile obj;
    if (obj.open(ofToDataPath("pointcloud"+ ofToString(ofGetFrameNum())+".obj"), ofFile::WriteOnly)) {
        for (int i = 0; i < simulator->particles.size(); i++) {
            Particle& p = simulator->particles[i];
            float *x = p.x.data();
            obj << "v " << x[0] << " " << x[1] << " " << x[2] << endl;
        }
    }
    obj.close();
    */
    ofClear(0);
    ofScale(8, 8);
    glBegin(GL_POINTS);
    for (int i = 0; i < simulator->particles.size(); i++) {
        Particle& p = simulator->particles[i];
        float v = fminf(1, p.u.norm()*1.5);
        float *x = p.x.data();
        //float *n = p.normal.data();
        
        //glColor3f(v*x[2]/20,(.6+v*.4)*x[2]/20,1*x[2]/20);
        float brightness = 1-(x[2]/160);
        glColor3f(v*brightness,(.6+v*.4)*brightness,1*brightness);
        glVertex3f(x[0], x[1], 1-x[2]/80);
        
        
    }
    glEnd();
    ofScale(1.0/8, 1.0/8);
    
    ofColor(255);
    
    frameRates[ofGetFrameNum()/100] = ofGetFrameRate();
    for (int i = 0; i < ofGetFrameNum()/100; i++) {
        ofDrawBitmapString(ofToString(frameRates[i]), 10+i*50, 10);
    }
    simulator->waitForThread();
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){

}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}