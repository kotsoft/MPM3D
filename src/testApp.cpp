#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    simulator = new Simulator(23, 80, 250);
    simulator->AddParticle(5.5, 5.5, 5.5);
    
    pointsFBO.allocate(1024, 768);
    
    glEnable(GL_DEPTH_TEST);
}

//--------------------------------------------------------------
void testApp::update(){
}

//--------------------------------------------------------------
void testApp::draw(){
    simulator->startThread();
    ofBackground(0);
    
    pointsFBO.begin();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    ofScale(5, 5);
    glBegin(GL_POINTS);
    for (int i = 0; i < simulator->particles.size(); i++) {
        Particle& p = simulator->particles[i];
        float v = fminf(1, p.u.norm()*1.5);
        glColor3f(v, .6+.4*v, 1);
        float *x = p.x.data();
        glVertex3f(x[2], x[1], x[0]/30);
        //glVertex3f(x2[2], x2[1], x2[0]/20);
    }
    glEnd();
    ofScale(.2, .2);
    pointsFBO.end();
    
    pointsFBO.draw(0, 0);
    
    ofColor(255);
    ofDrawBitmapString(ofToString(ofGetFrameRate()), 10, 10);
    simulator->waitForThread(false);
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