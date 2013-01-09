#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    simulator = Simulator(250, 80, 40);
    simulator.AddParticle(5.5, 5.5, 5.5);
}

//--------------------------------------------------------------
void testApp::update(){
    simulator.Update();
}

//--------------------------------------------------------------
void testApp::draw(){
    ofScale(5, 5);
    
    glBegin(GL_POINTS);
    for (int i = 0; i < simulator.particles.size(); i++) {
        Particle& p = simulator.particles[i];
        //ofCircle(p.x[0], p.x[1], .5);
        float z = p.x[2]/20;
        glColor3f(z,z,z);
        glVertex2f(p.x[0], p.x[1]);
    }
    glEnd();
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