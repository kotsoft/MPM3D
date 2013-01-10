#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    simulator = Simulator(125, 40, 20);
    simulator.AddParticle(5.5, 5.5, 5.5);
    
    glEnable(GL_DEPTH_TEST);
}

//--------------------------------------------------------------
void testApp::update(){
    simulator.Update();
}

//--------------------------------------------------------------
void testApp::draw(){
    ofBackground(0);
    ofScale(8, 8);
    
    glBegin(GL_LINES);
    for (int i = 0; i < simulator.particles.size(); i++) {
        Particle& p = simulator.particles[i];
        float v = fminf(1, p.u.norm()*1.5);
        glColor3f(v,.6+v*.4,1);
        float *x = p.x.data();
        float *x2 = Vector4f(p.x-p.stress.col(2)).data();
        glVertex3f(x[0], x[1], x[2]/20);
        glVertex3f(x2[0], x2[1], x2[2]/20);
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