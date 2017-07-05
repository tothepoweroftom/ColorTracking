#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"
#include "ofxGui.h"
#include "ofxKinectV2.h"
#include "ofEvents.h"



class ofApp : public ofBaseApp {
public:
    void setup();
    void update();
    void draw();
    void drawOutputContours();
    void mousePressed(int x, int y, int button);
    
    ofVideoGrabber cam;
    ofxCv::ContourFinder contourFinder;
    ofColor targetColor;
    ofxCvColorImage		origOutput;
    
    
    ofxPanel gui;
    ofParameter<float> threshold;
    ofParameter<bool> trackHs;
    
    
    //KINECT ----------|=====|-------------
    vector < shared_ptr<ofxKinectV2> > kinects;
    vector <ofTexture> texDepth;
    vector <ofTexture> texRGB;
    
    //Kalman Filter - To improve tracking of ball even when obscured.
    ofxCv::KalmanPosition kalman;
    ofMesh predicted, line, estimated;
    ofVec2f point;
    ofPolyline velVector;
    float speed;
    
    
    //Serial Comms
    ofArduino	ard;
    bool		bSetupArduino;			// flag variable for setting up arduino once
    
private:
    //Serial Comms
    void setupArduino(const int & version);
    void digitalPinChanged(const int & pinNum);
    void analogPinChanged(const int & pinNum);
    void updateArduino();
    
    string buttonState;
    string potValue;
    
    
};
