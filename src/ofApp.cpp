#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup() {
    // listen for EInitialized notification. this indicates that
    // the arduino is ready to receive commands and it is safe to
    // call setupArduino()
    ofAddListener(ard.EInitialized, this, &ofApp::setupArduino);
    bSetupArduino	= false;
    
    //Color Contour Finder ==-=-=-=-==(){}[]=-=--=-==
    contourFinder.setMinAreaRadius(10);
    contourFinder.setMaxAreaRadius(350);
    
    //KINECT Setup
    //see how many devices we have.
    ofxKinectV2 tmp;
    vector <ofxKinectV2::KinectDeviceInfo> deviceList = tmp.getDeviceList();
    //allocate for this many devices
    kinects.resize(deviceList.size());
    texDepth.resize(kinects.size());
    texRGB.resize(kinects.size());
    
    //Note you don't have to use ofxKinectV2 as a shared pointer, but if you want to have it in a vector ( ie: for multuple ) it needs to be.
    for(int d = 0; d < kinects.size(); d++){
        kinects[d] = shared_ptr <ofxKinectV2> (new ofxKinectV2());
        kinects[d]->open(deviceList[d].serial);
    }
    
    
    
    //KALMAN-IA -------------------> <-------------------------------
    kalman.init(1/10000., 1/100.); // inverse of (smoothness, rapidness)
    
    line.setMode(OF_PRIMITIVE_LINE_STRIP);
    predicted.setMode(OF_PRIMITIVE_LINE_STRIP);
    estimated.setMode(OF_PRIMITIVE_LINE_STRIP);
    
    speed = 0.f;
    
    
    //ARDUINO =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    ard.connect("/dev/tty.usbmodem1411", 57600);
    
    
    
    gui.setup();
    gui.add(threshold.set("Threshold", 30, 0, 255));
}

void ofApp::update() {
    updateArduino();
    kinects[0]->update();
    
    if( kinects[0]->isFrameNew() ){
        texRGB[0].loadData( kinects[0]->getRgbPixels() );
        ofPixels kinectPix = kinects[0]->getRgbPixels();
        origOutput.setFromPixels(kinectPix);
        contourFinder.setTargetColor(targetColor,TRACK_COLOR_HSV);
        contourFinder.setThreshold(threshold);
        contourFinder.findContours(kinectPix);
        
        if (contourFinder.size()>0) {
            ofVec2f curPoint(contourFinder.getCentroid(0).x, contourFinder.getCentroid(0).y);
            line.addVertex(curPoint);
            
            kalman.update(curPoint); // feed measurement
            
            point = kalman.getPrediction(); // prediction before measurement
            int a = static_cast<int>(ofMap(point.y, 0.0, 1080.0, 0.0, 180.0));
            // rotate servo head to 180 degrees
            int ini = ofGetElapsedTimeMicros();
            //                cout << "ini:" << ofGetElapsedTimeMicros() << endl;
            ard.sendServo(9, a, false);
            
            if(ofGetElapsedTimeMicros() - ini > 100) {
                cout << "duration:" << (ofGetElapsedTimeMicros() - ini) << endl;
                
            }
            ard.sendDigital(18, ARD_HIGH);  // pin 20 if using StandardFirmata from Arduino 0022 or older
            
            predicted.addVertex(point);
            estimated.addVertex(kalman.getEstimation()); // corrected estimation after measurement
            ofVec3f vel = kalman.getVelocity();
            speed = vel.length();
            
            velVector.addVertex(vel.x, vel.y);
            int alpha = ofMap(speed, 0, 20, 50, 255, true);
            line.addColor(ofColor(255, 255, 255, alpha));
            predicted.addColor(ofColor(255, 0, 0, alpha));
            estimated.addColor(ofColor(0, 255, 255, alpha));
        }
    }
}

void ofApp::draw() {
    drawOutputContours();
    gui.draw();
}

void ofApp::mousePressed(int x, int y, int button) {
    targetColor = kinects[0]->getRgbPixels().getColor(x, y);
}


void ofApp::drawOutputContours() {
    origOutput.draw(0,0, 1920, 1080);
    ofSetLineWidth(2);
    
    ofPushStyle();
    ofSetColor(ofColor::red, 128);
    ofFill();
    estimated.draw();
    if(estimated.getNumVertices()> 5){
        estimated.removeVertex(0);
    }
    ofPopStyle();
    
    
    ofPushStyle();
    ofSetColor(255, 0, 0);
    ofFill();
    contourFinder.draw();
    ofPopStyle();
}



//ARDUINO |||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~|||
//        |||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~|||
//        |||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~|||
//        |||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~|||
//        |||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~|||
//        |||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~|||
//        |||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~|||
//        |||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||/|||~~*~~~*~~|||\||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~|||
//        |||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~|||
//        |||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~|||
//        |||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~|||
//        |||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~|||
//        |||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~|||
//        |||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~||||||~~*~~~*~~|||
void ofApp::setupArduino(const int & version) {
    
    // remove listener because we don't need it anymore
    ofRemoveListener(ard.EInitialized, this, &ofApp::setupArduino);
    
    // it is now safe to send commands to the Arduino
    bSetupArduino = true;
    
    // print firmware name and version to the console
    ofLogNotice() << ard.getFirmwareName();
    ofLogNotice() << "firmata v" << ard.getMajorFirmwareVersion() << "." << ard.getMinorFirmwareVersion();
    
    // Note: pins A0 - A5 can be used as digital input and output.
    // Refer to them as pins 14 - 19 if using StandardFirmata from Arduino 1.0.
    // If using Arduino 0022 or older, then use 16 - 21.
    // Firmata pin numbering changed in version 2.3 (which is included in Arduino 1.0)
    
    // set pins D2 and A5 to digital input
    ard.sendDigitalPinMode(2, ARD_INPUT);
    ard.sendDigitalPinMode(19, ARD_INPUT);  // pin 21 if using StandardFirmata from Arduino 0022 or older
    
    // set pin A0 to analog input
    ard.sendAnalogPinReporting(0, ARD_ANALOG);
    
    // set pin D13 as digital output
    ard.sendDigitalPinMode(13, ARD_OUTPUT);
    // set pin A4 as digital output
    ard.sendDigitalPinMode(18, ARD_OUTPUT);  // pin 20 if using StandardFirmata from Arduino 0022 or older
    
    // set pin D11 as PWM (analog output)
    ard.sendDigitalPinMode(11, ARD_PWM);
    
    // attach a servo to pin D9
    // servo motors can only be attached to pin D3, D5, D6, D9, D10, or D11
    ard.sendServoAttach(9);
    
    // Listen for changes on the digital and analog pins
    ofAddListener(ard.EDigitalPinChanged, this, &ofApp::digitalPinChanged);
    ofAddListener(ard.EAnalogPinChanged, this, &ofApp::analogPinChanged);
}

// ------------------------------------------------------------------------------------------------------------
void ofApp::updateArduino(){
    
    // update the arduino, get any data or messages.
    // the call to ard.update() is required
    ard.update();
    
    // do not send anything until the arduino has been set up
    if (bSetupArduino) {
        // fade the led connected to pin D11
        ard.sendPwm(11, (int)(128 + 128 * sin(ofGetElapsedTimef())));   // pwm...
    }
    
}

// digital pin event handler, called whenever a digital pin value has changed
// note: if an analog pin has been set as a digital pin, it will be handled
// by the digitalPinChanged function rather than the analogPinChanged function.

//--------------------------------------------------------------
void ofApp::digitalPinChanged(const int & pinNum) {
    // do something with the digital input. here we're simply going to print the pin number and
    // value to the screen each time it changes
    buttonState = "digital pin: " + ofToString(pinNum) + " = " + ofToString(ard.getDigital(pinNum));
}

// analog pin event handler, called whenever an analog pin value has changed

//--------------------------------------------------------------
void ofApp::analogPinChanged(const int & pinNum) {
    // do something with the analog input. here we're simply going to print the pin number and
    // value to the screen each time it changes
    potValue = "analog pin: " + ofToString(pinNum) + " = " + ofToString(ard.getAnalog(pinNum));
}
