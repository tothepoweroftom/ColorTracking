#include "ofApp.h"




//--------------------------------------------------------------

//------ - - - - - - - - - -Simple Color Tracker - - - - - - - - - - - - - - - - -?
void ofApp::setup(){
    
    ofSetVerticalSync(true);
    
    ///GUI SETUP
    
    gui.setup();
    
    gui.add(lowHue.setup("lowHue", 32, 0, 360));
    gui.add(highHue.setup("highHue", 82, 0, 360));
    gui.add(lowSat.setup("lowSat", 100, 0, 360));
    gui.add(highSat.setup("highSat", 20, 0, 360));
    gui.add(rapidness.setup("Rapidness", 0.1, 0.000001, 1.0));
    gui.add(smoothness.setup("smoothness", 0.1, 0.000001, 1.0));


    kalman.init(1/10000., 1/100.); // inverse of (smoothness, rapidness)
    
    line.setMode(OF_PRIMITIVE_LINE_STRIP);
    predicted.setMode(OF_PRIMITIVE_LINE_STRIP);
    estimated.setMode(OF_PRIMITIVE_LINE_STRIP);
    
    speed = 0.f;
//    myPlayer.load("/Users/tom.power/Documents/of_v0.9.8_osx_release/addons/ofxCv/ColourTracking/sample.mov");
//    wCam.initGrabber(wWidth, wHeight);
    


    hue = 0;
    sat = 0;
    val = 0;
    
    drawEnabled = true;
    
    
    //Video Grabber from webcam
//    wCam.initGrabber(wWidth, wHeight);
    
    
    //KINECT INSTEAD ----
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

//    myPlayer.play();
    wWidth  = 1920;
    wHeight = 1080;
    


    //Allocate memory for outputs
    origOutput.allocate(wWidth, wHeight);
    origOutputHSV.allocate(wWidth, wHeight);
    
    hueOutput.allocate(wWidth, wHeight);
    satOutput.allocate(wWidth, wHeight);
    briOutput.allocate(wWidth, wHeight);
    
    
    //Tracked pixels
    lockedOnPixels = new unsigned char [wWidth * wHeight];
    lockedOnTexture.allocate(wWidth, wHeight, GL_LUMINANCE);
    lockedOutput.allocate(wWidth, wHeight);
    
    
    //SERIAL COMMS -----------------------==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-_=-+_+_+_+_+_==_++_
    
    // replace the string below with the serial port for your Arduino board
    // you can get this from the Arduino application or via command line
    // for OSX, in your terminal type "ls /dev/tty.*" to get a list of serial devices
    ard.connect("/dev/tty.usbmodem1411", 57600);
    
    // listen for EInitialized notification. this indicates that
    // the arduino is ready to receive commands and it is safe to
    // call setupArduino()
    ofAddListener(ard.EInitialized, this, &ofApp::setupArduino);
    bSetupArduino	= false;	// flag so we setup arduino when its ready, you don't need to touch this :)
    


}


//- --- -- --- --- - -  - -- -- -- -- -- -- - - ARDUINO ---------------------------------
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

//--------------------------------------------------------------
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



//--------------------------------------------------------------
void ofApp::update(){
    updateArduino();

        kinects[0]->update();
        if( kinects[0]->isFrameNew() ){
//            texDepth[d].loadData( kinects[d]->getDepthPixels() );
            texRGB[0].loadData( kinects[0]->getRgbPixels() );
            ofPixels kinectPix = kinects[0]->getRgbPixels();
            ofLog(OF_LOG_NOTICE, "Size" + ofToString(kinectPix.getWidth()));

            origOutput.setFromPixels(kinectPix);
            
            origOutputHSV = origOutput;
            origOutputHSV.convertRgbToHsv();
            
            origOutputHSV.convertToGrayscalePlanarImages(hueOutput, satOutput, briOutput);
            
            hueOutput.flagImageChanged();
            satOutput.flagImageChanged();
            briOutput.flagImageChanged();
            
            ofPixels huePixels = hueOutput.getPixels();
            ofPixels satPixels = satOutput.getPixels();
            ofPixels briPixels = briOutput.getPixels();
            
            //This is a flat array of the pixel values that we iterate through. Set to white the pixels that are there.
            for (int i = 0; i < (wWidth * wHeight); i++) {
                if ((huePixels[i] >= lowHue  && huePixels[i] <= highHue) &&
                    (satPixels[i] >= lowSat)) {
                    lockedOnPixels[i] = 255;
                } else {
                    lockedOnPixels[i] = 0;
                }
            }
            
            lockedOnTexture.loadData(lockedOnPixels, wWidth, wHeight, GL_LUMINANCE);
            lockedOutput.setFromPixels(lockedOnPixels, wWidth, wHeight);
            
            lockedContours.findContours(lockedOutput, 160, (wWidth * wHeight) / 3, 1, false, true);
            
            
            if (lockedContours.blobs.size() > 0) {
                
                ofVec2f curPoint(lockedContours.blobs.at(0).centroid.x, lockedContours.blobs.at(0).centroid.y);
                line.addVertex(curPoint);
                
                
                kalman.update(curPoint); // feed measurement
                
                point = kalman.getPrediction(); // prediction before measurement
                int a = static_cast<int>(ofMap(point.x, 0.0, 1920.0, 0.0, 180.0));
                // rotate servo head to 180 degrees
                ard.sendServo(9, a, false);
                ard.sendDigital(18, ARD_HIGH);  // pin 20 if using StandardFirmata from Arduino 0022 or older
                
                predicted.addVertex(point);
                estimated.addVertex(kalman.getEstimation()); // corrected estimation after measurement
                ofVec3f vel = kalman.getVelocity();
                speed = vel.length();
                int alpha = ofMap(speed, 0, 20, 50, 255, true);
                line.addColor(ofColor(255, 255, 255, alpha));
                predicted.addColor(ofColor(255, 0, 0, alpha));
                estimated.addColor(ofColor(0, 255, 255, alpha));
            }
            
            
            
            
        }
}

    
//    wCam.update(); // get all the new frames
//    if (wCam.isFrameNew()) {
//
//        
//    }


//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(100, 100, 100);
//    ofSetColor(0xffffff);
    
    texRGB[0].draw(0,0, wWidth, wHeight );
//    origOutputHSV.draw(360, 0, wWidth/4, wHeight/4);
    lockedContours.draw();
//
//    lockedOnTexture.draw(200, 380, wWidth/4, wHeight/4);
//    
    
    char tmpStr[255];
    sprintf(tmpStr, "h: %i\ns: %i\nv: %i", hue, sat, val);
    ofDrawBitmapString(tmpStr, 20, 260);
    
//    line.draw();
    
    predicted.draw();
    ofPushStyle();
//    ofSetColor(ofColor::red, 128);
    ofFill();
    ofDrawCircle(point, speed * 2);
    ofPopStyle();
    
    estimated.draw();
    gui.draw();

}

void ofApp::keyPressed  (int key){
    switch (key) {
        case OF_KEY_RIGHT:
            // rotate servo head to 180 degrees
            ard.sendServo(9, 18, false);
            ard.sendDigital(18, ARD_HIGH);  // pin 20 if using StandardFirmata from Arduino 0022 or older
            break;
        case OF_KEY_LEFT:
            // rotate servo head to 0 degrees
            ard.sendServo(9, 0, false);
            ard.sendDigital(18, ARD_LOW);  // pin 20 if using StandardFirmata from Arduino 0022 or older
            break;
        default:
            break;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    ofPixels huePixels = hueOutput.getPixels();
    ofPixels satPixels = satOutput.getPixels();
    ofPixels briPixels = briOutput.getPixels();
    
    hue = huePixels[x + (y * hueOutput.width)];
    sat = satPixels[x + (y * satOutput.width)];
    val = briPixels[x + (y * briOutput.width)];
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}
