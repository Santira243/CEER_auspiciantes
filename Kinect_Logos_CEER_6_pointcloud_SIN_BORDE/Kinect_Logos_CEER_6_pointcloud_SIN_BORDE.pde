// The Nature of Code
// <http://www.shiffman.net/teaching/nature>
// Spring 2011
// Box2DProcessing example

// Basic example of falling rectangles

import shiffman.box2d.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import KinectPV2.KJoint;
import KinectPV2.*;
import gab.opencv.*;


OpenCV opencv;

//PImage img;
Vec2 ubica; 

int time;
float x_track, y_track;
boolean mano_abierta;
// A reference to our box2d world
Box2DProcessing box2d;
int   zval     = 500;
// A list we'll use to track fixed objects
ArrayList<Boundary> boundaries;
ArrayList<Particle> particulas;
PImage sombra;

KinectPV2 kinect;
int xfactor = 3;//3
int yfactor = 3;//2
int zfactor = 30; //20
int offsetx =width+500;
int offsety = int(height*2.3);

float polygonFactor = 1;

int threshold = 10;

//Distance in cm
int maxD = 4500; //4.5m
int minD = 50; //50cm

boolean    contourBodyIndex = false;
boolean    skeletonIndex = false;

void setup() {
   time = millis();
   size(displayWidth, displayHeight, P3D);
   smooth();
   //img = loadImage("manaos.png");
  // Initialize box2d physics and create the world
  box2d = new Box2DProcessing(this);
  box2d.createWorld();
  // We are setting a custom gravity
  box2d.setGravity(0, -14);
//opencv = new OpenCV(this, 512, 424);
  opencv = new OpenCV(this, displayWidth, displayHeight);
  // Create ArrayLists	

  boundaries = new ArrayList<Boundary>();
  particulas =  new ArrayList<Particle>();  
  
 
  // Add a bunch of fixed boundaries
  boundaries.add(new Boundary(width/4,height-5,width/2-300,10));
  boundaries.add(new Boundary(3*width/4-20,height-5,width/2-300,10));
  boundaries.add(new Boundary(width-5,height/2,10,height));
  boundaries.add(new Boundary(5,height/2,10,height));
  
  
  
  
  kinect = new KinectPV2(this);
  
  kinect.enableDepthMaskImg(true);
  kinect.enableSkeletonDepthMap(true);
  kinect.enableSkeletonColorMap(true);
  
  kinect.enableDepthImg(true);
  kinect.enableBodyTrackImg(true);
  kinect.enablePointCloud(true);
  
  
  kinect.init();
  }

//void mousePressed() {
//  Vec2 mouse = new Vec2(mouseX,mouseY);
//   for (int i =particulas.size()-1; i >= 0; i--) 
//   {
//    Particle b = particulas.get(i);
//    Vec2 c = b.ubica().sub(mouse);
//    if ((abs(c.x)<24) && (abs(c.y)<24)) 
//    {
//      b.estado_descubierto(true);
//    }
    
//  }
//}

void draw() {
  background(0);
  
  noFill();
  strokeWeight(3);
  if (contourBodyIndex) {
     PImage profundo =  kinect.getBodyTrackImage().get(0, 0,kinect.getBodyTrackImage().width,kinect.getBodyTrackImage().height); 
    profundo.resize(displayWidth, displayHeight);
    opencv.loadImage(profundo);
    opencv.gray();
    opencv.threshold(threshold);
    PImage dst = opencv.getOutput();
  } else {
  
    //PImage profundo =  kinect.getPointCloudDepthImage().get(0, 0,kinect.getPointCloudDepthImage().width,kinect.getPointCloudDepthImage().height); 
    //profundo.resize(displayWidth, displayHeight);
    ////image(profundo,0,0);
    //opencv.loadImage(profundo);
    //opencv.gray();
    //opencv.threshold(threshold);
    //PImage dst = opencv.getOutput();
  }
    
  //sombra = opencv.getOutput();
  ArrayList<Contour> contours = opencv.findContours(false, false);

    if (contours.size() > 0) {
      for (Contour contour : contours) {
  
        contour.setPolygonApproximationFactor(polygonFactor);
        if (contour.numPoints() > 50) {
  
          stroke(0, 200, 200);
          beginShape();
  
          for (PVector point : contour.getPolygonApproximation ().getPoints()) {
            vertex(point.x , point.y );
          }
          endShape();
        }
      }
    }
  noStroke();
  //fill(0);
  //rect(0, 0, 130, 100);
  //fill(255, 0, 0);
  text("fps: "+frameRate, 20, 20);
  text("threshold: "+threshold, 20, 40);
  text("minD: "+minD, 20, 60);
  text("maxD: "+maxD, 20, 80);
  text("DisplayWidth: "+ displayWidth, 20, 100);

  kinect.setLowThresholdPC(minD);
  kinect.setHighThresholdPC(maxD);      
  
  
  // We must always step through time!
  box2d.step();
  //kinect 
    ArrayList<KSkeleton> skeletonArray =  kinect.getSkeletonDepthMap();
  ////individual JOINTS
  for (int i = 0; i < skeletonArray.size(); i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    if (skeleton.isTracked()) {
      KJoint[] joints = skeleton.getJoints();
      color col  = skeleton.getIndexColor();
      fill(col);
      stroke(col);
      if(skeletonIndex){
        pushMatrix();      
        drawBody(joints);
        //draw different color for each hand state
        drawHandState(joints[KinectPV2.JointType_HandRight],'r');
        drawHandState(joints[KinectPV2.JointType_HandLeft],'l');
        popMatrix();
      }
    }
  }
   
      
  fill(255, 0, 0);
  text(frameRate, 50, 50);
   if (time+3000 < millis()){
    //Box p = new Box(random(width),10);
    time = millis();
    if (particulas.size()<10)
    {
    Vec2 a = new Vec2(random(width/2+100),random(height/2));
    Particle pa = new Particle(a); //donde aparecen.
    particulas.add(pa);
    }
  }
  
  // Display all the boundaries
  for (Boundary wall: boundaries) {
    wall.display();
  }

  for (Particle b: particulas) {
     Vec2 a = new Vec2(random(width),random(height));
     b.attract(a);
    }
    
 
   // Display all the caritas
  for (Particle ba: particulas) {
    ba.display();
  }

  // Boxes that leave the screen, we delete them
  // (note they have to be deleted from both the box2d world and our list
  for (int i =particulas.size()-1; i >= 0; i--) {
    Particle b = particulas.get(i);
    if (b.done()) {
      particulas.remove(i);
      println("Se fuee");
    }
  }
  
  fill(0);

}

void drawBody(KJoint[] joints) {
  drawBone(joints, KinectPV2.JointType_Head, KinectPV2.JointType_Neck);
  drawBone(joints, KinectPV2.JointType_Neck, KinectPV2.JointType_SpineShoulder);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_SpineMid);
  drawBone(joints, KinectPV2.JointType_SpineMid, KinectPV2.JointType_SpineBase);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderRight);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderLeft);
  drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipRight);
  drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipLeft);

  // Right Arm
  drawBone(joints, KinectPV2.JointType_ShoulderRight, KinectPV2.JointType_ElbowRight);
  drawBone(joints, KinectPV2.JointType_ElbowRight, KinectPV2.JointType_WristRight);
  drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_HandRight);
  drawBone(joints, KinectPV2.JointType_HandRight, KinectPV2.JointType_HandTipRight);
  drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_ThumbRight);

  // Left Arm
  drawBone(joints, KinectPV2.JointType_ShoulderLeft, KinectPV2.JointType_ElbowLeft);
  drawBone(joints, KinectPV2.JointType_ElbowLeft, KinectPV2.JointType_WristLeft);
  drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_HandLeft);
  drawBone(joints, KinectPV2.JointType_HandLeft, KinectPV2.JointType_HandTipLeft);
  drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_ThumbLeft);

  // Right Leg
  drawBone(joints, KinectPV2.JointType_HipRight, KinectPV2.JointType_KneeRight);
  drawBone(joints, KinectPV2.JointType_KneeRight, KinectPV2.JointType_AnkleRight);
  drawBone(joints, KinectPV2.JointType_AnkleRight, KinectPV2.JointType_FootRight);

  // Left Leg
  drawBone(joints, KinectPV2.JointType_HipLeft, KinectPV2.JointType_KneeLeft);
  drawBone(joints, KinectPV2.JointType_KneeLeft, KinectPV2.JointType_AnkleLeft);
  drawBone(joints, KinectPV2.JointType_AnkleLeft, KinectPV2.JointType_FootLeft);

  drawJoint(joints, KinectPV2.JointType_HandTipLeft);
  drawJoint(joints, KinectPV2.JointType_HandTipRight);
  drawJoint(joints, KinectPV2.JointType_FootLeft);
  drawJoint(joints, KinectPV2.JointType_FootRight);

  drawJoint(joints, KinectPV2.JointType_ThumbLeft);
  drawJoint(joints, KinectPV2.JointType_ThumbRight);

  drawJoint(joints, KinectPV2.JointType_Head);
}

//draw joint
void drawJoint(KJoint[] joints, int jointType) {
  
  pushMatrix();
  //translate(offsetx + joints[jointType].getX()/xfactor, offsety+joints[jointType].getY()/yfactor, joints[jointType].getZ()/zfactor);
  translate(offsetx +joints[jointType].getX(),offsety+joints[jointType].getY(), joints[jointType].getZ());
  ellipse(0, 0, 15, 15);
  popMatrix();
 
 }

//draw bone
void drawBone(KJoint[] joints, int jointType1, int jointType2) {
  //stroke(200,0,0,60);
  //strokeWeight(5);
  pushMatrix();
  //translate(offsetx + joints[jointType1].getX()/xfactor, offsety + joints[jointType1].getY()/yfactor, joints[jointType1].getZ()/zfactor);
  translate(offsetx +joints[jointType1].getX(), offsety +joints[jointType1].getY(), joints[jointType1].getZ());
  //ellipse(0, 0, 10, 10);
  ellipse(0, 0, 25, 25);
  popMatrix();
  //line(offsetx +joints[jointType1].getX()/xfactor,offsety+ joints[jointType1].getY()/yfactor, joints[jointType1].getZ()/zfactor, offsetx+ joints[jointType2].getX()/xfactor, offsety+joints[jointType2].getY()/yfactor, joints[jointType2].getZ()/zfactor);
  line(offsetx +joints[jointType1].getX(), offsety+joints[jointType1].getY(), joints[jointType1].getZ(), offsetx+joints[jointType2].getX(),offsety+ joints[jointType2].getY(), joints[jointType2].getZ());
}

//draw hand state
void drawHandState(KJoint joint, char a) {
  noStroke();
  handState(joint.getState());
  pushMatrix();
  //translate(offsetx + joint.getX()/xfactor, offsety+joint.getY()/yfactor, joint.getZ()/zfactor);
  translate(offsetx +joint.getX(), offsety+joint.getY(), joint.getZ());
  ellipse(0, 0, 10, 10);
  //x_track = offsetx+ joint.getX()/xfactor;
  //y_track = offsety+ joint.getY()/yfactor;
  
  x_track = offsetx+ joint.getX();
  y_track = offsety+ joint.getY();
  
  popMatrix();
  Vec2 hand = new Vec2(x_track ,y_track);
   for (int i =particulas.size()-1; i >= 0; i--) 
   {
    Particle b = particulas.get(i);
    Vec2 c = b.ubica().sub(hand);
    if ((abs(c.x)<24) && (abs(c.y)<24)) 
    {
      b.estado_descubierto(true);
    }
    
  }
}

/*
Different hand state
 KinectPV2.HandState_Open
 KinectPV2.HandState_Closed
 KinectPV2.HandState_Lasso
 KinectPV2.HandState_NotTracked
 */
void handState(int handState) {
  switch(handState) {
  case KinectPV2.HandState_Open:
    fill(0, 255, 0);
    mano_abierta = true;
    break;
  case KinectPV2.HandState_Closed:
    fill(255, 0, 0);
    mano_abierta = false;
    break;
  case KinectPV2.HandState_Lasso:
    fill(0, 0, 255);
    break;
  case KinectPV2.HandState_NotTracked:
    fill(255, 255, 255);
    break;
  }
}

/*
// BFG function
void rotateXYZ(float xx, float yy, float zz) {
float cx, cy, cz, sx, sy, sz;

cx = cos(xx);
cy = cos(yy);
cz = cos(zz);
sx = sin(xx);
sy = sin(yy);
sz = sin(zz);

applyMatrix(cy*cz, (cz*sx*sy)-(cx*sz), (cx*cz*sy)+(sx*sz), 0.0,
cy*sz, (cx*cz)+(sx*sy*sz), (-cz*sx)+(cx*sy*sz), 0.0,
-sy, cy*sx, cx*cy, 0.0,
0.0, 0.0, 0.0, 1.0);
}
*/

void keyPressed() {
  //change contour finder from contour body to depth-PC
  if( key == 'b'){
   contourBodyIndex = !contourBodyIndex;
   if(contourBodyIndex)
     threshold = 200;
    else
     threshold = 40;
  }
  
  if( key == 'e'){
   skeletonIndex = !skeletonIndex;
  }

  if (key == 'a') {
    threshold+=1;
  }
  if (key == 's') {
    threshold-=1;
  }

  if (key == '1') {
    minD += 10;
  }

  if (key == '2') {
    minD -= 10;
  }

  if (key == '3') {
    maxD += 10;
  }

  if (key == '4') {
    maxD -= 10;
  }

  if (key == '5')
    polygonFactor += 0.1;

  if (key == '6')
    polygonFactor -= 0.1;
}
