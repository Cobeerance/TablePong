/*
Thomas Sanchez Lengeling.
http://codigogenerativo.com/

KinectPV2, Kinect for Windows v2 library for processing

Point Cloud example in a 2d Image, with threshold example
*/

import blobDetection.*;
import KinectPV2.KJoint;
import KinectPV2.*;

BlobDetection theBlobDetection;
PGraphics img;
KinectPV2 kinect;

//Distance Threashold
int maxD = 1430; // 4.5mx
int minD = 1310;  //  50cm

int ksx1 = 113, ksy1 = 148, kswidth = 252, ksheight = 152;
float kscalex, kscaley;
int dsx1 = 602, dsy1 = 362, dswidth = 726, dsheight = 447;
boolean kcal = false;
boolean dcal = false;
boolean dblob = false;
boolean dvis = false;
boolean dgame = true;

PGraphics kthresimg;

Spot[] glowspots;
int curspot=0, nspots=100; 

void setup() {
  //size(1024, 424, P3D);
  fullScreen();

  kinect = new KinectPV2(this);

  //Enable point cloud
  kinect.enableColorImg(true);
  kinect.enableDepthImg(true);
  kinect.enablePointCloud(true);

  kinect.init();

  kthresimg = createGraphics(512, 424);
  kthresimg.beginDraw();
  kthresimg.background(0);
  kthresimg.noStroke();
  kthresimg.fill(0);
  kthresimg.endDraw();
  
  theBlobDetection = new BlobDetection(512, 424);
  theBlobDetection.setPosDiscrimination(false);
  theBlobDetection.setThreshold(0.1f);//0.38f);
  
  kscalex = float(width) / kswidth;
  kscaley = float(height) / ksheight;
  
  glowspots = new Spot[nspots];
  for (int i=0; i < glowspots.length; i++) {
    glowspots[i] = new Spot(0,0);
  }
}

void draw() {

  //obtain the raw depth data in integers from [0 - 4500]
  int [] rawData = kinect.getRawDepthData();

  //Threahold of the point Cloud.
  kinect.setLowThresholdPC(minD);
  kinect.setHighThresholdPC(maxD);
  

  if (dgame){
    // displaygame
    //background(0);
    drawvis();
  } else {
    background(0);
    //image(kinect.getDepthImage(), 0, 0);
    image(kinect.getColorImage(), 0, 0);
  
    /* obtain the point cloud as a PImage
     * Each pixel of the PointCloudDepthImage corresponds to the Z value
     * of Point Cloud i.e. distances.
     * The Point cloud values are mapped from (0 - 4500) mm  to gray color format (0 - 255)
     */
    image(kinect.getPointCloudDepthImage(), 0, 0);
    kthresimg.beginDraw();
    kthresimg.image(kinect.getPointCloudDepthImage(), 0, 0);
    kthresimg.endDraw();
    image(kthresimg, 512, 0);
    
    if (kcal) {
      drawkcal(); // Draw calibration surface
    }
    if (dcal) {
      drawdcal(); // Draw calibration surface
    }
    if (dblob){
      //drawblob();
      theBlobDetection.computeBlobs(kthresimg.pixels);
      pushMatrix();
      if (dvis){
        scale(-kscalex, kscaley);
        translate(-ksx1 - kswidth,-ksy1);
      }
      drawBlobsAndEdges(true, true);
      popMatrix();
    }
  }
  fill(128, 128, 128);
  text("fps: "+frameRate, 50, 50);
  text("g = exit / enter game mode\nc = calibrate\nb = show blobs\nv = apply calibration", 50, 100);
}

void keyPressed() {
  if (key == '1') {
    minD += 10;
    println("Change min: "+minD);
  }

  if (key == '2') {
    minD -= 10;
    println("Change min: "+minD);
  }

  if (key == '3') {
    maxD += 10;
    println("Change max: "+maxD);
  }

  if (key == '4') {
    maxD -=10;
    println("Change max: "+maxD);
  }
  
  if (key == 'c') {
    // calibrate kinect surface
    kcal = !kcal;
    dcal = false;
  }
  if (key == 'd') {
    // calibrate display surface
    dcal = !dcal;
    kcal = false;
  }
  if (key == 'b'){
    // draw blobs
    dblob = !dblob;
  }
  if (key == 'v'){
    // draw visuals
    dvis = !dvis;
  }
  if (key == 'g'){
    // enter game mode
    dgame = !dgame;
    if (dgame) background(0);
  }
}

void mousePressed(){
  if (kcal){
    // collect calibration points
    if (mouseButton == LEFT){
      kswidth += ksx1 - mouseX;
      ksheight += ksy1 - mouseY;
      ksx1 = mouseX;
      ksy1 = mouseY;
      
    } else if (mouseButton == RIGHT){
      kswidth = mouseX - ksx1;
      ksheight = mouseY - ksy1;
    }
    kscalex = float(width) / kswidth;
    kscaley = float(height) / ksheight;
    println("Kinect Surface: (", ksx1, ", ", ksy1, ") - (", ksx1 + kswidth, ", ", ksy1 + ksheight, ")");
  }
  if (dcal){
    // collect calibration points
    if (mouseButton == LEFT){
      dswidth += dsx1 - mouseX;
      dsheight += dsy1 - mouseY;
      dsx1 = mouseX;
      dsy1 = mouseY;
    } else if (mouseButton == RIGHT){
      dswidth = mouseX - dsx1;
      dsheight = mouseY - dsy1;
    }
    println("Display Surface: (", dsx1, ", ", dsy1, ") - (", dsx1 + dswidth, ", ", dsy1 + dsheight, ")");
  }
}

void drawkcal(){
  fill(color(0,128,0), 50);
  ellipse(ksx1, ksy1, 10, 10);
  ellipse(ksx1 + kswidth, ksy1, 10, 10);
  ellipse(ksx1, ksy1 + ksheight, 10, 10);
  ellipse(ksx1 + kswidth, ksy1 + ksheight, 10, 10);
  rect(ksx1, ksy1, kswidth, ksheight);
  
  fill(0,255,0);
  int rectwidth = 50;
  rect(0, 0, rectwidth, rectwidth); rect(width-rectwidth, 0, rectwidth, rectwidth); rect(0, height-rectwidth, rectwidth, rectwidth); rect(width-rectwidth, height-rectwidth, rectwidth, rectwidth);
  rect(0, 0, width, rectwidth/2); rect(0, height - rectwidth/2, width, rectwidth/2);
  rect(0, 0, rectwidth/2, height); rect(width - rectwidth/2, 0, rectwidth/2, height);
}

void drawdcal(){
  fill(color(128, 0, 0), 50);
  ellipse(dsx1, dsy1, 10, 10);
  ellipse(dsx1 + dswidth, dsy1, 10, 10);
  ellipse(dsx1, dsy1 + dsheight, 10, 10);
  ellipse(dsx1 + dswidth, dsy1 + dsheight, 10, 10);
  rect(dsx1, dsy1, dswidth, dsheight);
}

void drawvis(){
  // draw visuals
  // Setup buffer image for blob calculation
  kthresimg.beginDraw();
  kthresimg.image(kinect.getPointCloudDepthImage(), 0, 0);
  kthresimg.endDraw();
  
  fill(0,10);
  rect(0,0,width,height);
  pushMatrix();
  scale(-kscalex, kscaley);
  translate(-ksx1 - kswidth,-ksy1);
  
  getblobs();
  strokeWeight(1);
  fill(255);
  stroke(255);
  drawblobs(0);
  
  for (int i=0; i < glowspots.length; i++) {
    glowspots[i].grow();
    glowspots[i].display();
  }
  
  popMatrix();
}

void getblobs(){
  // get the blobs from the blob algorithm
  theBlobDetection.computeBlobs(kthresimg.pixels);
}

void drawblobs(int nsparks){
  // draw the shapes around the found blobs
  Blob b;
  EdgeVertex eA, eB;
  for (int n=0; n<theBlobDetection.getBlobNb(); n++)
  {
    b=theBlobDetection.getBlob(n);
    if (b!=null)
    {
      if ((b.xMin*kthresimg.width > ksx1) && (b.yMin*kthresimg.height > ksy1) && ((b.xMin*kthresimg.width + b.w*kthresimg.width) < (ksx1 + kswidth)) && ((b.yMin*kthresimg.height + b.h*kthresimg.height) < (ksy1 + ksheight))){
        if (nsparks > 0) {
          // draw sparkles
          pushMatrix();
          translate(b.xMin*kthresimg.width, b.yMin*kthresimg.height);
          for (int i=0; i<nsparks; i++){
            float r = random(10, 20);
            ellipse(random(b.w*kthresimg.width), random(b.h*kthresimg.height), r, r);
          }
          popMatrix();
          curspot++;
          curspot = curspot % nspots;
          glowspots[curspot].set(b.xMin*kthresimg.width + random(b.w*kthresimg.width),
                                b.yMin*kthresimg.height + random(b.h*kthresimg.height));
        } else {
        ellipse(b.xMin*kthresimg.width + b.w*kthresimg.width / 2.0,
          b.yMin*kthresimg.height + b.h*kthresimg.height/2.0,
          b.w*kthresimg.width, b.h*kthresimg.height);
        }
      }
      //rect(b.xMin*kthresimg.width, b.yMin*kthresimg.height, b.w*kthresimg.width, b.h*kthresimg.height);
    }
  }
}



// ==================================================
// drawBlobsAndEdges()
// ==================================================
void drawBlobsAndEdges(boolean drawBlobs, boolean drawEdges)
{
  noFill();
  Blob b;
  EdgeVertex eA, eB;
  for (int n=0; n<theBlobDetection.getBlobNb(); n++)
  {
    b=theBlobDetection.getBlob(n);
    if (b!=null)
    {
      // Edges
      if (drawEdges)
      {
        strokeWeight(2);
        stroke(0, 255, 0);
        for (int m=0; m<b.getEdgeNb(); m++)
        {
          eA = b.getEdgeVertexA(m);
          eB = b.getEdgeVertexB(m);
          if (eA !=null && eB !=null)
            line(
              eA.x*kthresimg.width, eA.y*kthresimg.height, 
              eB.x*kthresimg.width, eB.y*kthresimg.height
              );
        }
      }

      // Blobs
      if (drawBlobs)
      {
        strokeWeight(1);
        stroke(255, 0, 0);
        rect(
          b.xMin*kthresimg.width, b.yMin*kthresimg.height, 
          b.w*kthresimg.width, b.h*kthresimg.height
          );
      }
    }
  }
}


class Spot {
  float x, y;         // X-coordinate, y-coordinate
  float diameter = 0;     // Diameter of the circle
  float gspeed = 1;        // diameter growth speed
  int alpha = 255; // Brightness of spot
  float aspeed = 5; // brightness dimming speed
  
  // Constructor
  Spot(float xpos, float ypos) {
    x = xpos;
    y = ypos;
  }
  
  void set(float xpos, float ypos) {
    x = xpos;
    y = ypos;
    diameter = 0;
    gspeed = 1;
    alpha = 255;
    aspeed = 5;
  }
    
  void grow() {
    alpha -= aspeed;
    diameter += gspeed;
    if (alpha <= 0) { 
      alpha = 0;
      diameter = 0;
    } 
  }
  
  void display() {
    fill(255, alpha);
    ellipse(x, y, diameter, diameter);
  }
}