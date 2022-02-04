/**
 **********************************************************************************************************************
 * @file       maze.pde
 * @author     Elizabeth Reid, modified from hello wall by Steve Ding, Colin Gallacher
 * @version    V3.0.0
 * @date       03-Febuary-2022
 * @brief      Maze example adapted from Hello Wall
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */
 
  /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  


/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;

/* pantagraph link parameters in meters */
float             l                                   = 0.07;
float             L                                   = 0.09;


/* end effector radius in meters */
float             rEE                                 = 0.003;

/* virtual wall parameter  */
float             kWall                               = 100;
float             hWall                               = 1;
PVector           fWall                               = new PVector(0, 0);
PVector           penWall                             = new PVector(0, 0);
// outer walls
PVector           posWall                             = new PVector(0, 0.13);
PVector           posWall2                            = new PVector(-0.07, 0.05);
PVector           posWall3                            = new PVector(0.03, 0.05);
PVector           posWall4                            = new PVector(-0.0075, 0.05);
// inner walls
PVector           posWall5                            = new PVector(0.015, 0.0625);
PVector           posWall6                            = new PVector(0.0225, 0.075);
PVector           posWall7                            = new PVector(-0.025, 0.0625);
PVector           posWall8                            = new PVector(-0.04, 0.05);
PVector           posWall9                            = new PVector(-0.055, 0.0625);
PVector           posWall10                           = new PVector(0, 0.0875);
PVector           posWall11                           = new PVector(0.015, 0.0875);
PVector           posWall12                           = new PVector(0.015, 0.1175);
PVector           posWall13                           = new PVector(0, 0.105);
PVector           posWall14                           = new PVector(-0.0215, 0.101);
PVector           posWall15                           = new PVector(-0.025, 0.0875);
PVector           posWall16                           = new PVector(-0.008, 0.116);

// river
PVector           posRiver                            = new PVector(-0.045, 0.035); 
PVector           posRiver2                           = new PVector(0.01, 0.035);

// x length for wall
float wall_1_xlen = 0.05;
float wall_2_xlen = 0.02; // outer left wall
float wall_3_xlen = 0.02; // outer right wall
float wall_4_xlen = 0.0425; // outer top wall
// inner walls
float wall_5_xlen = 0.02;
float wall_6_xlen = 0.0275;
float wall_7_xlen = 0.02;
float wall_8_xlen = 0.02;
float wall_9_xlen = 0.02;
float wall_10_xlen = 0.035;
float wall_11_xlen = 0.02;
float wall_12_xlen = 0.02;
float wall_13_xlen = 0.02;
float wall_14_xlen = 0.0275;
float wall_15_xlen = 0.02;
float wall_16_xlen = 0.0275;

// river
float river_xlen = 0.08;


/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

/* World boundaries reference */
final int         worldPixelWidth                     = 1000;
final int         worldPixelHeight                    = 650;


/* graphical elements */
PShape pGraph, joint, endEffector;
PShape wall;
PShape wall2;
PShape wall3;
PShape wall4;
PShape wall5;
PShape wall6;
PShape wall7;
PShape wall8;
PShape wall9;
PShape wall10;
PShape wall11;
PShape wall12;
PShape wall13;
PShape wall14;
PShape wall15;
PShape wall16;
PShape river;
PShape river2;

// background and other image
PImage img;
PImage chest;
PImage second_background;

// for treasure effects
boolean treasure_stolen = false; 
boolean mist = false;

/* end elements definition *********************************************************************************************/ 



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 650);
  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */ 
  haplyBoard          = new Board(this, "COM4", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);

  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();
    
  
  /* visual elements setup */
  background(0);
  deviceOrigin.add(worldPixelWidth/2, 0);
  
  /* create pantagraph graphics */
  create_pantagraph();
  
  /* create wall graphics */
  wall = create_wall(posWall.x-wall_1_xlen, posWall.y+rEE, posWall.x+wall_1_xlen, posWall.y+rEE);
  wall.setStroke(color(0));
  
  wall2 = create_wall(posWall2.x-wall_2_xlen+0.04, posWall2.y+rEE, posWall2.x+wall_2_xlen, posWall2.y+rEE + 0.08);
  wall2.setStroke(color(0));
  
  wall3 = create_wall(posWall3.x-wall_3_xlen+0.04, posWall3.y+rEE, posWall3.x+wall_3_xlen, posWall3.y+rEE + 0.08);
  wall3.setStroke(color(0));
  
  wall4 = create_wall(posWall4.x-wall_4_xlen, posWall4.y+rEE, posWall4.x+wall_4_xlen, posWall4.y+rEE);
  wall4.setStroke(color(0));
  
  wall5 = create_wall(posWall5.x-wall_5_xlen, posWall5.y+rEE, posWall5.x+wall_5_xlen, posWall5.y+rEE);
  wall5.setStroke(color(0));
  
  wall6 = create_wall(posWall6.x-wall_6_xlen, posWall6.y+rEE, posWall6.x+wall_6_xlen, posWall6.y+rEE);
  wall6.setStroke(color(0));
  
  wall7 = create_wall(posWall7.x-wall_7_xlen+0.04, posWall7.y+rEE, posWall7.x+wall_7_xlen, posWall7.y+rEE + 0.0125);
  wall7.setStroke(color(0));
  
  wall8 = create_wall(posWall8.x-wall_8_xlen+0.04, posWall8.y+rEE, posWall8.x+wall_8_xlen, posWall8.y+rEE + 0.025);
  wall8.setStroke(color(0));
  
  wall9 = create_wall(posWall9.x-wall_9_xlen+0.04, posWall9.y+rEE, posWall9.x+wall_9_xlen, posWall9.y+rEE + 0.025);
  wall9.setStroke(color(0));
  
  wall10 = create_wall(posWall10.x-wall_10_xlen, posWall10.y+rEE, posWall10.x+wall_10_xlen, posWall10.y+rEE);
  wall10.setStroke(color(0));
  
  wall11 = create_wall(posWall11.x-wall_11_xlen+0.04, posWall11.y+rEE, posWall11.x+wall_11_xlen, posWall11.y+rEE + 0.0125);
  wall11.setStroke(color(0));
  
  wall12 = create_wall(posWall12.x-wall_12_xlen+0.04, posWall12.y+rEE, posWall12.x+wall_12_xlen, posWall12.y+rEE + 0.0125);
  wall12.setStroke(color(0));
  
  wall13 = create_wall(posWall13.x-wall_13_xlen+0.04, posWall13.y+rEE, posWall13.x+wall_13_xlen, posWall13.y+rEE + 0.025);
  wall13.setStroke(color(0));
  
  wall14 = create_wall(posWall14.x-wall_14_xlen, posWall14.y+rEE, posWall14.x+wall_14_xlen, posWall14.y+rEE);
  wall14.setStroke(color(0));
  
  wall15 = create_wall(posWall15.x-wall_15_xlen+0.04, posWall15.y+rEE, posWall15.x+wall_15_xlen, posWall15.y+rEE + 0.0125);
  wall15.setStroke(color(0));
  
  wall16 = create_wall(posWall16.x-wall_16_xlen, posWall16.y+rEE, posWall16.x+wall_16_xlen, posWall16.y+rEE);
  wall16.setStroke(color(0));
  
  river2 = create_river(posRiver2.x + 0.04, posRiver2.y, posRiver2.x - 0.06, posRiver2.y/2);
  river = create_river(posRiver.x-river_xlen, posRiver.y, posRiver.x+river_xlen, posRiver.y/2);
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
  
  // load images
  img = loadImage("Background.png");
  second_background = loadImage("SecondBackground.png");
  chest = loadImage("Chest.png");
  chest.resize(50,0);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(152,190,100);
    image(img, 0, 48);
    if (mist) {
      image(second_background, 0, 48);
    }
    image(chest, 280, 190);
    update_animation(angles.x*radsPerDegree, angles.y*radsPerDegree, posEE.x, posEE.y);
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(device_to_graphics(posEE)); 
      
      
      /* haptic wall force calculation */
      fWall.set(0, 0);
      
      float force_offset = 0; // to account for weakness when the end effector is perpendicular to the motors
      if (posEE.x > -0.03 && posEE.x < 0.03) {
        force_offset = 0.1;
      }
      float height_offset = (posWall.y - (posEE.y + rEE)); // to account for the difference in force close and far from the motors
      
      // the very last wall is also a bit weak
      if (height_offset < 0.01) {
        height_offset = 0.06;
      }
      else if (height_offset < 0.03) {
        height_offset = 0.03;
      }
      
      penWall.set(0, 2 * height_offset + 0.05 + force_offset);

      // for this wall, want to move right and left, not up and down
      PVector rightPenWall = new PVector(0.17 + (force_offset/2), 0);
      // for short vertical walls
      PVector shortWall = new PVector(0.2 + (force_offset/2),0);
      
      // positions of outer walls
      float wall_1_y = posWall.y - (posEE.y + rEE);
      float wall_1_x1 = posWall.x - wall_1_xlen - posEE.x;
      float wall_1_x2 = posWall.x + wall_1_xlen - posEE.x;
      
      float wall_2_y1 = posWall2.y - (posEE.y + rEE);
      float wall_2_y2 = posWall2.y - (posEE.y + rEE) + 0.08; 
      float wall_2_x = posWall2.x + wall_2_xlen - (posEE.x + rEE);
      
      float wall_3_y1 = posWall3.y - (posEE.y + rEE);
      float wall_3_y2 = posWall3.y - (posEE.y + rEE) + 0.08; 
      float wall_3_x = posWall3.x + wall_3_xlen - (posEE.x + rEE);
      
      float wall_4_y = posWall4.y - (posEE.y + rEE);
      float wall_4_x1 = posWall4.x - wall_4_xlen - posEE.x;
      float wall_4_x2 = posWall4.x + wall_4_xlen - posEE.x;
      
      // positions of inner walls
      float wall_5_y = posWall5.y - (posEE.y + rEE);
      float wall_5_x1 = posWall5.x - wall_5_xlen - posEE.x;
      float wall_5_x2 = posWall5.x + wall_5_xlen - posEE.x;
      
      float wall_6_y = posWall6.y - (posEE.y + rEE);
      float wall_6_x1 = posWall6.x - wall_6_xlen - posEE.x;
      float wall_6_x2 = posWall6.x + wall_6_xlen - posEE.x;
      
      float wall_7_y1 = posWall7.y - (posEE.y + rEE) + 0.0025;
      float wall_7_y2 = posWall7.y - (posEE.y + rEE) + 0.0175; 
      float wall_7_x = posWall7.x + wall_7_xlen - (posEE.x + rEE);
      
      float wall_8_y1 = posWall8.y - (posEE.y + rEE) + 0.003;
      float wall_8_y2 = posWall8.y - (posEE.y + rEE) + 0.0325; 
      float wall_8_x = posWall8.x + wall_8_xlen - (posEE.x + rEE);
      
      float wall_9_y1 = posWall9.y - (posEE.y + rEE) + 0.003;
      float wall_9_y2 = posWall9.y - (posEE.y + rEE) + 0.0325; 
      float wall_9_x = posWall9.x + wall_9_xlen - (posEE.x + rEE);
      
      float wall_10_y = posWall10.y - (posEE.y + rEE);
      float wall_10_x1 = posWall10.x - wall_10_xlen - posEE.x;
      float wall_10_x2 = posWall10.x + wall_10_xlen - posEE.x;
      
      float wall_11_y1 = posWall11.y - (posEE.y + rEE) + 0.0025;
      float wall_11_y2 = posWall11.y - (posEE.y + rEE) + 0.0175; 
      float wall_11_x = posWall11.x + wall_11_xlen - (posEE.x + rEE);
      
      float wall_12_y1 = posWall12.y - (posEE.y + rEE) + 0.0025;
      float wall_12_y2 = posWall12.y - (posEE.y + rEE) + 0.0175; 
      float wall_12_x = posWall12.x + wall_12_xlen - (posEE.x + rEE);
      
      float wall_13_y1 = posWall13.y - (posEE.y + rEE) + 0.003;
      float wall_13_y2 = posWall13.y - (posEE.y + rEE) + 0.0325; 
      float wall_13_x = posWall13.x + wall_13_xlen - (posEE.x + rEE);
      
      float wall_14_y = posWall14.y - (posEE.y + rEE);
      float wall_14_x1 = posWall14.x - wall_14_xlen - posEE.x;
      float wall_14_x2 = posWall14.x + wall_14_xlen - posEE.x;
      
      float wall_15_y1 = posWall15.y - (posEE.y + rEE) + 0.0025;
      float wall_15_y2 = posWall15.y - (posEE.y + rEE) + 0.0175; 
      float wall_15_x = posWall15.x + wall_15_xlen - (posEE.x + rEE);
      
      float wall_16_y = posWall16.y - (posEE.y + rEE);
      float wall_16_x1 = posWall16.x - wall_16_xlen - posEE.x;
      float wall_16_x2 = posWall16.x + wall_16_xlen - posEE.x;
      
      // river 
      float river_y = posRiver.y - (posEE.y + rEE);
      float river_x1 = posRiver.x - river_xlen - posEE.x;
      float river_x2 = posRiver.x + river_xlen - posEE.x;
      
      float river2_y = posRiver2.y - (posEE.y + rEE);
      float river2_x1 = posRiver2.x + 0.04 - posEE.x;
      float river2_x2 = posRiver2.x + 0.1 - posEE.x;
      
      
      // outer walls
      calculate_horizontal_wall_force(wall_1_y, wall_1_x1, wall_1_x2, penWall);
      calculate_horizontal_wall_force(wall_4_y, wall_4_x1, wall_4_x2, penWall);
      calculate_vertical_wall_force(wall_2_x, wall_2_y1, wall_2_y2, rightPenWall);
      calculate_vertical_wall_force(wall_3_x, wall_3_y1, wall_3_y2, rightPenWall);
      
      // inner walls
      calculate_horizontal_wall_force(wall_5_y, wall_5_x1, wall_5_x2, penWall);
      calculate_horizontal_wall_force(wall_6_y, wall_6_x1, wall_6_x2, penWall);
      calculate_vertical_wall_force(wall_7_x, wall_7_y1, wall_7_y2, shortWall);
      calculate_vertical_wall_force(wall_8_x, wall_8_y1, wall_8_y2, rightPenWall);
      calculate_vertical_wall_force(wall_9_x, wall_9_y1, wall_9_y2, rightPenWall);
      calculate_horizontal_wall_force(wall_10_y, wall_10_x1, wall_10_x2, penWall);
      calculate_vertical_wall_force(wall_11_x, wall_11_y1, wall_11_y2, shortWall);
      calculate_vertical_wall_force(wall_12_x, wall_12_y1, wall_12_y2, shortWall);
      calculate_vertical_wall_force(wall_13_x, wall_13_y1, wall_13_y2, rightPenWall);
      calculate_horizontal_wall_force(wall_14_y, wall_14_x1, wall_14_x2, penWall);
      calculate_vertical_wall_force(wall_15_x, wall_15_y1, wall_15_y2, shortWall);
      calculate_horizontal_wall_force(wall_16_y, wall_16_x1, wall_16_x2, penWall);
      
      // river
      calculate_river_force(river_y, river_x1, river_x2, posRiver);
      calculate_river_force(river2_y, river2_x1, river2_x2, posRiver2);
      
      fEE = (fWall.copy()).mult(-1);
      fEE.set(graphics_to_device(fEE));
      /* end haptic wall force calculation */
    }
    
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/


/* helper functions section, place helper functions here ***************************************************************/
void create_pantagraph(){
  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;
  float rEEAni = pixelsPerMeter * (rEE/2);
  
  pGraph = createShape();
  pGraph.beginShape();
  pGraph.fill(255);
  pGraph.stroke(0);
  pGraph.strokeWeight(2);
  
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.endShape(CLOSE);
  
  joint = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, rEEAni, rEEAni);
  joint.setStroke(color(0));
  
  fill(127,0,0);
  endEffector = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, 2*rEEAni, 2*rEEAni);
  endEffector.setStroke(color(0));
  strokeWeight(5);
  
}

void calculate_horizontal_wall_force(float wall_y, float wall_x1, float wall_x2, PVector pen_wall) {
  if(wall_y < -0.003 && wall_y > -0.009 && wall_x1 < 0 && wall_x2 > 0){
        // make sure the force is applied outward from the wall, whatever side we're on
        float wallForce = hWall; 
        if (wall_y < -0.005) {
          wallForce = -hWall;
        }
        fWall = fWall.add(pen_wall.mult(wallForce)); // updates this global variable 
      }
}

void calculate_vertical_wall_force(float wall_x, float wall_y1, float wall_y2, PVector pen_wall) {
  if(wall_y1 < -0.001 && wall_y2 > 0 && wall_x < 0 && wall_x > -0.007){
        // make sure the force is applied outward from the wall, whatever side we're on
        float wallForce = -hWall;
        if (wall_x > -0.002) {
          wallForce = hWall;
        }
        fWall = fWall.add(pen_wall.mult(wallForce));  
      }
}

void calculate_river_force(float y, float x1, float x2, PVector riverPos) {
  if(y < -0.003 && y > -0.02 && x1 < 0 && x2 > 0){
        // keep in center of river
        PVector riverWall;
        if (y > -0.01) {
          riverWall = new PVector((riverPos.x - (posEE.y + rEE)), 0);
        }
        else {
          print("here");
          riverWall = new PVector((riverPos.x - (posEE.y + rEE)), 2*(riverPos.y - (posEE.y + rEE)));
        }
        float wallForce = -kWall;
        fWall = fWall.add(riverWall.mult(wallForce));  
      }
}


PShape create_wall(float x1, float y1, float x2, float y2){
  x1 = pixelsPerMeter * x1;
  y1 = pixelsPerMeter * y1;
  x2 = pixelsPerMeter * x2;
  y2 = pixelsPerMeter * y2;
  
  return createShape(LINE, deviceOrigin.x + x1, deviceOrigin.y + y1, deviceOrigin.x + x2, deviceOrigin.y+y2);
}

PShape create_river(float x1, float y1, float x2, float y2){
  x1 = pixelsPerMeter * x1;
  y1 = pixelsPerMeter * y1;
  x2 = pixelsPerMeter * x2;
  y2 = pixelsPerMeter * y2;
  
  return createShape(RECT, deviceOrigin.x + x1, deviceOrigin.y + y1, deviceOrigin.x + x2, deviceOrigin.y+y2);
}


void update_animation(float th1, float th2, float xE, float yE){
  background(152,190,100);
  image(img, 0, 48);
  
  // treasure effects
  if (posEE.x >= 0.005 && posEE.x < 0.015 && posEE.y <= 0.13 && posEE.y > 0.12) {
    treasure_stolen = true;
    mist = true;
  }
  else if (treasure_stolen == false){
    image(chest, 520, 490);
    // instructions
    String s = "Try to steal the treasure chest from the castle!";
    fill(0);
    textSize(20);
    text(s, 40, 40, 280, 320);
  }
  
  // display sucess message if we get out of the maze with the treasure
  if (treasure_stolen && posEE.y < 0.048) {
    mist = false;
    String s = "Congratulations, you made it out! Now, go find somewhere to spend all that gold.";
    fill(0);
    textSize(20);
    text(s, 40, 40, 280, 320);
  }
  
  if (mist) {
      image(second_background, 0, 48);
      String s = "Oh no, looks like the treasure chest was booby trapped! There's mist everywhere... You'll have to try and feel your way out of the castle!";
      fill(0);
      textSize(20);
      text(s, 40, 40, 280, 320);
    }
  
  //float lAni = pixelsPerMeter * l;
  //float LAni = pixelsPerMeter * L;
  
  xE = pixelsPerMeter * xE;
  yE = pixelsPerMeter * yE;
  
  //th1 = 3.14 - th1;
  //th2 = 3.14 - th2;
  
  //pGraph.setVertex(1, deviceOrigin.x + lAni*cos(th1), deviceOrigin.y + lAni*sin(th1));
  //pGraph.setVertex(3, deviceOrigin.x + lAni*cos(th2), deviceOrigin.y + lAni*sin(th2));
  //pGraph.setVertex(2, deviceOrigin.x + xE, deviceOrigin.y + yE);
  
  //shape(pGraph);
  //shape(joint);
  
  if (mist == false) {
    shape(wall);
    shape(wall2);
    shape(wall3);
    shape(wall4);
    shape(wall5);
    shape(wall6);
    shape(wall7);
    shape(wall8);
    shape(wall9);
    shape(wall10);
    shape(wall11);
    shape(wall12);
    shape(wall13);
    shape(wall14);
    shape(wall15);
    shape(wall16);
  }
    translate(xE, yE);
    shape(endEffector);
}

// reset if any key is pressed and the treasure has already been taken out sucessfully
void keyPressed() {
  if (treasure_stolen && mist == false) {
    treasure_stolen = false;
  }
}


PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}



/* end helper functions section ****************************************************************************************/




 
