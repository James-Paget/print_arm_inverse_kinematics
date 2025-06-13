
#include <Servo.h>

// Initialisation
Servo servo_b;
Servo servo_s1;
Servo servo_s2;
Servo servo_s3;
int navigation_pins[4] = {2,3,4,5};   // Respectively are; {..., +Y, ..., -X} / {UP, DOWN, RIGHT, LEFT}
int servo_pins[4] = {8, 9, 10, 11};                       // for {b, s1, s2, s3}
float servo_thetas[4] = {PI/2.0, PI/2.0, PI/2.0, PI/2.0}; // for {b, s1, s2, s3}
//** Note; Issues with pin 8 for fixed servos; intermittent disconnection

float t_pos[3] = {0.0, 0.0, 0.0};   // Target position [x,y,z] for arm head, relative to base
float connection_length = 1.0;
float solution_mode = -1.0;         // Set to pm1.0, <solution required is -1.0>. Which of the two solutions given by IK to use; results in oppose orientation for angles when switched between pm1.0

float u = 0.0;      // Parameterisation variable init. in [0,1]

// -- MODES --
// (0) Move servos to equilbrium position at PI/2.0 [CALIBRATION]
// (1) Follow target moving along a single axis only [SERVO_TEST_1]
// (2) Follow target moving around a circle [SERVO_TEST_2]
// (3) Follow target moving randomly about a spherical volume [SERVO_TEST_3]

int calibration_mode = 0;
int axis_select = 0;      // [0,1,2]==[X,Y,Z], for mode 1,2
float axis_offset = 1.3;  // Number of arm lengths to offset the target by in the axis of choice, used for mode 2
int servoDelayTimeMs = 20;    // Delay time for each servo step in milliseconds
float paramTickRate  = 0.005;  // Increment amount of parameterisation variable each servo tick (above)

// --> For spherical random motion only
float spherePush = 1.0;
float spherePushRate = 0.0075;
float sphereDrag = 0.02;
float sphereMinHeight = 1.42*connection_length;
float sphereMaxHeight = 2.0*connection_length;
float sphereRadius = (sphereMaxHeight-sphereMinHeight)/2.0;
float sphereOrigin[3] = {0.0, 0.0, sphereMinHeight+sphereRadius};
float targetPos[3]    = {sphereOrigin[0]+0.01, sphereOrigin[1], sphereOrigin[2]}; //**Note; +delta added to soften exact alignment
float targetVel[3]    = {0.0, 0.0, 0.0};
// <-- For spherical random motion only


void setup() {
  // Required setup
  Serial.begin(9600);
  servo_b.attach(servo_pins[0]);
  servo_s1.attach(servo_pins[1]);
  servo_s2.attach(servo_pins[2]);
  servo_s3.attach(servo_pins[3]);

  pinMode(navigation_pins[0],INPUT_PULLUP);
  pinMode(navigation_pins[1],INPUT_PULLUP);
  pinMode(navigation_pins[2],INPUT_PULLUP);
  pinMode(navigation_pins[3],INPUT_PULLUP);

  // Mode specific setup
  if(calibration_mode==0) {
    Serial.println("Setup Cal.Mode 0");
    servo_b.write(90);
    servo_s1.write(90);
    servo_s2.write(90);
    servo_s3.write(90);
    
  } else if(calibration_mode==1) {
    Serial.println("Setup Test.Mode Axial");
    // Parameterisation variables
    u = 0.0;
    
    // Set target to be at some point, then move all servos to be there
    t_pos[0] = 0.0;
    t_pos[1] = 0.0;
    t_pos[2] = 0.0;
    if( (0 <= axis_select) || (axis_select < 3) ) { t_pos[axis_select] = 1.2+0.5*sin(u*2.0*PI); } // Relative arm lengths, hence is max 2.0 for this 2 arm system

    // Calculate initial servo positions for this initial setup
    calculations();
    // Adjust calcualted angles to be in terms of servo angle system
    convertThetaRange();
    // Move servos to required locations
    writeServoValues();
    
  } else if(calibration_mode==2) {
    Serial.println("Setup Test.Mode Circular");
    // Parameterisation variables
    u = 0.0;
    
    // Set target orbit about the axis given
    boolean triggerInit = false;
    for(int i=0; i<3; i++) {
      if(i == axis_select) {
        t_pos[i] = axis_offset;
      } else {
        if(!triggerInit) {t_pos[i] = axis_offset*0.3*cos(u*2.0*PI); triggerInit=true;}
        else {            t_pos[i] = axis_offset*0.3*sin(u*2.0*PI);}
      }
    }

    // Calculate initial servo positions for this initial setup
    calculations();
    // Adjust calcualted angles to be in terms of servo angle system
    convertThetaRange();
    // Move servos to required locations
    writeServoValues();
    
  } else if(calibration_mode==3) {
    Serial.println("Setup Test.Mode RandomSphere");
    
    // Set target orbit about the axis given
    setRandomSphericalTarget();

    // Calculate initial servo positions for this initial setup
    calculations();
    // Adjust calcualted angles to be in terms of servo angle system
    convertThetaRange();
    // Move servos to required locations
    writeServoValues();
  }
  
}
void loop() {
  if(calibration_mode==0) {
    // Print navigation readings
    Serial.print("nav 0, AT ");Serial.print(navigation_pins[0]);Serial.print("= ");Serial.println(digitalRead(navigation_pins[0]));
    Serial.print("nav 1, AT ");Serial.print(navigation_pins[1]);Serial.print("= ");Serial.println(digitalRead(navigation_pins[1]));
    Serial.print("nav 2, AT ");Serial.print(navigation_pins[2]);Serial.print("= ");Serial.println(digitalRead(navigation_pins[2]));
    Serial.print("nav 3, AT ");Serial.print(navigation_pins[3]);Serial.print("= ");Serial.println(digitalRead(navigation_pins[3]));
    
    // Set all servos to their equilibrium angle (PI/2.0)
    Serial.println("Calibrating To Zero...");
    servo_b.write(90);
    servo_s1.write(90);
    servo_s2.write(90);
    servo_s3.write(90);
    delay(2000);

  } else if(calibration_mode==1) {
//    Serial.println("Loop Test.Mode Axial");
    // Update parametisation variable
    u = fmod(u+paramTickRate, 1.0);
    
    // Set target location
    if( (0 <= axis_select) || (axis_select < 3) ) { t_pos[axis_select] = 1.2+0.5*sin(u*2.0*PI); }

    calculations();
    convertThetaRange();
    writeServoValues();
    delay(servoDelayTimeMs);
    
  } else if(calibration_mode==2) {
//    Serial.println("Loop Test.Mode Circular");
    // Update parametisation variable
    u = fmod(u+paramTickRate, 1.0);
    
    // Set target location
    boolean triggerInit = false;
    for(int i=0; i<3; i++) {
      if(i == axis_select) {
        t_pos[i] = axis_offset;
      } else {
        if(!triggerInit) {t_pos[i] = axis_offset*0.3*cos(u*2.0*PI); triggerInit=true;}
        else {            t_pos[i] = axis_offset*0.3*sin(u*2.0*PI);}
      }
    }

    calculations();
    convertThetaRange();
    writeServoValues();
    delay(servoDelayTimeMs);
    
  } else if(calibration_mode==3) {
//    Serial.println("Loop Test.Mode RandomSphere");
    
    // Set target location
    setRandomSphericalTarget();

    calculations();
    convertThetaRange();
    writeServoValues();
    delay(servoDelayTimeMs);
    
  }

}


void setRandomSphericalTarget() {
    // -- SPHERICAL RANDOM MOTION --
    float targetAcc[3] = {-sphereDrag*targetVel[0], -sphereDrag*targetVel[1], -sphereDrag*targetVel[2]};
    // Probaility to be given a push
    if(random(0.0, 1.0) < spherePushRate) {
        float sphereTheta = random(0.0, 1.0*PI);
        float spherePhi   = random(0.0, 2.0*PI);
        targetAcc[0] += spherePush*sin(sphereTheta)*cos(spherePhi);
        targetAcc[1] += spherePush*sin(sphereTheta)*sin(spherePhi);
        targetAcc[2] += spherePush*cos(sphereTheta);
    }
    // Update kinematics of target position
    targetVel[0] += targetAcc[0];
    targetVel[1] += targetAcc[1];
    targetVel[2] += targetAcc[2];
    targetPos[0] += targetVel[0];
    targetPos[1] += targetVel[1];
    targetPos[2] += targetVel[2];
    // 'Bounce' the target back into the spherical area wanted
    float dist = sqrt( pow(sphereOrigin[0]-targetPos[0],2) + pow(sphereOrigin[1]-targetPos[1],2) +pow(sphereOrigin[2]-targetPos[2],2) );
    if(dist > sphereRadius) {
        float unitVec[3] = {(targetPos[0]-sphereOrigin[0])/dist, (targetPos[1]-sphereOrigin[1])/dist, (targetPos[2]-sphereOrigin[2])/dist};
        // Move target back into sphere
        targetPos[0] += -2.0*(dist-sphereRadius)*unitVec[0];
        targetPos[1] += -2.0*(dist-sphereRadius)*unitVec[1];
        targetPos[2] += -2.0*(dist-sphereRadius)*unitVec[2];
        // Reverse velocity vector of target
        targetVel[0] = -targetVel[0];
        targetVel[1] = -targetVel[1];
        targetVel[2] = -targetVel[2];
    }
    
    t_pos[0] = targetPos[0];
    t_pos[1] = targetPos[1];
    t_pos[2] = targetPos[2];
}

void convertThetaRange() {
    // Converts angles form [-PI/2, PI/2] format to [0, PI] format; used in calculations to used by servo library
    servo_thetas[0] += PI/2.0;
    servo_thetas[1] += PI/2.0;
    servo_thetas[2] += PI/2.0;
    servo_thetas[3] += PI/2.0;
}
void writeServoValues() {
    servo_b.write(  rad_to_deg(servo_thetas[0]) );
    servo_s1.write( rad_to_deg(servo_thetas[1]) );
    servo_s2.write( rad_to_deg(servo_thetas[2]) );
    servo_s3.write( rad_to_deg(servo_thetas[3]) );
}


void calculations() {
    boolean isConstrained = true; // Whether to apply the [-PI/2, PI/2] constraint to calcualtions (required for real setup)
    float theta_lower = -PI/2.0;  // True for all servos used here
    float theta_upper =  PI/2.0;
    // UPDATED KINEMATICS
    calculate_servo_theta_b( theta_lower, theta_upper, isConstrained);
    calculate_servo_theta_s1(theta_lower, theta_upper, isConstrained);
    calculate_servo_theta_s2(theta_lower, theta_upper, isConstrained);
    calculate_servo_theta_s3(theta_lower, theta_upper, isConstrained);
}

void calculate_servo_theta_b(float theta_lower, float theta_upper, boolean constrained) {
    float v1 = (PI/2.0)*t_pos[1]/abs(t_pos[1]);                     // Account for singularities
    if(t_pos[0] != 0.0) { v1 = atan(t_pos[1] / t_pos[0]); }         // Otherwise do regular arctan
    servo_thetas[0] = v1;
    if(constrained) { servo_thetas[0]  = constrain_servo_angle(servo_thetas[0] , theta_lower, theta_upper); }
}

void calculate_servo_theta_s1(float theta_lower, float theta_upper, boolean constrained) {
    float r_b = sqrt(pow(t_pos[0],2) +pow(t_pos[1],2));
    // if(target.x < 0.0) {
    //     r_b *= -1.0;}   // To match axes it is supposed to represent => allows arm to work on +ve and -ve 'r' axis
    float z_b = t_pos[2];
    float t1 = atan2(r_b,z_b);
    float f1 = min(1.0, sqrt(pow(r_b,2) +pow(z_b,2)) / (2.0*connection_length));
    if( (-1.0 <= f1) && (f1 <= 1.0) ) {
        float t2 = solution_mode*acos(f1);
        servo_thetas[1] = t1-t2;
        if(constrained) { servo_thetas[1] = constrain_servo_angle(servo_thetas[1], theta_lower, theta_upper); }
        if(t_pos[0] < 0.0) {servo_thetas[1] = -servo_thetas[1];}  // Flip subsequent servos (after 0th base) if outside [-PI/2, PI/2] range, due to base servo being limited to this range => flip to cover other half
    }
}
void calculate_servo_theta_s2(float theta_lower, float theta_upper, boolean constrained) {
    float r_b = sqrt(pow(t_pos[0],2) +pow(t_pos[1],2));
    // if(target.x < 0.0) {
    //     r_b *= -1.0;}   // To match axes it is supposed to represent => allows arm to work on +ve and -ve 'r' axis
    float z_b = t_pos[2];
    float f1 = min(1.0, sqrt(pow(r_b,2) +pow(z_b,2)) / (2.0*connection_length));
    if( (-1.0 <= f1) && (f1 <= 1.0) ) {
        float t2 = solution_mode*acos(f1);
        servo_thetas[2] = 2.0*t2;
        if(constrained) { servo_thetas[2] = constrain_servo_angle(servo_thetas[2], theta_lower, theta_upper); }
        if(t_pos[0] < 0.0) {servo_thetas[2] = -servo_thetas[2];}  // Flip subsequent servos (after 0th base) if outside [-PI/2, PI/2] range, due to base servo being limited to this range => flip to cover other half
    }
}
void calculate_servo_theta_s3(float theta_lower, float theta_upper, boolean constrained) {
  // /-- ALTERNATE CALCUALTION FOR SERVO 3 --
    // float r_b = sqrt(pow(t_pos[0],2) +pow(target.y,2));
    // // if(target.x < 0.0) {
    // //     r_b *= -1.0;}   // To match axes it is supposed to represent => allows arm to work on +ve and -ve 'r' axis
    // float z_b = target.z;
    // float t1 = atan2(r_b,z_b);
    // float f1 = min(1.0, sqrt(pow(r_b,2) +pow(z_b,2)) / (2.0*connection_length));
    // float t2 = solutionMode*acos(f1);
    // if( (t1!=Float.NaN) && (t2!=Float.NaN) ) {
    //     theta_s3 = -t1-t2;
    //     if(constrained) { theta_s3 = constrain_servo_angle(theta_s3, theta_lower, theta_upper); }
    // }
    // \-- ALTERNATE CALCUALTION FOR SERVO 3 --
    servo_thetas[3] = -(servo_thetas[1]+servo_thetas[2]);
    if(constrained) { servo_thetas[3] = constrain_servo_angle(servo_thetas[3], theta_lower, theta_upper); }
    // Negative adjustment not required since already accoutned for in prior servos (**if calculated in order)
}
float constrain_servo_angle(float theta_orig, float theta_lower, float theta_upper) {
    float theta = theta_orig;   // Theta should be between 0 and PI here
    if(theta_orig < theta_lower) {
        theta = theta_lower;}
    if(theta_orig > theta_upper) {
        theta = theta_upper;}
    return theta;               // Should return a theta between limits given here
}


float rad_to_deg(float rad) {
  return rad*360.0/(2.0*PI);
}
float deg_to_rad(float deg) {
  return deg*(2.0*PI)/360.0;
}









// ### LEGACY ###
//void calculate_servo_theta_constrained() {
//    /*
//    . Calculates the servo_thetas[0], servo_thetas[1], servo_thetas[2] and servo_thetas[3] angles required to have the bit head at the target location
//    . Found by solving simulataneous eqs for bit end position for servo_thetas[1], servo_thetas[2]
//
//    NOTE; When using this in the real arm, equilibrium angles are offset by PI/2.0, hence use tis function as normal (with normal target position) 
//        and offset all found theta values by PI/2.0 (so constraints don't have to be reworked too)
//    */
//    float target[3] = {t_pos[0], t_pos[1], t_pos[2]};
//    
//    // Constraints
//    //NOTE; Assumes 0.0 is the rest angle, but is actually PI/2.0 for the real servos
//    float theta_lower = -PI/2.0;    // Lowest value servo is able to turn to (true for every servo)
//    float theta_upper = PI/2.0;     // Highest value servo is able to turn to (true for every servo)
//
//    // Base
//    float flipCorrection = 1.0;              // Flip all but 0th servo to simulate the other pi/2.0->pi and -pi->-pi/2.0
//    if(target[0]<0.0) {flipCorrection=-1.0;} //
//    float b1 = atan(target[1]/target[0]);
////    if( (target[0]<0) && (target[1]>0) ) { b1 += PI; }        // NOT required for (-pi/2, pi/2) servos
////    else if( (target[0]<0) && (target[1]<0) ) { b1 -= PI; }
//    servo_thetas[0] = b1;
//
//    // S1 & S2
//    float r_b = sqrt(pow(target[0],2) +pow(target[1],2));
//    if(target[0] < 0.0) {
//        r_b *= -1.0;}   // To match axes it is supposed to represent => allows arm to work on +ve and -ve 'r' axis
//    float z_b = target[2];
//    float t1 = atan2(r_b,z_b);
//    float t2 = solution_mode*acos(sqrt(pow(r_b,2) +pow(z_b,2)) / (2.0*connection_length));
//    servo_thetas[2] = 2.0*t2;
//    servo_thetas[1] = t1-(servo_thetas[2]/2.0);
//
//    // S3
//    servo_thetas[3] = (0.0)-servo_thetas[1]-servo_thetas[2];
//
//    // Constrain servos
////    servo_thetas[0]  = constrain_servo_angle(servo_thetas[0] , theta_lower, theta_upper);
//    servo_thetas[1] = flipCorrection*constrain_servo_angle(servo_thetas[1], theta_lower, theta_upper);
//    servo_thetas[2] = flipCorrection*constrain_servo_angle(servo_thetas[2], theta_lower, theta_upper);
//    servo_thetas[3] = flipCorrection*constrain_servo_angle(servo_thetas[3], theta_lower, theta_upper);
//}
