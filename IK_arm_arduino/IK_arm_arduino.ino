
#include <Servo.h>

// Initialisation
Servo servo_b;
Servo servo_s1;
Servo servo_s2;
Servo servo_s3;
int servo_pins[4] = {8, 9, 10, 11};                       // for {b, s1, s2, s3}
float servo_thetas[4] = {PI/2.0, PI/2.0, PI/2.0, PI/2.0}; // for {b, s1, s2, s3}

float t_pos[3] = {0.0, 0.0, 0.0};   // Target position [x,y,z] for arm head, relative to base
float connection_length = 1.0;      //## CHANGE THIS TO A SCALE BASED ON REALTIVE ARM LENGTHS, NO ARBITRARY UNITS OF 50 USED FOR SIMPLICITY IN PROCESSING
float solution_mode = 1.0;        // Which of the two solutions given by IK to use; results in oppose orientation for angles when switched between pm1.0

//(0) Move servos to equilbrium position at PI/2.0
//(1) Have servos follow target
int calibration_mode = 0;

void setup() {
  servo_b.attach(servo_pins[0]);
  servo_s1.attach(servo_pins[1]);
  servo_s2.attach(servo_pins[2]);
  servo_s3.attach(servo_pins[3]);
}

void loop() {
  if(calibration_mode==0) {
    // Set all servos to their equilibrium angle (PI/2.0)
    servo_b.write(180);
    servo_s1.write(180);
    servo_s2.write(180);
    servo_s3.write(180);
    delay(2000);

  } else {
    // Update target position
    t_pos[0] = 0.0;
    t_pos[1] = 0.0;
    t_pos[2] = 0.0;

    // Calculate servo theta values required (IK)
    calculate_servo_theta_constrained(&t_pos[3]);

    // Move servos according to thetas required
    servo_b.write(PI/2.0);
    servo_s1.write(PI/2.0);
    servo_s2.write(PI/2.0);
    servo_s3.write(PI/2.0);
    delay(500);
  }
}

void calculate_servo_theta_constrained(float* target) {
    /*
    . Calculates the servo_thetas[0], servo_thetas[1], servo_thetas[2] and servo_thetas[3] angles required to have the bit head at the target location
    . Found by solving simulataneous eqs for bit end position for servo_thetas[1], servo_thetas[2]

    NOTE; When using this in the real arm, equilibrium angles are offset by PI/2.0, hence use tis function as normal (with normal target position) 
        and offset all found theta values by PI/2.0 (so constraints don't have to be reworked too)
    */
    // Constraints
    //NOTE; Assumes 0.0 is the rest angle, but is actually PI/2.0 for the real servos
    float theta_lower = -PI/2.0;    // Lowest value servo is able to turn to (true for every servo)
    float theta_upper = PI/2.0;     // Highest value servo is able to turn to (true for every servo)

    // Base
    servo_thetas[0] = atan2(target[1], target[0]);

    // S1 & S2
    float r_b = sqrt(pow(target[0],2) +pow(target[1],2));
    if(target[0] < 0.0) {
        r_b *= -1.0;}   // To match axes it is supposed to represent => allows arm to work on +ve and -ve 'r' axis
    float z_b = target[2];
    float t1 = atan2(r_b,z_b);
    float t2 = solution_mode*acos(sqrt(pow(r_b,2) +pow(z_b,2)) / (2.0*connection_length));
    servo_thetas[2] = 2.0*t2;
    servo_thetas[1] = t1-(servo_thetas[2]/2.0);

    // S3
    servo_thetas[3] = (0.0)-servo_thetas[1]-servo_thetas[2];

    // Constrain servos
    servo_thetas[0]  = constrain_servo_angle(servo_thetas[0] , theta_lower, theta_upper);
    servo_thetas[1] = constrain_servo_angle(servo_thetas[1], theta_lower, theta_upper);
    servo_thetas[2] = constrain_servo_angle(servo_thetas[2], theta_lower, theta_upper);
    servo_thetas[3] = constrain_servo_angle(servo_thetas[3], theta_lower, theta_upper);
}
float constrain_servo_angle(float theta_orig, float theta_lower, float theta_upper) {
    float theta = theta_orig;
    if(theta_orig < theta_lower) {
        theta = theta_lower;}
    if(theta_orig > theta_upper) {
        theta = theta_upper;}
    return theta;
}
float rad_to_deg(float rad) {
  return rad*360.0/(2.0*PI);
}
float def_to_rad(float deg) {
  return deg*(2.0*PI)/360.0;
}