
#include <Servo.h>

// Initialisation
Servo servo_b;
Servo servo_s1;
Servo servo_s2;
Servo servo_s3;
int navigation_pins[4] = {2,3,4,5};   // Respectively are; {..., +Y, ..., -X} / {UP, DOWN, RIGHT, LEFT}
int servo_pins[4] = {8, 9, 10, 11};                       // for {b, s1, s2, s3}
float servo_thetas[4] = {PI/2.0, PI/2.0, PI/2.0, PI/2.0}; // for {b, s1, s2, s3}

float t_pos[3] = {0.0, 0.0, 0.0};   // Target position [x,y,z] for arm head, relative to base
float connection_length = 1.0;      //## CHANGE THIS TO A SCALE BASED ON REALTIVE ARM LENGTHS, NO ARBITRARY UNITS OF 50 USED FOR SIMPLICITY IN PROCESSING
float solution_mode = 1.0;        // Which of the two solutions given by IK to use; results in oppose orientation for angles when switched between pm1.0

//(0) Move servos to equilbrium position at PI/2.0
//(1) Have servos follow target
int calibration_mode = 0;

void setup() {
  Serial.begin(9600);
  servo_b.attach(servo_pins[0]);
  servo_s1.attach(servo_pins[1]);
  servo_s2.attach(servo_pins[2]);
  servo_s3.attach(servo_pins[3]);

  pinMode(navigation_pins[0],INPUT_PULLUP);
  pinMode(navigation_pins[1],INPUT_PULLUP);
  pinMode(navigation_pins[2],INPUT_PULLUP);
  pinMode(navigation_pins[3],INPUT_PULLUP);
}

void loop() {
  if(calibration_mode==0) {
    // Print navigation readings
    Serial.print("nav 0, AT ");
    Serial.print(navigation_pins[0]);Serial.print("= ");Serial.println(digitalRead(navigation_pins[0]));
    Serial.print("nav 1, AT ");
    Serial.print(navigation_pins[1]);Serial.print("= ");Serial.println(digitalRead(navigation_pins[1]));
    Serial.print("nav 2, AT ");
    Serial.print(navigation_pins[2]);Serial.print("= ");Serial.println(digitalRead(navigation_pins[2]));
    Serial.print("nav 3, AT ");
    Serial.print(navigation_pins[3]);Serial.print("= ");Serial.println(digitalRead(navigation_pins[3]));
    
    // Set all servos to their equilibrium angle (PI/2.0)
    Serial.println("Calibrating...");
    servo_b.write(180);
    servo_s1.write(180);
    servo_s2.write(180);
    servo_s3.write(180);
    delay(2000);

  } else {
    Serial.println("Main Program...");
    // Update target position
    t_pos[0] = 0.0;
    t_pos[1] = 0.0;
    t_pos[2] = 1.8;
    int UP=digitalRead(navigation_pins[1]);
    int LEFT=digitalRead(navigation_pins[3]);
    if(UP==0) {
      t_pos[1] += 0.1;}
    if(LEFT==0) {
      t_pos[0] -= 0.1;}

    // Calculate servo theta values required (IK)
    calculate_servo_theta_constrained();
    servo_thetas[0] = rad_to_deg(servo_thetas[0] +PI/2.0);
    servo_thetas[1] = rad_to_deg(servo_thetas[1] +PI/2.0);
    servo_thetas[2] = rad_to_deg(servo_thetas[2] +PI/2.0);
    servo_thetas[3] = rad_to_deg(servo_thetas[3] +PI/2.0);

    // Move servos according to thetas required
    Serial.println("===");
    Serial.print("servo_thetas[0] = ");
    Serial.println(servo_thetas[0]);
    Serial.print("servo_thetas[1] = ");
    Serial.println(servo_thetas[1]);
    Serial.print("servo_thetas[2] = ");
    Serial.println(servo_thetas[2]);
    Serial.print("servo_thetas[3] = ");
    Serial.println(servo_thetas[3]);
    servo_b.write(servo_thetas[0]);
    servo_s1.write(servo_thetas[1]);
    servo_s2.write(servo_thetas[2]);
    servo_s3.write(servo_thetas[3]);
    delay(2000);
  }
}

void calculate_servo_theta_constrained() {
    /*
    . Calculates the servo_thetas[0], servo_thetas[1], servo_thetas[2] and servo_thetas[3] angles required to have the bit head at the target location
    . Found by solving simulataneous eqs for bit end position for servo_thetas[1], servo_thetas[2]

    NOTE; When using this in the real arm, equilibrium angles are offset by PI/2.0, hence use tis function as normal (with normal target position) 
        and offset all found theta values by PI/2.0 (so constraints don't have to be reworked too)
    */
    float target[3] = {t_pos[0], t_pos[1], t_pos[2]};
    
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

    Serial.print("target[0] = ");
    Serial.println(target[0]);
    Serial.print("target[1] = ");
    Serial.println(target[1]);
    Serial.print("target[2] = ");
    Serial.println(target[2]);
    Serial.print("r_b = ");
    Serial.println(r_b);
    Serial.print("z_b = ");
    Serial.println(z_b);
    Serial.print("t1 = ");
    Serial.println(t1);
    Serial.print("t2 = ");
    Serial.println(t2);

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
