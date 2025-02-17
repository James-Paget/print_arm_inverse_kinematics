// Servos vertically up for PI/2
float theta_b  = 0.0;   //PI/2.0;    // Base servo
float theta_s1 = 0.0;   //PI/2.0;    // Lower knuckle servo
float theta_s2 = 0.0;   //PI/2.0;    // Upper knuckle servo
float theta_s3 = 0.0;   //PI/2.0;    // Bit head servo

float connection_length = 50.0; // Length of each arm

PVector origin;
PVector target=new PVector(0.0, 0.0, 0.0);
float servo_display_radius = 10.0;
float solutionMode = 1.0;   //-1.0  // Two solutions found for motion in each case, this allows either to be set

boolean toggle_free = false;

void setup() {
    size(800, 800);
    origin = new PVector(width/2.0, height*0.8, 0.0);   // Purely visual offset
}

void draw() {
    background(100,100,200);
    display_arm_full(origin);

    // UPDATED KINEMATICS
    target = new PVector(mouseX-origin.x, 0.0, -(mouseY-origin.y));    // Position relative to base of arm, assuming XY view plane is real XZ plane to move in
    if(toggle_free) {
        calculate_servo_theta_free(target);
    } else {
        calculate_servo_theta_constrained(target);
    }
}

void keyPressed() {
    if(key=='1') {
        solutionMode *= -1.0;
    }
    if(key=='2') {
        toggle_free = !toggle_free;
    }
}

void calculate_servo_theta_constrained(PVector target) {
    /*
    . Calculates the theta_b, theta_s1, theta_s2 and theta_s3 angles required to have the bit head at the target location
    . Found by solving simulataneous eqs for bit end position for theta_s1, theta_s2

    NOTE; When using this in the real arm, equilibrium angles are offset by PI/2.0, hence use tis function as normal (with normal target position) 
        and offset all found theta values by PI/2.0 (so constraints don't have to be reworked too)
    */
    // Constraints
    //NOTE; Assumes 0.0 is the rest angle, but is actually PI/2.0 for the real servos
    float theta_lower = -PI/2.0;    // Lowest value servo is able to turn to (true for every servo)
    float theta_upper = PI/2.0;     // Highest value servo is able to turn to (true for every servo)

    // Base
    theta_b = atan2(target.y, target.x);

    // S1 & S2
    float r_b = sqrt(pow(target.x,2) +pow(target.y,2));
    if(target.x < 0.0) {
        r_b *= -1.0;}   // To match axes it is supposed to represent => allows arm to work on +ve and -ve 'r' axis
    float z_b = target.z;
    float t1 = atan2(r_b,z_b);
    float t2 = solutionMode*acos(sqrt(pow(r_b,2) +pow(z_b,2)) / (2.0*connection_length));
    theta_s2 = 2.0*t2;
    theta_s1 = t1-(theta_s2/2.0);

    // S3
    theta_s3 = (0.0)-theta_s1-theta_s2;

    // Constrain servos
    theta_b  = constrain_servo_angle(theta_b , theta_lower, theta_upper);
    theta_s1 = constrain_servo_angle(theta_s1, theta_lower, theta_upper);
    theta_s2 = constrain_servo_angle(theta_s2, theta_lower, theta_upper);
    theta_s3 = constrain_servo_angle(theta_s3, theta_lower, theta_upper);
}
float constrain_servo_angle(float theta_orig, float theta_lower, float theta_upper) {
    float theta = theta_orig;
    if(theta_orig < theta_lower) {
        theta = theta_lower;}
    if(theta_orig > theta_upper) {
        theta = theta_upper;}
    return theta;
}

void calculate_servo_theta_free(PVector target) {
    /*
    . Calculates the theta_b, theta_s1, theta_s2 and theta_s3 angles required to have the bit head at the target location
    . Found by solving simulataneous eqs for bit end position for theta_s1, theta_s2
    */
    // Base
    theta_b = atan2(target.y, target.x);

    // S1 & S2
    float r_b = sqrt(pow(target.x,2) +pow(target.y,2));
    if(target.x < 0.0) {
        r_b *= -1.0;}   // To match axes it is supposed to represent => allows arm to work on +ve and -ve 'r' axis
    float z_b = target.z;
    float t1 = atan2(r_b,z_b);
    float t2 = solutionMode*acos(sqrt(pow(r_b,2) +pow(z_b,2)) / (2.0*connection_length));
    theta_s2 = 2.0*t2;
    theta_s1 = t1-(theta_s2/2.0);

    // S3
    theta_s3 = (0.0)-theta_s1-theta_s2;
}

void display_arm_full(PVector origin) {
    pushMatrix();
    //translate(origin.x, origin.y, origin.z);
    
    pushStyle();
    // Target point
    fill(200,200,100);
    stroke(0,0,0);
    strokeWeight(1);
    ellipse(
        origin.x+target.x,
        origin.y-target.z,
        15.0, 15.0
    );
    
    // Each servo
    fill(200,100,100);
    stroke(0,0,0);
    strokeWeight(2);
    ellipse(
        origin.x,
        origin.y,
        servo_display_radius, 
        servo_display_radius
    );
    ellipse(
        origin.x +connection_length*sin(theta_s1),
        origin.y -connection_length*cos(theta_s1),
        servo_display_radius, 
        servo_display_radius
    );
    ellipse(
        origin.x +connection_length*sin(theta_s1) +connection_length*sin(theta_s1+theta_s2),
        origin.y -connection_length*cos(theta_s1) -connection_length*cos(theta_s1+theta_s2),
        servo_display_radius, 
        servo_display_radius
    );
    ellipse(
        origin.x +connection_length*sin(theta_s1) +connection_length*sin(theta_s1+theta_s2) +connection_length*sin(theta_s1+theta_s2+theta_s3),
        origin.y -connection_length*cos(theta_s1) -connection_length*cos(theta_s1+theta_s2) -connection_length*cos(theta_s1+theta_s2+theta_s3),
        servo_display_radius, 
        servo_display_radius
    );
    // Knuckle lower -> knuckle upper
    noFill();
    stroke(200,0,0);
    strokeWeight(5);
    line(
        origin.x,
        origin.y,
        origin.x +connection_length*sin(theta_s1),
        origin.y -connection_length*cos(theta_s1)
    );
    line(
        origin.x +connection_length*sin(theta_s1),
        origin.y -connection_length*cos(theta_s1),
        origin.x +connection_length*sin(theta_s1) +connection_length*sin(theta_s1+theta_s2),
        origin.y -connection_length*cos(theta_s1) -connection_length*cos(theta_s1+theta_s2)
    );
    line(
        origin.x +connection_length*sin(theta_s1) +connection_length*sin(theta_s1+theta_s2),
        origin.y -connection_length*cos(theta_s1) -connection_length*cos(theta_s1+theta_s2),
        origin.x +connection_length*sin(theta_s1) +connection_length*sin(theta_s1+theta_s2) +connection_length*sin(theta_s1+theta_s2+theta_s3),
        origin.y -connection_length*cos(theta_s1) -connection_length*cos(theta_s1+theta_s2) -connection_length*cos(theta_s1+theta_s2+theta_s3)
    );
    popStyle();

    popMatrix();
}