// Servos vertically up for PI/2
float theta_b  = 0.0;   //PI/2.0;    // Base servo
float theta_s1 = 0.0;   //PI/2.0;    // Lower knuckle servo
float theta_s2 = 0.0;   //PI/2.0;    // Upper knuckle servo
float theta_s3 = 0.0;   //PI/2.0;    // Bit head servo
float connection_length = 100.0; // Length of each arm
float targetOffset = 0.0;

// Camera controls
float panRadius = 600.0;
float cameraTheta = 0.0;
float cameraPhi   = 0.0;

PVector target=new PVector(0.0, 0.0, 0.0);
float servo_display_radius = 10.0;
float solutionMode = -1.0;   //-1.0  // Two solutions found for motion in each case, this allows either to be set

boolean toggle_free = false;
boolean mouseSelect = true;
boolean panSelect   = true;

// --> For spherical random motion only
float spherePush = 1.0;
float spherePushRate = 0.0075;
float sphereDrag = 0.02;
float sphereMinHeight = 1.42*connection_length;
float sphereMaxHeight = 2.0*connection_length;
float sphereRadius = (sphereMaxHeight-sphereMinHeight)/2.0;
PVector sphereOrigin = new PVector(0.0, 0.0, sphereMinHeight+sphereRadius);
PVector targetPos = new PVector(sphereOrigin.x+0.01, sphereOrigin.y, sphereOrigin.z);
PVector targetVel = new PVector(0.0, 0.0, 0.0);
// <-- For spherical random motion only

void setup() {
    size(800, 800, P3D);
    // origin = new PVector(width/2.0, height*0.8, 0.0);   // Purely visual offset
}

void draw() {
    background(50,50,50);
    camera_controls_P3D(panSelect);
    display();
    calculations();    

    // DISPLAY SERVO VALUES
    // println("---");
    // println("Theta0="+str(theta_b));
    // println("Theta1="+str(theta_s1));
    // println("Theta2="+str(theta_s2));
    // println("Theta3="+str(theta_s3));
}

void keyPressed() {
    if(key=='1') {
        solutionMode *= -1.0;               // Choose alternate solution to reach same target point
    }
    if(key=='2') {
        toggle_free = !toggle_free;         // ## INACTIVE ##
    }
    if(key=='3') {
        mouseSelect = !mouseSelect;         // Fix target to mouse or to alternate pattern given (see setTarget() function)
    }
    if(key=='4') {
        panSelect = !panSelect;             // Fix camera view
    }
    if(key=='5') {
        targetOffset -= connection_length*0.1;  // Move target in Y axis
    }
    if(key=='6') {
        targetOffset += connection_length*0.1;  // "" ""
    }
}

void display() {
    // P3D Display
    display_arm_full_P3D();
}

void calculations() {
    boolean isConstrained = true;
    float theta_lower = -PI/2.0;
    float theta_upper =  PI/2.0;
    // UPDATED KINEMATICS
    target = setTarget(mouseSelect);    // Position relative to base of arm, assuming XY view plane is real XZ plane to move in
    // calculate_servo_theta_b(target, 0.0, 2.0*PI, isConstrained);theta_lower, theta_upper
    calculate_servo_theta_b(target,  theta_lower, theta_upper, isConstrained);
    calculate_servo_theta_s1(target, theta_lower, theta_upper, isConstrained);
    calculate_servo_theta_s2(target, theta_lower, theta_upper, isConstrained);
    calculate_servo_theta_s3(target, theta_lower, theta_upper, isConstrained);

    // setSolutionMode(target);     // Prevents impossible angles for either side -> But will cause a jump at the switch
}

void camera_controls_P3D(boolean active) {
    if(active) {
        cameraTheta = -1.0*PI*(mouseY/float(height))*3.0;
        cameraPhi   =  2.0*PI*(mouseX/float(width)) *3.0;
        PVector cameraPos = new PVector(
            panRadius*cos(cameraPhi)*sin(cameraTheta),
            panRadius*sin(cameraPhi)*sin(cameraTheta),
            panRadius*cos(cameraTheta)
        );
        camera(     // [eyeX, eyeY, eyeZ,   centerX, centerY, centerZ,  upX, upY, upZ]
            cameraPos.x, cameraPos.y, cameraPos.z,
            0.0, 0.0, 0.0,
            0.0, 0.0, 1.0
        );
    }
}

void convert_servo_nonnegative_base(boolean useRadians) {
    /*
    . Converts the theta from a [-90 -> 90, 0 centre] degree scale to a [0 -> 180, 90 centre] degree scale
    . This can be done in either degrees or radians
    */
    float scaleFactor = 90.0;
    if(useRadians) { 
        scaleFactor = PI/4.0;
    }
    theta_b  += scaleFactor;
    theta_s1 += scaleFactor;
    theta_s2 += scaleFactor;
    theta_s3 += scaleFactor;
}

PVector setTarget(boolean mouseMode) {
    if(mouseMode) {
        return new PVector(width/2.0-mouseX, targetOffset, -(height/2.0-mouseY));   // Fix to mouse
    } else {
        // -- CIRCULAR MOTION --
        // float theta = (2.0*PI*frameCount/60.0) /5.0;
        // float radius = connection_length*1.4;
        // return new PVector(radius*cos(theta), targetOffset, radius*sin(theta));

        // -- SPHERICAL RANDOM MOTION --
        PVector targetAcc = new PVector(-sphereDrag*targetVel.x, -sphereDrag*targetVel.y, -sphereDrag*targetVel.z);
        // Probaility to be given a push
        if(random(0.0, 1.0) < spherePushRate) {
            float sphereTheta = random(0.0, 1.0*PI);
            float spherePhi   = random(0.0, 2.0*PI);
            targetAcc.x += spherePush*sin(sphereTheta)*cos(spherePhi);
            targetAcc.y += spherePush*sin(sphereTheta)*sin(spherePhi);
            targetAcc.z += spherePush*cos(sphereTheta);
        }
        // Update kinematics of target position
        targetVel.x += targetAcc.x;
        targetVel.y += targetAcc.y;
        targetVel.z += targetAcc.z;
        targetPos.x += targetVel.x;
        targetPos.y += targetVel.y;
        targetPos.z += targetVel.z;
        // 'Bounce' the target back into the spherical area wanted
        float dist = sqrt( pow(sphereOrigin.x-targetPos.x,2) + pow(sphereOrigin.y-targetPos.y,2) +pow(sphereOrigin.z-targetPos.z,2) );
        if(dist > sphereRadius) {
            PVector unitVec = new PVector((targetPos.x-sphereOrigin.x)/dist, (targetPos.y-sphereOrigin.y)/dist, (targetPos.z-sphereOrigin.z)/dist);
            // Move target back into sphere
            targetPos.x += -2.0*(dist-sphereRadius)*unitVec.x;
            targetPos.y += -2.0*(dist-sphereRadius)*unitVec.y;
            targetPos.z += -2.0*(dist-sphereRadius)*unitVec.z;
            // Reverse velocity vector of target
            targetVel.x = -targetVel.x;
            targetVel.y = -targetVel.y;
            targetVel.z = -targetVel.z;
        }

        return targetPos;
    }
}

void calculate_servo_theta_b(PVector target, float theta_lower, float theta_upper, boolean constrained) {
    float v1 = (PI/2.0)*target.y/abs(target.y);                     // Account for singularities
    if(target.x != 0.0) { v1 = atan(target.y / target.x); }         // Otherwise do regular arctan
    if(v1!=Float.NaN) {
        theta_b = v1;
        if(constrained) { theta_b  = constrain_servo_angle(theta_b , theta_lower, theta_upper); }
    }
}

void calculate_servo_theta_s1(PVector target, float theta_lower, float theta_upper, boolean constrained) {
    float r_b = sqrt(pow(target.x,2) +pow(target.y,2));
    // if(target.x < 0.0) {
    //     r_b *= -1.0;}   // To match axes it is supposed to represent => allows arm to work on +ve and -ve 'r' axis
    float z_b = target.z;
    float t1 = atan2(r_b,z_b);
    float f1 = min(1.0, sqrt(pow(r_b,2) +pow(z_b,2)) / (2.0*connection_length));
    float t2 = solutionMode*acos(f1);
    if( (t1!=Float.NaN) && (t2!=Float.NaN) ) {
        theta_s1 = t1-t2;
        if(constrained) { theta_s1 = constrain_servo_angle(theta_s1, theta_lower, theta_upper); }
        if(target.x < 0.0) {theta_s1 = -theta_s1;}  // Flip subsequent servos (after 0th base) if outside [-PI/2, PI/2] range, due to base servo being limited to this range => flip to cover other half
    }
}

void calculate_servo_theta_s2(PVector target, float theta_lower, float theta_upper, boolean constrained) {
    float r_b = sqrt(pow(target.x,2) +pow(target.y,2));
    // if(target.x < 0.0) {
    //     r_b *= -1.0;}   // To match axes it is supposed to represent => allows arm to work on +ve and -ve 'r' axis
    float z_b = target.z;
    float f1 = min(1.0, sqrt(pow(r_b,2) +pow(z_b,2)) / (2.0*connection_length));
    float t2 = solutionMode*acos(f1);
    if( t2!=Float.NaN ) {
        theta_s2 = 2.0*t2;
        if(constrained) { theta_s2 = constrain_servo_angle(theta_s2, theta_lower, theta_upper); }
        if(target.x < 0.0) {theta_s2 = -theta_s2;}  // Flip subsequent servos (after 0th base) if outside [-PI/2, PI/2] range, due to base servo being limited to this range => flip to cover other half
    }
}

void calculate_servo_theta_s3(PVector target, float theta_lower, float theta_upper, boolean constrained) {
    // /-- ALTERNATE CALCUALTION FOR SERVO 3 --
    // float r_b = sqrt(pow(target.x,2) +pow(target.y,2));
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
    theta_s3 = -(theta_s1+theta_s2);
    if(constrained) { theta_s3 = constrain_servo_angle(theta_s3, theta_lower, theta_upper); }
    // NOT required to flip here since the prior angles have already had their negative sign accoutned for
}

float constrain_servo_angle(float theta_orig, float theta_lower, float theta_upper) {
    float theta = theta_orig;   // Theta should be between 0 and PI here
    if(theta_orig < theta_lower) {
        theta = theta_lower;}
    if(theta_orig > theta_upper) {
        theta = theta_upper;}
    return theta;               // Should return a theta between limits given here
}

void display_arm_full_P3D() {
    // AXES
    pushStyle();
    pushMatrix();
    float axisLength = connection_length;
    strokeWeight(1.0);
    stroke(255,0,0);
    line(0.0, 0.0, 0.0, axisLength, 0.0, 0.0);
    stroke(0,255,0);
    line(0.0, 0.0, 0.0, 0.0, axisLength, 0.0);
    stroke(0,0,255);
    line(0.0, 0.0, 0.0, 0.0, 0.0, axisLength);
    popMatrix();
    popStyle();


    PVector v1 = new PVector(
        connection_length*cos(theta_b)*sin(theta_s1),
        connection_length*sin(theta_b)*sin(theta_s1),
        connection_length*cos(theta_s1)
    );
    PVector v2 = new PVector(
        connection_length*cos(theta_b)*sin(theta_s1+theta_s2),
        connection_length*sin(theta_b)*sin(theta_s1+theta_s2),
        connection_length*cos(theta_s1+theta_s2)
    );
    PVector v3 = new PVector(
        connection_length*cos(theta_b)*sin(theta_s1+theta_s2+theta_s3),
        connection_length*sin(theta_b)*sin(theta_s1+theta_s2+theta_s3),
        connection_length*cos(theta_s1+theta_s2+theta_s3)
    );

    pushMatrix();
    // SERVO 0
    stroke(0,0,20);strokeWeight(1);
    sphere(servo_display_radius);
    stroke(255,0,0);strokeWeight(10.0);
    line(
        0.0, 0.0, 0.0,
        v1.x, v1.y, v1.z
    );

    // SERVO 1
    translate(v1.x, v1.y, v1.z);
    stroke(0,0,40);strokeWeight(1);
    sphere(servo_display_radius);
    stroke(255,0,0);strokeWeight(10.0);
    line(
        0.0, 0.0, 0.0,
        v2.x, v2.y, v2.z
    );

    // SERVO 2
    translate(v2.x, v2.y, v2.z);
    stroke(0,0,60);strokeWeight(1);
    sphere(servo_display_radius);
    stroke(255,0,0);strokeWeight(10.0);
    line(
        0.0, 0.0, 0.0,
        v3.x, v3.y, v3.z
    );

    // SERVO 3
    // translate(v3.x, v3.y, v3.z);
    // stroke(0,0,80);strokeWeight(1);
    // sphere(servo_display_radius);
    popMatrix();


    // Target point
    pushStyle();
    pushMatrix();
    translate(
        target.x,
        target.y,
        target.z
    );
    stroke(200,100,20);strokeWeight(1);
    sphere(servo_display_radius*0.5);
    popMatrix();
    popStyle();
}



// ### LEGACY ###
// void calculate_servo_theta_free(PVector target) {
//     /*
//     . Calculates the theta_b, theta_s1, theta_s2 and theta_s3 angles required to have the bit head at the target location
//     . Found by solving simulataneous eqs for bit end position for theta_s1, theta_s2
//     */
//     // Base
//     theta_b = atan2(target.y, target.x);

//     // S1 & S2
//     float r_b = sqrt(pow(target.x,2) +pow(target.y,2));
//     if(target.x < 0.0) {
//         r_b *= -1.0;}   // To match axes it is supposed to represent => allows arm to work on +ve and -ve 'r' axis
//     float z_b = target.z;
//     float t1 = atan2(r_b,z_b);
//     float t2 = solutionMode*acos(sqrt(pow(r_b,2) +pow(z_b,2)) / (2.0*connection_length));
//     theta_s2 = 2.0*t2;
//     theta_s1 = t1-(theta_s2/2.0);

//     // S3
//     theta_s3 = (0.0)-theta_s1-theta_s2;
// }


// void calculate_servo_theta_constrained(PVector target) {
//     /*
//     . Calculates the theta_b, theta_s1, theta_s2 and theta_s3 angles required to have the bit head at the target location
//     . Found by solving simulataneous eqs for bit end position for theta_s1, theta_s2

//     NOTE; When using this in the real arm, equilibrium angles are offset by PI/2.0, hence use tis function as normal (with normal target position) 
//         and offset all found theta values by PI/2.0 (so constraints don't have to be reworked too)
//     */
//     // Constraints
//     //NOTE; Assumes 0.0 is the rest angle, but is actually PI/2.0 for the real servos
//     float theta_lower = -PI/2.0;    // Lowest value servo is able to turn to (true for every servo)
//     float theta_upper = PI/2.0;     // Highest value servo is able to turn to (true for every servo)

//     // Base
//     theta_b = atan2(target.y, target.x);

//     // S1 & S2
//     float r_b = sqrt(pow(target.x,2) +pow(target.y,2));
//     if(target.x < 0.0) {
//         r_b *= -1.0;}   // To match axes it is supposed to represent => allows arm to work on +ve and -ve 'r' axis
//     float z_b = target.z;
//     float t1 = atan2(r_b,z_b);
//     float t2 = solutionMode*acos(sqrt(pow(r_b,2) +pow(z_b,2)) / (2.0*connection_length));
//     theta_s2 = 2.0*t2;
//     theta_s1 = t1-(theta_s2/2.0);

//     // S3
//     theta_s3 = (0.0)-theta_s1-theta_s2;

//     // Constrain servos
//     theta_b  = constrain_servo_angle(theta_b , theta_lower, theta_upper);
//     theta_s1 = constrain_servo_angle(theta_s1, theta_lower, theta_upper);
//     theta_s2 = constrain_servo_angle(theta_s2, theta_lower, theta_upper);
//     // theta_s3 = constrain_servo_angle(theta_s3, theta_lower, theta_upper);
// }