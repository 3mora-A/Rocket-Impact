




//These libraries are used for OpenGL rendering, window management, input handling, and drawing 3D objects.

#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>


// These libraries are used for:
// - Math calculations
// - Random number generation
// - Input and output operations
// - Data containers (vectors)


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <algorithm>
#include <iostream>
#include <vector>

// --- IMAGE LOADING ---

//This library is used to load image files(such as the Earth texture) and convert them into OpenGL textures.
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- CONFIGURATION CONSTANTS ---
// These constants define the main simulation parameters such as
// window size, environment scale, physics values, and control behavior.

#define WINDOW_WIDTH 1200
#define WINDOW_HEIGHT 900
#define EARTH_RADIUS 1000.0f
#define FLIGHT_DURATION 40.0f
#define CRUISE_DURATION 60.0f
#define NUM_STARS 4000
#define MANUAL_THRUST_MAX 200.0f
#define MANUAL_GRAVITY 9.8f
#define TURN_RATE 0.5f
#define DAMPING_FACTOR 10.0f

// GUIDED MODE CONSTANTS
// These constants control the behavior, appearance, and camera settings
// of the rocket during guided flight mode.

#define GUIDED_THRUST 100.0f
#define INITIAL_CAM_DISTANCE EARTH_RADIUS * 1.2f
#define ROCKET_CYLINDER_HEIGHT 4.0f
#define ROCKET_CONE_HEIGHT 1.5f
#define ROCKET_RADIUS 0.5f
#define MARKER_SCALE 0.005f
#define TRACKING_CAM_LAG 0.1f
#define HORIZONTAL_FLIGHT_ALTITUDE 15.0f

// CRUISE MODE CONSTANTS
// These constants define the behavior of the rocket during cruise flight mode.
#define TARGET_CRUISE_ALTITUDE 5.0f
#define CRUISE_FLIGHT_SPEED 20.0f
#define CRUISE_GUIDANCE_FORCE 10.0f

// --- GLOBALS ---
// Global variables used to control the simulation state, rocket motion,
// camera behavior, and rendering data.


enum Mode { MODE_GUIDED, MODE_MANUAL, MODE_CRUISE };
enum StageState { STAGE_ATTACHED, STAGE_SEPARATED };
enum ThrustState { THRUSTING, COASTING };
float rocketDir[3] = { 0.0f, 0.0f, 1.0f };
float boosterDir[3] = { 0.0f, 0.0f, 1.0f };


Mode currentMode;
StageState stageState = STAGE_ATTACHED;
ThrustState thrustState = THRUSTING;

float simTime = 0.0f;
float timeScale = 1.0f;
bool hasExploded = false;
float explosionScale = 0.0f;
float flashTime = 0.0f;
bool inOuterAtmosphere = false;
bool isCruising = false;

// Manual Control Flags
bool isThrusting = false;
bool isWarheadThrusting = false;
bool isAiming = false;

// Guided Mode Camera Flags
bool isRocketTracking = false;

// Coordinates
float launchLat, launchLon, targetLat, targetLon;
float startPos[3], endPos[3];
float totalArcLength = 0.0f;

// Rocket (Active Part/Warhead) State
float rocketPos[3] = { 0.0f, 0.0f, 0.0f };
float rocketVelocity[3] = { 0.0f, 0.0f, 0.0f };
float rocketOrient[3] = { 0.0f, 90.0f, 0.0f };

// Booster (Dropped Part) State
float boosterPos[3] = { 0.0f, 0.0f, 0.0f };
float boosterVelocity[3] = { 0.0f, 0.0f, 0.0f };
float boosterOrient[3] = { 0.0f, 0.0f, 0.0f };

// Global Camera
float camAngleX = 20.0f;
float camAngleY = 45.0f;
float camDist = INITIAL_CAM_DISTANCE;

// Tracking Camera Smoothed Position
float trackingCamTarget[3] = { 0.0f };

int lastMx, lastMy;
bool isMouseOrbiting = false;

// Trajectory for trail drawing
std::vector<float> guidedTrajectory;

// Assets
GLuint earthTextureID = 0;
struct Star { float x, y, z, b; };
std::vector<Star> stars;

// --- PROTOTYPES ---
// Function declarations used for physics, rendering, input handling,
// trajectory computation, and simulation control.

void calculateHorizontalTrajectory(float t, float* out);
void toCartesian(float lat, float lon, float r, float* out);
void slerp(float* p1, float* p2, float t, float* out);
void normalize(float* v);
void crossProduct(float* a, float* b, float* out);
float dotProduct(float* a, float* b);
void getRocketVectors(float* pos, float yaw, float pitch, float* fwd, float* up, float* right);
void updateGuidedPosition(float dt);
void updateCruisePosition(float dt);
void updateBoosterPhysics(float dt);
void updateManualPosition(float dt);
void drawTrajectoryTrail();
void drawMarkers();
void transformAndDraw(float* pos, float* dir, bool drawBoost, bool drawHead);
void drawExplosion();
void drawNuclearFlash();
void drawScene();
void drawControlsHUD();
void drawCylinder(float h, float rBase, float rTop, float r, float g, float b);
void drawRocketParts(bool drawBooster, bool drawWarhead);
void drawString(float x, float y, void* font, const char* string);
void loadTexture(const char* filename, GLuint* textureID);
void init();
void display();
void update(int value);
void keyboard(unsigned char k, int x, int y);
void mouse(int b, int s, int x, int y);
void motion(int x, int y);
void mouseWheel(int wheel, int direction, int x, int y);
void getUserInput();
void reshape(int w, int h);
void calculateTotalArcLength();
void applyPhysics(float* pos, float* velocity, float thrust, float dt, float yaw, float pitch, bool isWarhead);
float calculateInitialYaw(float* start, float* end);
void velocityToOrientation(float* velocity, float* pos, float* yaw, float* pitch);
float distance(float* p1, float* p2);

// ---------------------------------------------------------
// --- MATH HELPERS ---
// ---------------------------------------------------------
  

// Normalizes a vector so its length becomes 1 and it is used only as a direction
void normalize(float* v) {
    float len = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (len > 0.0001f) { v[0] /= len; v[1] /= len; v[2] /= len; }
}

//Calculates the relationship between two directions and is used to find angles or similarity between vectors.
float dotProduct(float* a, float* b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}


//Computes a vector perpendicular to two vectors and is used to determine directions such as right and up.
void crossProduct(float* a, float* b, float* out) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

//Converts latitude and longitude into 3D Cartesian coordinates on the Earth’s surface
void toCartesian(float lat, float lon, float r, float* out) {
    float latRad = lat * M_PI / 180.0f;
    float lonRad = lon * M_PI / 180.0f;
    out[0] = r * cos(latRad) * sin(lonRad);
    out[1] = r * sin(latRad);
    out[2] = r * cos(latRad) * cos(lonRad);
}
//Computes a point between two points on a sphere to ensure realistic spherical motion
void slerp(float* p1, float* p2, float t, float* out) {
    float l1 = sqrt(p1[0] * p1[0] + p1[1] * p1[1] + p1[2] * p1[2]);
    float l2 = sqrt(p2[0] * p2[0] + p2[1] * p2[1] + p2[2] * p2[2]);
    float dot_val = (p1[0] * p2[0] + p1[1] * p2[1] + p1[2] * p2[2]) / (l1 * l2);
    dot_val = std::max(-1.0f, std::min(1.0f, dot_val));
    float theta = acos(dot_val);
    if (theta < 0.001f) { out[0] = p1[0]; out[1] = p1[1]; out[2] = p1[2]; return; }
    float w1 = sin((1.0f - t) * theta) / sin(theta);
    float w2 = sin(t * theta) / sin(theta);
    out[0] = w1 * p1[0] + w2 * p2[0];
    out[1] = w1 * p1[1] + w2 * p2[1];
    out[2] = w1 * p1[2] + w2 * p2[2];
}

//Calculates the distance between two points in 3D space
float distance(float* p1, float* p2) {
    float dx = p1[0] - p2[0];
    float dy = p1[1] - p2[1];
    float dz = p1[2] - p2[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Computes the rocket's forward, up, and right direction vectors.
void getRocketVectors(float* pos, float yaw, float pitch, float* fwd, float* up, float* right) {
    float r = sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]);
    if (r < 0.001f) r = 1.0f;
    up[0] = pos[0] / r; up[1] = pos[1] / r; up[2] = pos[2] / r;
    float worldY[3] = { 0.0f, 1.0f, 0.0f };
    float tempRight[3];
    crossProduct(up, worldY, tempRight);
    normalize(tempRight);
    float north[3];
    crossProduct(tempRight, up, north);
    normalize(north);
    float yawRad = yaw * M_PI / 180.0f;
    float aimedH[3];
    for (int i = 0; i < 3; i++) aimedH[i] = north[i] * cos(yawRad) + tempRight[i] * sin(yawRad);
    normalize(aimedH);
    float pitchRad = pitch * M_PI / 180.0f;
    for (int i = 0; i < 3; i++) fwd[i] = up[i] * sin(pitchRad) + aimedH[i] * cos(pitchRad);
    normalize(fwd);
    crossProduct(fwd, up, right);
    normalize(right);
}

//Converts the rocket’s movement direction into rotation angles for correct rendering.
void velocityToOrientation(float* velocity, float* pos, float* yaw, float* pitch) {
    float velMag = sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1] + velocity[2] * velocity[2]);
    if (velMag < 0.0001f) return;

    float velNorm[3] = { velocity[0] / velMag, velocity[1] / velMag, velocity[2] / velMag };
    float up[3] = { pos[0], pos[1], pos[2] };
    normalize(up);

    // Pitch is angle between up vector and velocity vector (0 is Vertical Up, 90 is Horizontal)
    float pitchRad = acos(dotProduct(up, velNorm));
    *pitch = 90.0f - (pitchRad * 180.0f / M_PI);
    // Note: In manual mode pitch 90 is horizontal. acos returns 0 for up.
    // If vel is Up, acos(1) = 0. Pitch 0.
    // If vel is Down, acos(-1) = 180. Pitch 180.

    // Calculate Yaw
    float worldY[3] = { 0.0f, 1.0f, 0.0f };
    float localEast[3];
    crossProduct(up, worldY, localEast);
    normalize(localEast);

    float localNorth[3];
    crossProduct(localEast, up, localNorth);
    normalize(localNorth);

    float up_proj_vel = dotProduct(velNorm, up);
    float horizontalVel[3];
    horizontalVel[0] = velNorm[0] - up_proj_vel * up[0];
    horizontalVel[1] = velNorm[1] - up_proj_vel * up[1];
    horizontalVel[2] = velNorm[2] - up_proj_vel * up[2];
    normalize(horizontalVel);

    float yawRad = atan2(dotProduct(horizontalVel, localEast), dotProduct(horizontalVel, localNorth));

    *yaw = yawRad * 180.0f / M_PI;
    if (*yaw < 0.0f) *yaw += 360.0f;
}

//Calculates the initial heading direction of the rocket toward the target
float calculateInitialYaw(float* start, float* end) {
    float up[3] = { start[0], start[1], start[2] };
    normalize(up);
    float targetVector[3] = { end[0] - start[0], end[1] - start[1], end[2] - start[2] };
    float up_proj = dotProduct(targetVector, up);
    float horizontalTarget[3] = {
        targetVector[0] - up_proj * up[0],
        targetVector[1] - up_proj * up[1],
        targetVector[2] - up_proj * up[2]
    };
    normalize(horizontalTarget);
    float worldY[3] = { 0.0f, 1.0f, 0.0f };
    float localEast[3];
    crossProduct(up, worldY, localEast);
    normalize(localEast);
    float localNorth[3];
    crossProduct(localEast, up, localNorth);
    normalize(localNorth);
    float yawRad = atan2(dotProduct(horizontalTarget, localEast), dotProduct(horizontalTarget, localNorth));
    float yawDeg = yawRad * 180.0f / M_PI;
    if (yawDeg < 0) yawDeg += 360.0f;
    return yawDeg;
}

//Calculates the rocket’s position in guided mode based on time and a spherical trajectory.
void calculateHorizontalTrajectory(float t, float* out) {
    float groundPos[3];
    // Interpolate position along the Great Circle Arc
    slerp(startPos, endPos, t, groundPos);

    float len = sqrt(groundPos[0] * groundPos[0] + groundPos[1] * groundPos[1] + groundPos[2] * groundPos[2]);
    float dir[3] = { groundPos[0] / len, groundPos[1] / len, groundPos[2] / len };

    // Altitude Profile
    float launchRadius = EARTH_RADIUS + ROCKET_CYLINDER_HEIGHT * 0.5f;
    float cruiseRadius = EARTH_RADIUS + HORIZONTAL_FLIGHT_ALTITUDE;
    float currentR;

    if (t < 0.2f) { // Smooth Ascent
        float p = t / 0.2f;
        currentR = launchRadius + (cruiseRadius - launchRadius) * sin(p * M_PI / 2.0f);
    }
    else if (t > 0.8f) { // Smooth Descent
        float p = (t - 0.8f) / 0.2f;
        currentR = cruiseRadius + (launchRadius - cruiseRadius) * (1.0f - cos(p * M_PI / 2.0f));
    }
    else { // Cruise
        currentR = cruiseRadius;
    }

    out[0] = dir[0] * currentR;
    out[1] = dir[1] * currentR;
    out[2] = dir[2] * currentR;
}

//Calculates the total length of the trajectory between the launch point and the target
void calculateTotalArcLength() {
    totalArcLength = 0.0f;
    const int steps = 100;
    float prevPos[3] = { 0.0f, 0.0f, 0.0f };
    for (int i = 0; i <= steps; ++i) {
        float t = (float)i / steps;
        float currentPos[3];
        calculateHorizontalTrajectory(t, currentPos);
        if (i > 0) {
            totalArcLength += distance(currentPos, prevPos);
        }
        prevPos[0] = currentPos[0]; prevPos[1] = currentPos[1]; prevPos[2] = currentPos[2];
    }
}

// ---------------------------------------------------------
// --- CORE LOGIC (Physics) ---
// ---------------------------------------------------------

void applyPhysics(float* pos, float* velocity, float thrust, float dt, float yaw, float pitch, bool isWarhead) {
    float r = sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]);
    if (r < 0.1f) r = 1.0f;

    float fwd[3], up[3], right[3];
    getRocketVectors(pos, yaw, pitch, fwd, up, right);

    if (thrust > 0.0f) {
        velocity[0] += fwd[0] * thrust * dt;
        velocity[1] += fwd[1] * thrust * dt;
        velocity[2] += fwd[2] * thrust * dt;
    }

    if (r > EARTH_RADIUS + 0.1f) {
        velocity[0] -= up[0] * MANUAL_GRAVITY * dt;
        velocity[1] -= up[1] * MANUAL_GRAVITY * dt;
        velocity[2] -= up[2] * MANUAL_GRAVITY * dt;
    }

    pos[0] += velocity[0] * dt;
    pos[1] += velocity[1] * dt;
    pos[2] += velocity[2] * dt;

    float newR = sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]);

    // Collision Check
    float minR = EARTH_RADIUS + 0.1f;
    if (newR < minR) {
        pos[0] = pos[0] / newR * minR;
        pos[1] = pos[1] / newR * minR;
        pos[2] = pos[2] / newR * minR;

        float up_normalized[3] = { pos[0] / minR, pos[1] / minR, pos[2] / minR };
        float radialVelocity = dotProduct(velocity, up_normalized);

        if (radialVelocity < 0.0f) {
            velocity[0] -= up_normalized[0] * radialVelocity;
            velocity[1] -= up_normalized[1] * radialVelocity;
            velocity[2] -= up_normalized[2] * radialVelocity;
        }

        if (currentMode != MODE_CRUISE && isWarhead) {
            if (!hasExploded) { hasExploded = true; flashTime = 1.0f; explosionScale = 0.0f; }
            velocity[0] = 0; velocity[1] = 0; velocity[2] = 0;
        }
    }
}

 //FIXED GUIDED UPDATE FUNCTION
void updateGuidedPosition(float dt) {
    float t = simTime / FLIGHT_DURATION;
    if (t > 1.0f) t = 1.0f;

    // 1. Move Rocket
    float prevPos[3] = { rocketPos[0], rocketPos[1], rocketPos[2] };
    calculateHorizontalTrajectory(t, rocketPos);

    // 2. Stage Separation Logic
    if (stageState == STAGE_ATTACHED && t > 0.2f) {
        stageState = STAGE_SEPARATED;

        boosterPos[0] = prevPos[0];
        boosterPos[1] = prevPos[1];
        boosterPos[2] = prevPos[2];

        float sepDir[3] = {
            rocketPos[0] - prevPos[0],
            rocketPos[1] - prevPos[1],
            rocketPos[2] - prevPos[2]
        };
        normalize(sepDir);

        boosterVelocity[0] = sepDir[0] * 50.0f;
        boosterVelocity[1] = sepDir[1] * 50.0f;
        boosterVelocity[2] = sepDir[2] * 50.0f;

        boosterDir[0] = sepDir[0];
        boosterDir[1] = sepDir[1];
        boosterDir[2] = sepDir[2];
    }

    // 3. Orientation Fix: follow trajectory tangent
    float lookAheadT = t + 0.005f;
    if (lookAheadT > 1.0f) lookAheadT = 1.0f;

    float nextPos[3];
    calculateHorizontalTrajectory(lookAheadT, nextPos);

    float pathDir[3] = {
        nextPos[0] - rocketPos[0],
        nextPos[1] - rocketPos[1],
        nextPos[2] - rocketPos[2]
    };
    normalize(pathDir);

    rocketDir[0] = pathDir[0];
    rocketDir[1] = pathDir[1];
    rocketDir[2] = pathDir[2];

    velocityToOrientation(pathDir, rocketPos, &rocketOrient[0], &rocketOrient[1]);

    // 4. Update Trail
    if (guidedTrajectory.empty() ||
        distance(rocketPos, &guidedTrajectory[guidedTrajectory.size() - 3]) > 2.0f)
    {
        guidedTrajectory.push_back(rocketPos[0]);
        guidedTrajectory.push_back(rocketPos[1]);
        guidedTrajectory.push_back(rocketPos[2]);
    }
}


void updateCruisePosition(float dt) {
    float r = sqrt(rocketPos[0] * rocketPos[0] + rocketPos[1] * rocketPos[1] + rocketPos[2] * rocketPos[2]);
    float currentAltitude = r - EARTH_RADIUS;

    if (stageState == STAGE_ATTACHED) {
        float boost_r = EARTH_RADIUS + TARGET_CRUISE_ALTITUDE;
        if (r < boost_r) {
            rocketOrient[1] = 80.0f;
            applyPhysics(rocketPos, rocketVelocity, GUIDED_THRUST, dt, rocketOrient[0], rocketOrient[1], true);
        }
        else {
            stageState = STAGE_SEPARATED;
            boosterPos[0] = rocketPos[0]; boosterPos[1] = rocketPos[1]; boosterPos[2] = rocketPos[2];
            boosterVelocity[0] = rocketVelocity[0]; boosterVelocity[1] = rocketVelocity[1]; boosterVelocity[2] = rocketVelocity[2];
            boosterOrient[0] = rocketOrient[0]; boosterOrient[1] = rocketOrient[1];

            float fwd[3], up[3], right[3];
            getRocketVectors(rocketPos, rocketOrient[0], 90.0f, fwd, up, right);
            rocketVelocity[0] = fwd[0] * CRUISE_FLIGHT_SPEED;
            rocketVelocity[1] = fwd[1] * CRUISE_FLIGHT_SPEED;
            rocketVelocity[2] = fwd[2] * CRUISE_FLIGHT_SPEED;
            simTime = 0.0f;
        }
    }
    else {
        float up[3] = { rocketPos[0], rocketPos[1], rocketPos[2] };
        normalize(up);

        float radialVelocity = dotProduct(rocketVelocity, up);
        float radialError = currentAltitude - TARGET_CRUISE_ALTITUDE;
        float thrustCorrection = -radialError * 0.5f - radialVelocity * 0.8f;

        float vel_mag = sqrt(rocketVelocity[0] * rocketVelocity[0] + rocketVelocity[1] * rocketVelocity[1] + rocketVelocity[2] * rocketVelocity[2]);
        float speedError = CRUISE_FLIGHT_SPEED - vel_mag;
        float speedThrust = speedError * 2.0f;

        rocketVelocity[0] += up[0] * thrustCorrection * dt * CRUISE_GUIDANCE_FORCE;
        rocketVelocity[1] += up[1] * thrustCorrection * dt * CRUISE_GUIDANCE_FORCE;
        rocketVelocity[2] += up[2] * thrustCorrection * dt * CRUISE_GUIDANCE_FORCE;

        float totalThrust = GUIDED_THRUST + speedThrust;

        velocityToOrientation(rocketVelocity, rocketPos, &rocketOrient[0], &rocketOrient[1]);
        rocketOrient[1] = 90.0f;

        applyPhysics(rocketPos, rocketVelocity, totalThrust, dt, rocketOrient[0], rocketOrient[1], true);

        if (guidedTrajectory.empty() || distance(rocketPos, &guidedTrajectory[guidedTrajectory.size() - 3]) > 0.1f) {
            guidedTrajectory.push_back(rocketPos[0]);
            guidedTrajectory.push_back(rocketPos[1]);
            guidedTrajectory.push_back(rocketPos[2]);
        }
    }
}

void updateBoosterPhysics(float dt) {
    if (stageState != STAGE_SEPARATED) return;
    applyPhysics(boosterPos, boosterVelocity, 0.0f, dt, boosterOrient[0], boosterOrient[1], false);
    boosterOrient[0] += dt * 50.0f;
    boosterOrient[1] += dt * 30.0f;
}

void updateManualPosition(float dt) {
    float thrust = 0.0f;
    if (isThrusting && stageState == STAGE_ATTACHED) thrust = MANUAL_THRUST_MAX;
    else if (isWarheadThrusting && stageState == STAGE_SEPARATED) thrust = MANUAL_THRUST_MAX;

    float fwd[3], up[3], right[3];
    getRocketVectors(rocketPos, rocketOrient[0], rocketOrient[1], fwd, up, right);
    if (thrust > 0.0f) {
        float dot_val = rocketVelocity[0] * fwd[0] + rocketVelocity[1] * fwd[1] + rocketVelocity[2] * fwd[2];
        float sideVelocity[3];
        sideVelocity[0] = rocketVelocity[0] - fwd[0] * dot_val;
        sideVelocity[1] = rocketVelocity[1] - fwd[1] * dot_val;
        sideVelocity[2] = rocketVelocity[2] - fwd[2] * dot_val;

        rocketVelocity[0] -= sideVelocity[0] * DAMPING_FACTOR * dt;
        rocketVelocity[1] -= sideVelocity[1] * DAMPING_FACTOR * dt;
        rocketVelocity[2] -= sideVelocity[2] * DAMPING_FACTOR * dt;
    }

    bool isWarhead = (stageState == STAGE_SEPARATED);
    applyPhysics(rocketPos, rocketVelocity, thrust, dt, rocketOrient[0], rocketOrient[1], isWarhead);
}


// ---------------------------------------------------------
// --- DRAWING FUNCTIONS ---
// ---------------------------------------------------------


//Draws a cylindrical shape used as the main building block for the rocket body and its components.
void drawCylinder(float h, float rBase, float rTop, float r, float g, float b) {
    glColor3f(1.0f, 1.0f, 1.0f);
    GLUquadric* q = gluNewQuadric();
    gluCylinder(q, rBase, rTop, h, 16, 1);
    gluDeleteQuadric(q);
}

//Draws the rocket according to its current stage(booster or warhead) and displays engine flame effects when thrust is active.
void drawRocketParts(bool drawBooster, bool drawWarhead)
{
    glEnable(GL_LIGHTING);

    /* =======================
       BOOSTER (Stage 1)
       ======================= */
    if (drawBooster) {
        glPushMatrix();

        // Body
        glColor3f(0.9f, 0.9f, 0.9f);
        glTranslatef(0.0f, 0.0f, -ROCKET_CYLINDER_HEIGHT);
        drawCylinder(
            ROCKET_CYLINDER_HEIGHT,
            ROCKET_RADIUS,
            ROCKET_RADIUS,
            1.0f, 1.0f, 1.0f
        );

        // Engine nozzle
        glColor3f(0.4f, 0.4f, 0.4f);
        glutSolidCone(ROCKET_RADIUS * 1.5f, 0.2f, 16, 8);

        // Flame condition
        bool drawFlame =
            (currentMode != MODE_MANUAL &&
                stageState == STAGE_ATTACHED)
            ||
            (currentMode == MODE_MANUAL &&
                isThrusting &&
                stageState == STAGE_ATTACHED);

        if (drawFlame) {
            glDisable(GL_LIGHTING);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);

            glColor4f(1.0f, 0.5f, 0.1f, 0.8f);

            glTranslatef(0.0f, 0.0f, -1.0f);

            glutSolidCone(
                ROCKET_RADIUS * 1.6f,
                1.0f,
                16,
                1
            );

            glDisable(GL_BLEND);
            glEnable(GL_LIGHTING);
        }

        glPopMatrix();  
    }

    /* =======================
       WARHEAD (Stage 2)
       ======================= */
    if (drawWarhead) {
        glPushMatrix();

        // Warhead body
        glColor3f(0.85f, 0.85f, 0.85f);
        drawCylinder(
            ROCKET_CONE_HEIGHT,
            ROCKET_RADIUS,
            0.0f,
            1.0f, 1.0f, 1.0f
        );

        bool drawFlame =
            (currentMode != MODE_MANUAL &&
                stageState == STAGE_SEPARATED)
            ||
            (currentMode == MODE_MANUAL &&
                isWarheadThrusting &&
                stageState == STAGE_SEPARATED);

        if (drawFlame) {
            glDisable(GL_LIGHTING);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);

            glColor4f(1.0f, 0.4f, 0.1f, 0.7f);

            glTranslatef(0.0f, 0.0f, -0.8f);

            glutSolidCone(
                ROCKET_RADIUS * 0.9f,
                0.6f,
                16,
                1
            );

            glDisable(GL_BLEND);
            glEnable(GL_LIGHTING);
        }



        glPopMatrix();
    }
}

//Draws the path the rocket has traveled, and in guided mode also shows the predicted trajectory.
void drawTrajectoryTrail() {
    if (currentMode == MODE_MANUAL || hasExploded) return;

    if (guidedTrajectory.size() >= 6) {
        glDisable(GL_LIGHTING);
        glLineWidth(2.0f);
        glColor3f(0.0f, 0.5f, 1.0f);
        glBegin(GL_LINE_STRIP);
        for (size_t i = 0; i < guidedTrajectory.size(); i += 3) {
            glVertex3f(guidedTrajectory[i], guidedTrajectory[i + 1], guidedTrajectory[i + 2]);
        }
        glEnd();
    }

    if (currentMode == MODE_GUIDED) {
        glLineWidth(2.0f);
        glLineStipple(1, 0x0F0F);
        glEnable(GL_LINE_STIPPLE);
        glColor3f(1.0f, 0.0f, 1.0f);
        glBegin(GL_LINE_STRIP);
        const int steps = 100;
        for (int i = 0; i <= steps; ++i) {
            float t = (float)i / steps;
            float predictedPos[3];
            calculateHorizontalTrajectory(t, predictedPos);
            glVertex3f(predictedPos[0], predictedPos[1], predictedPos[2]);
        }
        glEnd();
        glDisable(GL_LINE_STIPPLE);
    }
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

//Draws reference points on the Earth for the launch location(START) and the target location(TARGET), and hides them after the explosion.
void drawMarkers() {
    if (currentMode == MODE_MANUAL || hasExploded) return;

    glDisable(GL_LIGHTING);

    float pos[3];
    float r_launch = EARTH_RADIUS + ROCKET_CYLINDER_HEIGHT * 0.5f + 0.1f;

    auto drawPointWithText = [&](float lat, float lon,
        float R,
        float pr, float pg, float pb,
        const char* label) {
            toCartesian(lat, lon, R, pos);

            glPointSize(8.0f);
            glColor3f(pr, pg, pb);
            glBegin(GL_POINTS);
            glVertex3f(pos[0], pos[1], pos[2]);
            glEnd();

            glColor3f(1.0f, 1.0f, 1.0f);
            glRasterPos3f(
                pos[0] * 1.01f,
                pos[1] * 1.01f,
                pos[2] * 1.01f
            );

            for (const char* c = label; *c != '\0'; c++) {
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
            }
        };

   
    drawPointWithText(
        launchLat, launchLon,
        r_launch,
        1.0f, 0.0f, 0.0f,
        "START"
    );

    
    if (currentMode == MODE_GUIDED) {
        drawPointWithText(
            targetLat, targetLon,
            r_launch,
            0.0f, 1.0f, 0.0f,
            "TARGET"
        );
    }

    glPointSize(1.0f);
    glEnable(GL_LIGHTING);
}


//Moves the rocket to its correct position, rotates it according to its direction, and then draws the appropriate rocket parts.
void transformAndDraw(float* pos, float* dir,
    bool drawBoost, bool drawHead)
{
    glPushMatrix();
    glTranslatef(pos[0], pos[1], pos[2]);

    float zAxis[3] = { 0.0f, 0.0f, 1.0f };
    float axis[3];
    crossProduct(zAxis, dir, axis);

    float dot = dotProduct(zAxis, dir);
    dot = std::max(-1.0f, std::min(1.0f, dot));
    float angle = acos(dot) * 180.0f / M_PI;

    if (fabs(angle) > 0.001f) {
        glRotatef(angle, axis[0], axis[1], axis[2]);
    }

    drawRocketParts(drawBoost, drawHead);
    glPopMatrix();
}




//Draws a 3D explosion effect at the impact location when the rocket reaches its target or collides with the ground.
void drawExplosion() {
    if (!hasExploded) return;
    glDisable(GL_LIGHTING); glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    glPushMatrix();
    if (currentMode == MODE_GUIDED) {
        float r_launch = EARTH_RADIUS + ROCKET_CYLINDER_HEIGHT * 0.5f + 0.1f;
        float endGroundPos[3];
        toCartesian(targetLat, targetLon, r_launch, endGroundPos);
        glTranslatef(endGroundPos[0], endGroundPos[1], endGroundPos[2]);
    }
    else {
        glTranslatef(rocketPos[0], rocketPos[1], rocketPos[2]);
    }

    float alpha = std::min(1.0f, 1.0f / (explosionScale * 0.5f + 1.0f));
    glColor4f(1.0f, 0.5f, 0.0f, alpha);
    glutSolidSphere(explosionScale * 2.0f, 16, 16);

    if (explosionScale > 0.5f) {
        float smokeAlpha = std::min(1.0f, 1.0f / (explosionScale * 0.2f + 1.0f));
        glColor4f(0.3f, 0.3f, 0.3f, smokeAlpha * 0.5f);
        glutSolidSphere(explosionScale * 4.0f, 16, 16);
    }
    glPopMatrix();
    glDisable(GL_BLEND); glEnable(GL_LIGHTING);
}

 //Displays a full - screen white flash to simulate the intense light of a nuclear explosion.
void drawNuclearFlash() {
    if (flashTime <= 0.0f) return;
    glDisable(GL_DEPTH_TEST); glDisable(GL_LIGHTING);
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    float alpha = std::min(1.0f, flashTime * 4.0f);
    glColor4f(1.0f, 1.0f, 1.0f, alpha);

    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity(); gluOrtho2D(0, 1, 0, 1);
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();

    glBegin(GL_QUADS);
    glVertex2f(0, 0); glVertex2f(1, 0); glVertex2f(1, 1); glVertex2f(0, 1);
    glEnd();

    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW);
    glDisable(GL_BLEND); glEnable(GL_DEPTH_TEST); glEnable(GL_LIGHTING);
}

//Draws text on the screen at a specified position, mainly used for HUD information.
void drawString(float x, float y, void* font, const char* string) {
    glRasterPos2f(x, y);
    for (const char* c = string; *c != '\0'; c++) {
        glutBitmapCharacter(font, *c);
    }
}

//Displays on - screen information such as the current mode, time, altitude, and control instructions.
void drawControlsHUD() {
    glDisable(GL_LIGHTING); glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity(); gluOrtho2D(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT);
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();

    glColor3f(1.0f, 1.0f, 1.0f);
    void* font = GLUT_BITMAP_HELVETICA_12;
    char buffer[100];

    float r = sqrt(rocketPos[0] * rocketPos[0] + rocketPos[1] * rocketPos[1] + rocketPos[2] * rocketPos[2]);
    float altitude = (r - EARTH_RADIUS - 0.1f);

    // Top Status
    if (currentMode == MODE_MANUAL) {
        snprintf(buffer, sizeof(buffer), "MODE: MANUAL | ALT: %.2f km | STAGE: %s", altitude, (stageState == STAGE_ATTACHED) ? "STAGE 1" : "STAGE 2");
        drawString(10, WINDOW_HEIGHT - 30, GLUT_BITMAP_HELVETICA_18, buffer);
    }
    else if (currentMode == MODE_GUIDED) {
        const char* status = (altitude < HORIZONTAL_FLIGHT_ALTITUDE * 0.9f) ? "ASCENDING" : (altitude > HORIZONTAL_FLIGHT_ALTITUDE * 1.1f ? "DESCENDING" : "CRUISE");
        snprintf(buffer, sizeof(buffer), "MODE: GUIDED | Time: %.1fs / %.1fs | Status: %s", simTime, FLIGHT_DURATION, status);
        drawString(10, WINDOW_HEIGHT - 30, GLUT_BITMAP_HELVETICA_18, buffer);
        snprintf(buffer, sizeof(buffer), "WARHEAD TRACKING: %s | 'P' to Toggle", isRocketTracking ? "ON" : "OFF");
        drawString(10, WINDOW_HEIGHT - 50, GLUT_BITMAP_HELVETICA_18, buffer);
    }
    else if (currentMode == MODE_CRUISE) {
        snprintf(buffer, sizeof(buffer), "MODE: CRUISE | Sim Time: %.1fs", simTime);
        drawString(10, WINDOW_HEIGHT - 30, GLUT_BITMAP_HELVETICA_18, buffer);
    }

    drawString(10, 85, font, "--- CONTROLS ---");
    drawString(10, 70, font, "P: Toggle Camera | W/S: Zoom | A/D: Rotate | Mouse: Orbit");
    drawString(10, 55, font, "Q/E: Pitch | R: Reset Cam | +/-: Time Scale");

    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW);
    glEnable(GL_LIGHTING); glEnable(GL_DEPTH_TEST);
}

//Renders the entire 3D scene, including the background, Earth, rocket, trajectory, markers, and explosion effects.
void drawScene() {
    // ===== Stars =====
    glDisable(GL_LIGHTING);
    glBegin(GL_POINTS);
    for (const auto& star : stars) {
        glColor3f(star.b, star.b, star.b);
        glVertex3f(star.x, star.y, star.z);
    }
    glEnd();
    glEnable(GL_LIGHTING);

    // ===== Earth =====
    glPushMatrix();
    GLUquadric* quadric = gluNewQuadric();
    gluQuadricTexture(quadric, GL_TRUE);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, earthTextureID);
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    gluSphere(quadric, EARTH_RADIUS, 60, 60);
    glDisable(GL_TEXTURE_2D);
    gluDeleteQuadric(quadric);
    glPopMatrix();

    // ===== Trajectory & Markers =====
    drawTrajectoryTrail();
    drawMarkers();

    // ===== Rocket & Booster =====
    if (!hasExploded) {

        //  Rocket (direction-based)
        transformAndDraw(
            rocketPos,
            rocketDir,
            stageState == STAGE_ATTACHED,
            true
        );

        //  Booster (after separation)
        if (stageState == STAGE_SEPARATED) {
            transformAndDraw(
                boosterPos,
                boosterDir,
                true,
                false
            );
        }
    }

    // ===== Explosion =====
    if (hasExploded)
        drawExplosion();
}

//Renders one complete frame by setting up the camera, drawing the scene, and swapping the buffers
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();

    float currentCamDist = camDist;
    if ((currentMode == MODE_GUIDED || currentMode == MODE_CRUISE) && isRocketTracking) {
        currentCamDist = ROCKET_CYLINDER_HEIGHT * 10.0f + 10.0f;
    }

    glTranslatef(0.0f, 0.0f, -currentCamDist);
    glRotatef(camAngleX, 1.0f, 0.0f, 0.0f);
    glRotatef(camAngleY, 0.0f, 1.0f, 0.0f);

    if ((currentMode == MODE_GUIDED || currentMode == MODE_CRUISE) && isRocketTracking) {
        for (int i = 0; i < 3; ++i) {
            trackingCamTarget[i] = trackingCamTarget[i] * (1.0f - TRACKING_CAM_LAG) + rocketPos[i] * TRACKING_CAM_LAG;
        }
        glTranslatef(-trackingCamTarget[0], -trackingCamTarget[1], -trackingCamTarget[2]);
    }

    drawScene();
    drawNuclearFlash();
    drawControlsHUD();
    glutSwapBuffers();
}


//Updates the simulation over time, applies physics and logic updates, and continuously refreshes the scene
void update(int v) {
    float dt = 0.016f * timeScale;
    if (!hasExploded) {
        if (currentMode == MODE_GUIDED) {
            if (simTime < FLIGHT_DURATION) {
                updateGuidedPosition(dt);
                updateBoosterPhysics(dt);
                simTime += dt;
            }
            else if (!hasExploded) {
                hasExploded = true; flashTime = 1.0f; explosionScale = 0.0f;
                float r_launch = EARTH_RADIUS + ROCKET_CYLINDER_HEIGHT * 0.5f + 0.1f;
                toCartesian(targetLat, targetLon, r_launch, rocketPos);
                rocketVelocity[0] = 0; rocketVelocity[1] = 0; rocketVelocity[2] = 0;
                simTime = FLIGHT_DURATION;
            }
        }
        else if (currentMode == MODE_CRUISE) {
            updateCruisePosition(dt);
            updateBoosterPhysics(dt);
            if (stageState == STAGE_SEPARATED && simTime < CRUISE_DURATION) simTime += dt;
            else if (stageState == STAGE_SEPARATED) {
                if (!hasExploded) { hasExploded = true; flashTime = 1.0f; explosionScale = 0.0f; }
            }
        }
        else {
            updateManualPosition(dt);
            updateBoosterPhysics(dt);
        }
    }
    else explosionScale += dt * 10.0f;

    if (flashTime > 0.0f) flashTime = std::max(0.0f, flashTime - dt * 2.5f);
    glutPostRedisplay();
    glutTimerFunc(16, update, 0);
}

//Handles keyboard input to control the rocket, camera movement, mode switching, and time scaling.
void keyboard(unsigned char k, int x, int y)
{
    if (k == 27) exit(0); // ESC

    /* ==========================
       MANUAL MODE CONTROLS
       ========================== */
    if (currentMode == MODE_MANUAL)
    {
        // Thrust ON
        if (k == 'w' || k == 'W') {
            if (stageState == STAGE_ATTACHED)
                isThrusting = true;
            else
                isWarheadThrusting = true;
        }

        // Thrust OFF
        if (k == 's' || k == 'S') {
            if (stageState == STAGE_ATTACHED)
                isThrusting = false;
            else
                isWarheadThrusting = false;
        }

        // SPACE → Stage separation / explosion
        if (k == ' ')
        {
            if (stageState == STAGE_ATTACHED)
            {
                stageState = STAGE_SEPARATED;
                isThrusting = false;

                // Booster starts at rocket position
                boosterPos[0] = rocketPos[0];
                boosterPos[1] = rocketPos[1];
                boosterPos[2] = rocketPos[2];

                // Booster keeps rocket velocity
                boosterVelocity[0] = rocketVelocity[0];
                boosterVelocity[1] = rocketVelocity[1];
                boosterVelocity[2] = rocketVelocity[2];

                // === Booster direction = rocket movement direction ===
                float len = sqrt(
                    rocketVelocity[0] * rocketVelocity[0] +
                    rocketVelocity[1] * rocketVelocity[1] +
                    rocketVelocity[2] * rocketVelocity[2]
                );

                if (len > 0.0001f) {
                    boosterDir[0] = rocketVelocity[0] / len;
                    boosterDir[1] = rocketVelocity[1] / len;
                    boosterDir[2] = rocketVelocity[2] / len;
                }
                else {
                    boosterDir[0] = 0.0f;
                    boosterDir[1] = 0.0f;
                    boosterDir[2] = 1.0f;
                }
            }
            else if (!hasExploded)
            {
                hasExploded = true;
                flashTime = 1.0f;
            }
        }
    }

    /* ==========================
       CAMERA TRACKING (GUIDED / CRUISE)
       ========================== */
    if ((currentMode == MODE_GUIDED || currentMode == MODE_CRUISE) &&
        (k == 'p' || k == 'P'))
    {
        isRocketTracking = !isRocketTracking;
        if (isRocketTracking) {
            trackingCamTarget[0] = rocketPos[0];
            trackingCamTarget[1] = rocketPos[1];
            trackingCamTarget[2] = rocketPos[2];
        }
    }

    /* ==========================
       CAMERA CONTROLS
       ========================== */
    if (k == 'r' || k == 'R') {
        camAngleX = 20.0f;
        camAngleY = 45.0f;
        camDist = INITIAL_CAM_DISTANCE;
    }

    if (k == 'w' || k == 'W')
        camDist = std::max(1.1f * EARTH_RADIUS, camDist - 10.0f);

    if (k == 's' || k == 'S')
        camDist = std::min(5000.0f, camDist + 10.0f);

    if (k == 'a' || k == 'A')
        camAngleY += 1.0f;

    if (k == 'd' || k == 'D')
        camAngleY -= 1.0f;

    if (k == 'e' || k == 'E')
        camAngleX -= 1.0f;

    if (k == 'q' || k == 'Q')
        camAngleX += 1.0f;

    /* ==========================
       TIME SCALE
       ========================== */
    if (k == '=')
        timeScale += 0.5f;

    if (k == '-')
        timeScale = std::max(0.1f, timeScale - 0.5f);
}

//Handles mouse button input, such as starting camera orbiting or aiming in manual mode.
void mouse(int b, int s, int x, int y) {
    if (b == GLUT_LEFT_BUTTON) { isMouseOrbiting = (s == GLUT_DOWN); lastMx = x; lastMy = y; }
    if (currentMode == MODE_MANUAL && b == GLUT_RIGHT_BUTTON) { isAiming = (s == GLUT_DOWN); lastMx = x; lastMy = y; }
}

//Handles mouse movement to rotate the camera or adjust the rocket’s orientation while aiming.
void motion(int x, int y) {
    int dx = x - lastMx; int dy = y - lastMy;
    if (isMouseOrbiting && !isRocketTracking) {
        camAngleY += dx * 0.5f;
        camAngleX += dy * 0.5f;
        camAngleX = std::max(-89.0f, std::min(89.0f, camAngleX));
    }
    if (currentMode == MODE_MANUAL && isAiming) {
        rocketOrient[0] -= dx * TURN_RATE * 0.15f;
        rocketOrient[1] -= dy * TURN_RATE * 0.15f;
        rocketOrient[1] = std::max(10.0f, std::min(170.0f, rocketOrient[1]));
    }
    lastMx = x; lastMy = y;
}

//Controls camera zoom in and out using the mouse wheel.
void mouseWheel(int wheel, int direction, int x, int y) {
    if (direction > 0) camDist = std::max(1.1f * EARTH_RADIUS, camDist - 100.0f);
    else camDist = std::min(5000.0f, camDist + 100.0f);
}

//Loads an image file and converts it into an OpenGL texture, mainly used for the Earth surface.
void loadTexture(const char* filename, GLuint* textureID) {
    int w, h, c = 0;
    unsigned char* data = stbi_load(filename, &w, &h, &c, 0);

    if (!data) {
        printf(" FAILED to load texture: %s\n", filename);
        *textureID = 0;
        return;
    }

    printf(" Texture loaded: %s (%d x %d | channels=%d)\n", filename, w, h, c);

    glGenTextures(1, textureID);
    glBindTexture(GL_TEXTURE_2D, *textureID);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    GLint format = (c == 4) ? GL_RGBA : GL_RGB;
    glTexImage2D(GL_TEXTURE_2D, 0, format, w, h, 0, format, GL_UNSIGNED_BYTE, data);

    stbi_image_free(data);
}

//Initializes OpenGL settings such as lighting, depth testing, background color, textures, and star generation.
void init() {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);

    // Light position (sun-like)
    GLfloat pos[] = { 5000.0f, 5000.0f, 5000.0f, 0.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, pos);

    // Light colors 
    GLfloat ambient[] = { 0.3f, 0.3f, 0.3f, 1.0f };
    GLfloat diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);

    // Background
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    // Load Earth texture
    loadTexture("map.png", &earthTextureID);
    //loadTexture("assets/earth_map.png", &earthTextureID);

    // Stars
    srand((unsigned int)time(NULL));
    for (int i = 0; i < NUM_STARS; i++) {
        Star s;
        s.x = (rand() % 2000 - 1000) * 10.0f;
        s.y = (rand() % 2000 - 1000) * 10.0f;
        s.z = (rand() % 2000 - 1000) * 10.0f;
        s.b = 0.5f + (rand() % 10) / 20.0f;
        stars.push_back(s);
    }
}

//Updates the viewport and projection matrix when the window size changes.
void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    gluPerspective(45, (float)w / h, 0.1, 50000);
    glMatrixMode(GL_MODELVIEW);
}

//Collects simulation parameters from the user, such as mode selection and launch / target coordinates, before starting the simulation.
void getUserInput() {
    stageState = STAGE_ATTACHED; thrustState = THRUSTING; hasExploded = false;
    simTime = 0.0f; guidedTrajectory.clear(); isThrusting = false;
    camDist = INITIAL_CAM_DISTANCE; camAngleX = 20.0f; camAngleY = 45.0f;

    int mode_choice;
    std::cout << "\n--- ROCKET SIMULATOR ---\n";
    std::cout << "1. Guided (Smooth Path)\n2. Manual\n3. Cruise\nEnter mode: ";
    std::cin >> mode_choice;

    float r = EARTH_RADIUS + ROCKET_CYLINDER_HEIGHT * 0.5f + 0.1f;
    if (mode_choice == 1) {
        currentMode = MODE_GUIDED;
        std::cout << "Enter Launch Lat/Lon (e.g. 28.5 -80.6): "; std::cin >> launchLat >> launchLon;
        std::cout << "Enter Target Lat/Lon (e.g. 35.0 -85.0): "; std::cin >> targetLat >> targetLon;
        toCartesian(launchLat, launchLon, r, startPos);
        toCartesian(targetLat, targetLon, r, endPos);
        calculateTotalArcLength();
        rocketPos[0] = startPos[0]; rocketPos[1] = startPos[1]; rocketPos[2] = startPos[2];
        boosterPos[0] = startPos[0]; boosterPos[1] = startPos[1]; boosterPos[2] = startPos[2];
        float idealYaw = calculateInitialYaw(startPos, endPos);
        rocketOrient[0] = idealYaw; rocketOrient[1] = 85.0f;
    }
    else if (mode_choice == 3) {
        currentMode = MODE_CRUISE;
        std::cout << "Enter Launch Lat/Lon: "; std::cin >> launchLat >> launchLon;
        toCartesian(launchLat, launchLon, r, rocketPos);
        rocketOrient[0] = 45.0f; rocketOrient[1] = 90.0f;
    }
    else {
        currentMode = MODE_MANUAL;
        std::cout << "Enter Launch Lat/Lon: "; std::cin >> launchLat >> launchLon;
        toCartesian(launchLat, launchLon, r, rocketPos);
        rocketOrient[0] = 0.0f; rocketOrient[1] = 90.0f;
    }
}

//The entry point of the program Initializes GLUT, creates the window, registers callbacks, and starts the main simulation loop.
int main(int argc, char** argv) {
    getUserInput();
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    glutCreateWindow("Rocket Impact");
    init();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutTimerFunc(16, update, 0);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutMouseWheelFunc(mouseWheel);
    glutMainLoop();
    return 0;
}