// solar_system_perfect_fixed.cpp
// Professional Solar System Simulation - OpenGL + freeglut




#include <GL/freeglut.h>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

#define PI 3.14159265358979323846f
#define NUM_STARS 1000

static float yearAngle = 0.0f;
static float dayAngle = 0.0f;
static float rot_x = 25.0f;
static float rot_y = -45.0f;
static float distance_cam = 80.0f;
static int last_mouse_x = 0, last_mouse_y = 0;
static int mouse_button = -1;
static bool paused = false;
static float speed_scale = 1.0f;
static bool showOrbits = true;
static bool showLabels = true;
static bool showGrid = false;

struct Star {
    float x, y, z;
    float size;
    float brightness;
};

static Star stars[NUM_STARS];

struct PlanetInfo {
    std::string name;
    float orbit_radius;
    float planet_radius;
    float orbit_speed;
    float rotation_speed;
    float inclination;
    float eccentricity;
    float axial_tilt;
    float r, g, b;
    float orbit_offset;
    int moons;
    bool has_rings;
    float ring_inner;
    float ring_outer;
};

std::vector<PlanetInfo> planets = {
    {"Mercury", 5.8f, 0.38f, 4.15f, 0.17f, 7.0f, 0.205f, 0.034f, 0.7f, 0.6f, 0.5f, 0.0f, 0, false},
    {"Venus", 10.8f, 0.95f, 1.62f, 0.004f, 3.4f, 0.007f, 177.4f, 0.9f, 0.8f, 0.4f, 0.0f, 0, false},
    {"Earth", 15.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.017f, 23.44f, 0.2f, 0.4f, 0.8f, 0.0f, 1, false},
    {"Mars", 22.8f, 0.53f, 0.53f, 1.03f, 1.85f, 0.093f, 25.19f, 0.8f, 0.3f, 0.2f, 0.0f, 2, false},
    {"Jupiter", 38.0f, 2.5f, 0.08f, 2.4f, 1.31f, 0.048f, 3.13f, 0.9f, 0.8f, 0.6f, 0.0f, 79, false},
    {"Saturn", 52.0f, 2.1f, 0.03f, 2.3f, 2.49f, 0.054f, 26.73f, 0.95f, 0.9f, 0.7f, 0.0f, 82, true, 2.5f, 5.0f},
    {"Uranus", 72.0f, 1.4f, 0.01f, 1.4f, 0.77f, 0.047f, 97.77f, 0.6f, 0.8f, 0.95f, 0.0f, 27, true, 1.8f, 2.5f},
    {"Neptune", 85.0f, 1.35f, 0.006f, 1.5f, 1.77f, 0.009f, 28.32f, 0.2f, 0.3f, 0.8f, 0.0f, 14, false}
};

void generateStars() {
    srand(static_cast<unsigned>(time(NULL)));
    for (int i = 0; i < NUM_STARS; ++i) {
        float radius = 100.0f + (rand() / (float)RAND_MAX) * 300.0f;
        float theta = (rand() / (float)RAND_MAX) * 2.0f * PI;
        float phi = acosf(2.0f * (rand() / (float)RAND_MAX) - 1.0f);
        stars[i].x = radius * sinf(phi) * cosf(theta);
        stars[i].y = radius * sinf(phi) * sinf(theta) * 0.3f;
        stars[i].z = radius * cosf(phi);
        stars[i].size = 0.5f + (rand() / (float)RAND_MAX) * 2.0f;
        stars[i].brightness = 0.3f + (rand() / (float)RAND_MAX) * 0.7f;
    }
}

void drawStars() {
    glDisable(GL_LIGHTING);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

    glPointSize(1.0f);
    glBegin(GL_POINTS);
    for (int i = 0; i < NUM_STARS; ++i) {
        float intensity = stars[i].brightness;
        glColor3f(intensity, intensity * 0.95f, intensity * 0.9f);
        glVertex3f(stars[i].x, stars[i].y, stars[i].z);
    }
    glEnd();

    glPointSize(2.0f);
    glBegin(GL_POINTS);
    for (int i = 0; i < NUM_STARS / 20; ++i) {
        int idx = i * 20;
        float intensity = stars[idx].brightness;
        glColor3f(intensity * 1.2f, intensity, intensity * 0.8f);
        glVertex3f(stars[idx].x, stars[idx].y, stars[idx].z);
    }
    glEnd();

    glDisable(GL_POINT_SMOOTH);
    glEnable(GL_LIGHTING);
}

void drawGrid() {
    if (!showGrid) return;
    glDisable(GL_LIGHTING);
    glColor3f(0.2f, 0.2f, 0.3f);
    glLineWidth(0.5f);

    const int grid_size = 100;
    const int step = 10;

    glBegin(GL_LINES);
    for (int i = -grid_size; i <= grid_size; i += step) {
        glVertex3f((float)i, 0.0f, (float)-grid_size);
        glVertex3f((float)i, 0.0f, (float)grid_size);
        glVertex3f((float)-grid_size, 0.0f, (float)i);
        glVertex3f((float)grid_size, 0.0f, (float)i);
    }
    glEnd();

    glEnable(GL_LIGHTING);
}

void drawOrbit(const PlanetInfo& planet) {
    if (!showOrbits) return;

    glDisable(GL_LIGHTING);
    glColor4f(0.3f, 0.4f, 0.6f, 0.3f);
    glLineWidth(1.0f);

    const int segments = 360;
    glBegin(GL_LINE_LOOP);

    for (int i = 0; i < segments; ++i) {
        float angle = 2.0f * PI * i / segments;
        float r = planet.orbit_radius * (1.0f - planet.eccentricity * planet.eccentricity)
            / (1.0f + planet.eccentricity * cosf(angle + planet.orbit_offset));

        // التصحيح هنا - حساب الميل الصحيح
        float inclination_rad = planet.inclination * PI / 180.0f;
        float x = r * cosf(angle);
        float z = r * sinf(angle);
        float y = x * sinf(inclination_rad) * sinf(angle); // تصحيح الصيغة

        glVertex3f(x, y, z);
    }
    glEnd();

    glEnable(GL_LIGHTING);
}

void drawPlanetName(float x, float y, float z, const std::string& name, float planet_radius) {
    if (!showLabels) return;

    glDisable(GL_LIGHTING);
    glPushMatrix();
    glTranslatef(x, y + planet_radius * 1.5f, z);

    float modelview[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, modelview);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            modelview[i * 4 + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    glLoadMatrixf(modelview);
    glScalef(0.003f, 0.003f, 0.003f);
    glColor3f(1.0f, 1.0f, 1.0f);

    glRasterPos2f(0, 0);
    for (char c : name) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
    }

    glPopMatrix();
    glEnable(GL_LIGHTING);
}

void drawSun() {
    glPushMatrix();

    // تحسين شكل الشمس
    glDisable(GL_LIGHTING);

    // هالة الشمس
    for (int i = 0; i < 5; ++i) {
        float radius = 4.0f + i * 0.6f;
        float alpha = 0.1f - i * 0.02f;
        glColor4f(1.0f, 0.8f, 0.3f, alpha);
        glBegin(GL_LINE_LOOP);
        for (int j = 0; j < 48; ++j) {
            float angle = 2.0f * PI * j / 48;
            glVertex3f(radius * cosf(angle), radius * sinf(angle), 0.0f);
        }
        glEnd();
    }

    glEnable(GL_LIGHTING);

    // مواد الشمس مع تحسين
    GLfloat sun_emission[] = { 1.0f, 0.9f, 0.5f, 1.0f };
    GLfloat sun_diffuse[] = { 1.0f, 0.7f, 0.2f, 1.0f };
    GLfloat sun_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat sun_shininess[] = { 100.0f };

    glMaterialfv(GL_FRONT, GL_EMISSION, sun_emission);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, sun_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, sun_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, sun_shininess);

    // رسم الشمس مع تفاصيل أكثر
    glutSolidSphere(3.8f, 60, 50);

    // إضافة تفاصيل سطح الشمس (بقع شمسية)
    glDisable(GL_LIGHTING);
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    glColor3f(0.7f, 0.4f, 0.1f);
    for (int i = 0; i < 20; ++i) {
        float theta = (rand() / (float)RAND_MAX) * 2.0f * PI;
        float phi = acosf(2.0f * (rand() / (float)RAND_MAX) - 1.0f);
        float r = 3.8f;
        float x = r * sinf(phi) * cosf(theta);
        float y = r * sinf(phi) * sinf(theta);
        float z = r * cosf(phi);
        glVertex3f(x, y, z);
    }
    glEnd();
    glEnable(GL_LIGHTING);

    GLfloat no_emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    glMaterialfv(GL_FRONT, GL_EMISSION, no_emission);

    glPopMatrix();
}

void drawPlanet(const PlanetInfo& planet) {
    glPushMatrix();

    float orbitAngle = yearAngle * planet.orbit_speed * 0.001f;
    float trueAnomaly = orbitAngle + planet.orbit_offset;

    float r = planet.orbit_radius * (1.0f - planet.eccentricity * planet.eccentricity)
        / (1.0f + planet.eccentricity * cosf(trueAnomaly));

    // تطبيق الميل المداري
    glRotatef(planet.inclination, 0.0f, 0.0f, 1.0f);
    glRotatef(trueAnomaly * 180.0f / PI, 0.0f, 1.0f, 0.0f);
    glTranslatef(r, 0.0f, 0.0f);

    // رسم المدار
    if (planet.name != "Earth") {
        drawOrbit(planet);
    }

    // تطبيق ميلان المحور
    glRotatef(planet.axial_tilt, 0.0f, 0.0f, 1.0f);

    // دوران الكوكب
    glRotatef(dayAngle * planet.rotation_speed * 0.1f, 0.0f, 1.0f, 0.0f);

    // مواد الكوكب
    GLfloat mat_ambient[] = { planet.r * 0.2f, planet.g * 0.2f, planet.b * 0.2f, 1.0f };
    GLfloat mat_diffuse[] = { planet.r, planet.g, planet.b, 1.0f };
    GLfloat mat_specular[] = { 0.3f, 0.3f, 0.3f, 1.0f };
    GLfloat mat_shininess[] = { 30.0f };

    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);

    // رسم الكوكب
    glutSolidSphere(planet.planet_radius, 40, 30);

    // حلقات الكواكب
    if (planet.has_rings) {
        glPushMatrix();
        glRotatef(25.0f, 1.0f, 0.0f, 0.0f);

        glDisable(GL_LIGHTING);
        glColor3f(0.9f, 0.85f, 0.7f);

        glBegin(GL_TRIANGLE_STRIP);
        const int ring_segments = 120;
        for (int i = 0; i <= ring_segments; ++i) {
            float angle = 2.0f * PI * i / ring_segments;
            float cos_a = cosf(angle);
            float sin_a = sinf(angle);

            glVertex3f(planet.ring_outer * cos_a, 0.0f, planet.ring_outer * sin_a);
            glVertex3f(planet.ring_inner * cos_a, 0.0f, planet.ring_inner * sin_a);
        }
        glEnd();
        glEnable(GL_LIGHTING);

        glPopMatrix();
    }

    // اسم الكوكب
    drawPlanetName(0, planet.planet_radius, 0, planet.name, planet.planet_radius);

    glPopMatrix();
}

void drawMoon(float distance, float size, float speed) {
    glPushMatrix();

    glRotatef(yearAngle * speed * 0.01f, 0.0f, 1.0f, 0.0f);
    glTranslatef(distance, 0.0f, 0.0f);

    GLfloat moon_mat[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    glMaterialfv(GL_FRONT, GL_DIFFUSE, moon_mat);

    glutSolidSphere(size, 24, 20);

    glPopMatrix();
}

void drawEarthWithMoon() {
    PlanetInfo earth;
    for (const auto& p : planets) {
        if (p.name == "Earth") {
            earth = p;
            break;
        }
    }

    glPushMatrix();

    float orbitAngle = yearAngle * earth.orbit_speed * 0.001f;
    float trueAnomaly = orbitAngle + earth.orbit_offset;
    float r = earth.orbit_radius * (1.0f - earth.eccentricity * earth.eccentricity)
        / (1.0f + earth.eccentricity * cosf(trueAnomaly));

    glRotatef(earth.inclination, 0.0f, 0.0f, 1.0f);
    glRotatef(trueAnomaly * 180.0f / PI, 0.0f, 1.0f, 0.0f);
    glTranslatef(r, 0.0f, 0.0f);

    // مدار الأرض
    drawOrbit(earth);

    glRotatef(earth.axial_tilt, 0.0f, 0.0f, 1.0f);
    glRotatef(dayAngle * earth.rotation_speed * 0.1f, 0.0f, 1.0f, 0.0f);

    GLfloat earth_ambient[] = { 0.1f, 0.2f, 0.3f, 1.0f };
    GLfloat earth_diffuse[] = { 0.3f, 0.5f, 0.9f, 1.0f };
    GLfloat earth_specular[] = { 0.3f, 0.3f, 0.3f, 1.0f };
    GLfloat earth_shininess[] = { 25.0f };

    glMaterialfv(GL_FRONT, GL_AMBIENT, earth_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, earth_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, earth_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, earth_shininess);

    glutSolidSphere(earth.planet_radius, 40, 30);

    glPopMatrix();

    // القمر
    glPushMatrix();
    glRotatef(earth.inclination, 0.0f, 0.0f, 1.0f);
    glRotatef(trueAnomaly * 180.0f / PI, 0.0f, 1.0f, 0.0f);
    glTranslatef(r, 0.0f, 0.0f);
    drawMoon(2.5f, 0.27f, 13.0f);
    glPopMatrix();

    // اسم الأرض
    glPushMatrix();
    glRotatef(earth.inclination, 0.0f, 0.0f, 1.0f);
    glRotatef(trueAnomaly * 180.0f / PI, 0.0f, 1.0f, 0.0f);
    glTranslatef(r, 0.0f, 0.0f);
    drawPlanetName(0, earth.planet_radius, 0, earth.name, earth.planet_radius);
    glPopMatrix();
}

void drawHUD() {
    glDisable(GL_LIGHTING);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    int width = glutGet(GLUT_WINDOW_WIDTH);
    int height = glutGet(GLUT_WINDOW_HEIGHT);
    gluOrtho2D(0, width, 0, height);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glColor4f(0.0f, 0.0f, 0.1f, 0.7f);
    glBegin(GL_QUADS);
    glVertex2f(10, height - 10);
    glVertex2f(300, height - 10);
    glVertex2f(300, height - 140);
    glVertex2f(10, height - 140);
    glEnd();

    auto drawText = [&](int x, int y, const char* text) {
        glRasterPos2i(x, y);
        for (const char* c = text; *c != '\0'; ++c) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
        }
        };

    glColor3f(1.0f, 1.0f, 1.0f);
    drawText(20, height - 30, "Solar System Simulation");
    drawText(20, height - 50, "----------------------");

    std::string status = paused ? "Status: PAUSED" : "Status: RUNNING";
    drawText(20, height - 70, status.c_str());

    char speed_str[50];
    sprintf_s(speed_str, sizeof(speed_str), "Speed: x%.1f", speed_scale);
    drawText(20, height - 90, speed_str);

    drawText(20, height - 110, "Controls:");
    drawText(20, height - 130, "P=Pause +/-=Speed O=Orbits L=Labels");

    glColor3f(0.8f, 0.8f, 0.8f);
    drawText(10, 30, "Mouse: Drag=Rotate Scroll=Zoom");
    drawText(10, 15, "Arrows=Rotate PgUp/PgDn=Zoom ESC=Exit");

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glEnable(GL_LIGHTING);
}

void computeCamera(float& ex, float& ey, float& ez) {
    float rx = rot_x * PI / 180.0f;
    float ry = rot_y * PI / 180.0f;
    float proj = distance_cam * cosf(rx);
    ey = distance_cam * sinf(rx);
    ex = proj * sinf(ry);
    ez = proj * cosf(ry);
}

void specialKeys(int key, int x, int y) {
    switch (key) {
    case GLUT_KEY_LEFT: rot_y -= 3.0f; break;
    case GLUT_KEY_RIGHT: rot_y += 3.0f; break;
    case GLUT_KEY_UP: rot_x += 3.0f; if (rot_x > 89.0f) rot_x = 89.0f; break;
    case GLUT_KEY_DOWN: rot_x -= 3.0f; if (rot_x < -89.0f) rot_x = -89.0f; break;
    case GLUT_KEY_PAGE_UP: distance_cam -= 5.0f; if (distance_cam < 10.0f) distance_cam = 10.0f; break;
    case GLUT_KEY_PAGE_DOWN: distance_cam += 5.0f; if (distance_cam > 300.0f) distance_cam = 300.0f; break;
    }
    glutPostRedisplay();
}

void mouseFunc(int button, int state, int x, int y) {
    if (button == 3 || button == 4) {
        if (state == GLUT_DOWN) {
            distance_cam += (button == 3) ? -5.0f : 5.0f;
            distance_cam = std::max(10.0f, std::min(300.0f, distance_cam));
            glutPostRedisplay();
        }
        return;
    }

    if (state == GLUT_DOWN) {
        mouse_button = button;
        last_mouse_x = x;
        last_mouse_y = y;
    }
    else {
        mouse_button = -1;
    }
}

void motionFunc(int x, int y) {
    if (mouse_button == GLUT_LEFT_BUTTON) {
        int dx = x - last_mouse_x;
        int dy = y - last_mouse_y;
        rot_y += dx * 0.5f;
        rot_x += dy * 0.5f;
        if (rot_x > 89.0f) rot_x = 89.0f;
        if (rot_x < -89.0f) rot_x = -89.0f;
        last_mouse_x = x;
        last_mouse_y = y;
        glutPostRedisplay();
    }
}

void keyboardFunc(unsigned char key, int x, int y) {
    switch (key) {
    case 'p': case 'P': paused = !paused; break;
    case 'o': case 'O': showOrbits = !showOrbits; break;
    case 'l': case 'L': showLabels = !showLabels; break;
    case 'g': case 'G': showGrid = !showGrid; break;
    case '+': case '=': speed_scale *= 1.2f; if (speed_scale > 100.0f) speed_scale = 100.0f; break;
    case '-': case '_': speed_scale /= 1.2f; if (speed_scale < 0.01f) speed_scale = 0.01f; break;
    case 'r': case 'R': rot_x = 25.0f; rot_y = -45.0f; distance_cam = 80.0f; speed_scale = 1.0f; break;
    case 27: exit(0); break;
    }
    glutPostRedisplay();
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    float ex, ey, ez;
    computeCamera(ex, ey, ez);
    gluLookAt(ex, ey, ez, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);

    GLfloat light_position[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    GLfloat light_diffuse[] = { 1.0f, 0.95f, 0.9f, 1.0f };
    GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat light_ambient[] = { 0.1f, 0.1f, 0.1f, 1.0f };

    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);

    drawStars();
    drawGrid();
    drawSun();

    for (const auto& planet : planets) {
        if (planet.name == "Earth") {
            drawEarthWithMoon();
        }
        else {
            drawPlanet(planet);
        }
    }

    drawHUD();
    glutSwapBuffers();
}

void idle() {
    if (!paused) {
        yearAngle += 0.5f * speed_scale;
        dayAngle += 2.0f * speed_scale;
        if (yearAngle > 36000.0f) yearAngle -= 36000.0f;
        if (dayAngle > 36000.0f) dayAngle -= 36000.0f;
        glutPostRedisplay();
    }
}

void init() {
    glClearColor(0.0f, 0.0f, 0.05f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);

    generateStars();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0f, 16.0f / 9.0f, 0.1f, 1000.0f);
    glMatrixMode(GL_MODELVIEW);
}

void reshape(int width, int height) {
    if (height == 0) height = 1;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0f, (GLfloat)width / (GLfloat)height, 0.1f, 1000.0f);
    glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1280, 720);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Solar System 3D Simulation - Professional Edition");

    init();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glutKeyboardFunc(keyboardFunc);
    glutSpecialFunc(specialKeys);
    glutMouseFunc(mouseFunc);
    glutMotionFunc(motionFunc);

    glutMainLoop();
    return 0;
}