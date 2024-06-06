#include <GL/freeglut_std.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <random>
#include <utility>
#include <vector>
#include <iomanip>

#include "obs_definition.cpp"

void boundary_reset();
void draw_arrows(int);
void background();

// Init values and constants
float PI = 3.1416;
vertex center(-1.5, 0.0); // circle's center
GLfloat radius = 0.06;    // circle's radius
int triangleAmount = 40;
GLfloat twicePi = 2.0 * PI;

float MOVING_UNIT = 0.06;
float FALLING_VEL = 0.01;

// Boundaries
float MAX_X = 2.0, MIN_X = -2.0;
float MAX_Y = 2.0, MIN_Y = -2.0;
float COLLISION_RAD = 1.0;

// Rollback variables
float rollback_x = center.first;
float rollback_y = center.second;
int last_key = 0;

// Obstacle def
obstacles obs = createObs();
polygon centroids = centroid_calculation(obs);
obstacles valid_obs = island_definition(center, obs, centroids, COLLISION_RAD);
bool collision = SAT_collision_circle(center, radius, valid_obs);

// RNG settings
std::random_device rd{};
std::mt19937 engine{rd()};
std::uniform_real_distribution<float> dist{0.0, 1.0};

void initGL() { glClearColor(0.0f, 0.0f, 0.0f, 1.0f); }

void display() {
  glClear(GL_COLOR_BUFFER_BIT);

  // Using iterators to traverse obstacle's structures
  obstacles::iterator o_itr;
  polygon::iterator p_itr;

  // Reset circle's center in case of an out-of-boundary situation
  boundary_reset();

  // Draw background and arrow keys
  background();
  draw_arrows(last_key);

  float pos_x = center.first;
  float pos_y = center.second;

  for (o_itr = obs.begin(); o_itr != obs.end(); ++o_itr) {

    glBegin(GL_POLYGON);
    glColor3f(0.21f, 0.01f, 0.29f);

    for (p_itr = (*o_itr).begin(); p_itr != (*o_itr).end(); ++p_itr) {
      glVertex2f((GLfloat)(*p_itr).first, (GLfloat)(*p_itr).second);
    }

    glEnd();

    glBegin(GL_TRIANGLE_FAN);
    if (collision) {
      glColor3ub(238, 50, 5);
    } else {
      glColor3ub(238, 139, 21);
    }

    glVertex2f(pos_x, pos_y); // center of circle
    for (int i = 0; i <= triangleAmount; i++) {
      glVertex2f(pos_x + (radius * cos(i * twicePi / triangleAmount)),
                 pos_y + (radius * sin(i * twicePi / triangleAmount)));
    }

    glEnd();
  }

  glFlush();
}

void boundary_reset() {
  if (center.first > MAX_X - radius) {
    center.first = MAX_X - radius;
  } else if (center.first < MIN_X + radius) {
    center.first = MIN_X + radius;
  }

  if (center.second > MAX_Y - radius) {
    center.second = MAX_Y - radius;
  } else if (center.second < MIN_Y + radius) {
    center.second = MIN_Y + radius;
  }
}

void update_fall(int time) {

  rollback_y = center.second;
  center.second -= FALLING_VEL;

  valid_obs = island_definition(center, obs, centroids, COLLISION_RAD);
  collision = SAT_collision_circle(center, radius, valid_obs);

  if (collision) {
    center.second = rollback_y;
  }

  boundary_reset();

  glutPostRedisplay();
  glutTimerFunc(60, update_fall, 0);
}

void draw_arrows(int glut_key) {

  switch (glut_key) {
  case GLUT_KEY_UP:
    glBegin(GL_POLYGON);
    glColor3f(0.9f, 0.9f, 0.9f);
    glVertex2f(1.6, 1.7);
    glVertex2f(1.85, 1.7);
    glVertex2f(1.725, 1.85);
    glEnd();
    break;

  case GLUT_KEY_DOWN:
    glBegin(GL_POLYGON);
    glColor3f(0.9f, 0.9f, 0.9f);
    glVertex2f(1.6, 1.85);
    glVertex2f(1.85, 1.85);
    glVertex2f(1.725, 1.7);
    glEnd();
    break;

  case GLUT_KEY_LEFT:
    glBegin(GL_POLYGON);
    glColor3f(0.9f, 0.9f, 0.9f);
    glVertex2f(1.8, 1.85);
    glVertex2f(1.8, 1.6);
    glVertex2f(1.65, 1.725);
    glEnd();
    break;

  case GLUT_KEY_RIGHT:
    glBegin(GL_POLYGON);
    glColor3f(0.9f, 0.9f, 0.9f);
    glVertex2f(1.65, 1.85);
    glVertex2f(1.65, 1.6);
    glVertex2f(1.8, 1.725);
    glEnd();
    break;
  }
}

void move_circle(int key, int x, int y) {

  rollback_x = center.first;
  rollback_y = center.second;

  switch (key) {
  case GLUT_KEY_UP:
    center.second += MOVING_UNIT;
    break;

  case GLUT_KEY_DOWN:
    center.second -= MOVING_UNIT;
    break;

  case GLUT_KEY_RIGHT:
    center.first += MOVING_UNIT;
    break;

  case GLUT_KEY_LEFT:
    center.first -= MOVING_UNIT;
    break;
  }

  last_key = key;

  // Collision check
  valid_obs = island_definition(center, obs, centroids, COLLISION_RAD);
  collision = SAT_collision_circle(center, radius, valid_obs);
  
  std::cout << "SIZE OF VALID OBS: " << valid_obs.size() << "\n";
  if (collision) {
    center.first = rollback_x;
    center.second = rollback_y;
  }

  std::cout<<std::setprecision(2);
  std::cout << center.first << " " << center.second << "\n";
  glutPostRedisplay();
}

void background() {
  glBegin(GL_POLYGON);
  glColor3f(0.0f, 0.0f, 0.0f);
  glVertex2f(-2.0, -2.0);
  glVertex2f(2.0, -2.0);
  glVertex2f(2.0, 2.0);
  glVertex2f(-2.0, 2.0);
  glEnd();
}

int main(int argc, char **argv) {
  glutInit(&argc, argv);
  glutCreateWindow("Collision Detection Test");
  glutInitWindowSize(800, 800);
  gluOrtho2D(-2, 2, -2, 2);
  glViewport(0, 0, 800, 800);
  glutDisplayFunc(display);
  initGL();
  glutSpecialFunc(move_circle);
  glutTimerFunc(0, update_fall, 0);
  glutMainLoop();
  return 0;
}
