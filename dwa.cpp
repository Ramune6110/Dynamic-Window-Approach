#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include "dwa.h"

#define PI 3.141592653

using namespace std;
using namespace Eigen;

using Traj     = vector<array<float, 5>>;
using Obstacle = vector<array<float, 2>>;
using State    = array<float, 5>;
using Window   = array<float, 4>;
using Point    = array<float, 2>;
using Control  = array<float, 2>;

Dynamic_Window_Approach::Dynamic_Window_Approach() 
{
    // Time
    ts = 0.0;
    dt = 0.1;
    tf = 50.0;
    // DWA
    // Dynamics
    max_speed         = 1.0;
    min_speed         = -0.5;
    max_yawrate       = 40.0 * PI / 180.0;
    max_accel         = 0.2;
    // Robot 
    robot_radius      = 1.0;
    max_dyawrate      = 40.0 * PI / 180.0;
    v_reso            = 0.01;
    yawrate_reso      = 0.1 * PI / 180.0;
    // DWA calculation
    predict_time      = 3.0;
    to_goal_cost_gain = 1.0;
    speed_cost_gain   = 1.0;
}

Dynamic_Window_Approach::~Dynamic_Window_Approach()
{
    cout << "Finish" << endl;
}

State Dynamic_Window_Approach::motion(State x, Control u) 
{
  x[2] += u[1] * dt;
  x[0] += u[0] * cos(x[2]) * dt;
  x[1] += u[0] * sin(x[2]) * dt;
  x[3] = u[0];
  x[4] = u[1];

  return x;
}

Window Dynamic_Window_Approach::calc_dynamic_window(State x)
{
  return {{
    max((x[3] - max_accel * dt), min_speed),
    min((x[3] + max_accel * dt), max_speed),
    max((x[4] - max_dyawrate * dt), -max_yawrate),
    min((x[4] + max_dyawrate * dt), max_yawrate)
  }};
}

Traj Dynamic_Window_Approach::calc_trajectory(State x, float v, float y)
{
  Traj traj;
  traj.push_back(x);
  float time = 0.0;
  while (time <= predict_time) {
    x = motion(x, array<float, 2>{{v, y}});
    traj.push_back(x);
    time += dt;
  }

  return traj;
}

float Dynamic_Window_Approach::calc_obstacle_cost(Traj traj, Obstacle ob)
{
  // calc obstacle cost inf: collistion, 0:free
  int skip_n = 2;
  float minr = numeric_limits<float>::max();

  for (unsigned int ii = 0; ii < traj.size(); ii += skip_n) {
    for (unsigned int i=0; i< ob.size(); i++) {
      float ox = ob[i][0];
      float oy = ob[i][1];
      float dx = traj[ii][0] - ox;
      float dy = traj[ii][1] - oy;

      float r = sqrt(dx*dx + dy*dy);
      if (r <= robot_radius) {
          return numeric_limits<float>::max();
      }
      if (minr >= r){
          minr = r;
      }
    }
  }

  return 1.0 / minr;
}

float Dynamic_Window_Approach::calc_to_goal_cost(Traj traj, Point goal)
{
  float goal_magnitude = sqrt(goal[0] * goal[0] + goal[1] * goal[1]);
  float traj_magnitude = sqrt(pow(traj.back()[0], 2) + pow(traj.back()[1], 2));
  float dot_product    = (goal[0] * traj.back()[0]) + (goal[1] * traj.back()[1]);
  float error          = dot_product / (goal_magnitude * traj_magnitude);
  float error_angle    = acos(error);
  float cost           = to_goal_cost_gain * error_angle;

  return cost;
}

Traj Dynamic_Window_Approach::calc_final_input(State x, Control& u, Window dw, Point goal, vector<array<float, 2>>ob) 
{
    float min_cost = 10000.0;
    Control min_u  = u;
    min_u[0]       = 0.0;
    Traj best_traj;

    // evalucate all trajectory with sampled input in dynamic window
    for (float v = dw[0]; v <= dw[1]; v += v_reso){
      for (float y = dw[2]; y <= dw[3]; y += yawrate_reso){

        Traj traj = calc_trajectory(x, v, y);

        float to_goal_cost = calc_to_goal_cost(traj, goal);
        float speed_cost   = speed_cost_gain * (max_speed - traj.back()[3]);
        float ob_cost      = calc_obstacle_cost(traj, ob);
        float final_cost   = to_goal_cost + speed_cost + ob_cost;

        if (min_cost >= final_cost){
          min_cost = final_cost;
          min_u = Control{{v, y}};
          best_traj = traj;
        }
      }
    }
    u = min_u;
    return best_traj;
}

Traj Dynamic_Window_Approach::dwa_control(State x, Control & u, Point goal, Obstacle ob) 
{
    // # Dynamic Window control
    Window dw = calc_dynamic_window(x);
    Traj traj = calc_final_input(x, u, dw, goal, ob);

    return u, traj;
}

void Dynamic_Window_Approach::simulation()
{
    // Init State
    State x({{0.0, 0.0, PI/8.0, 0.0, 0.0}});
    // Goal
    Point goal({{10.0,10.0}});
    // Obstacle
    Obstacle ob({
        {{-1, -1}},
        {{0, 2}},
        {{4.0, 2.0}},
        {{5.0, 4.0}},
        {{5.0, 5.0}},
        {{5.0, 6.0}},
        {{5.0, 9.0}},
        {{8.0, 9.0}},
        {{7.0, 9.0}},
        {{12.0, 12.0}}
    });
    // Init Input
    Control u({{0.0, 0.0}});
    // save Init
    Traj traj;
    traj.push_back(x);
    // save data
    FILE *fp;
    if ((fp = fopen("data.txt", "w")) == NULL) {
        printf("Error\n");
        exit(1);
    }
    fprintf(fp, "%lf\t%lf\t%lf\n", ts, x[0], x[1]);
    // main loop
    for(ts; ts <= tf; ts += dt){
        ts = ts + dt;
        // ------Dwnamic Window Approch --------
        Traj ltraj = dwa_control(x, u, goal, ob);
        x = motion(x, u);
        traj.push_back(x);
        // save data
        fprintf(fp, "%lf\t%lf\t%lf\n", ts, x[0], x[1]);
        // Arrive Goal!!
        if (abs(x[0] - goal[0]) < 0.5 && abs(x[1]- goal[1]) < 0.5) {
        cout << "Arrive Goal!" << endl; 
        break;
        }
    }
    // file close
    fclose(fp);
}
