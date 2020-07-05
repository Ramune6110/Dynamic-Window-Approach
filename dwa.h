#ifndef DYNAMIC_WINDOW_APPROACH
#define DYNAMIC_WINDOW_APPROACH

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

using Traj     = vector<array<float, 5>>;
using Obstacle = vector<array<float, 2>>;
using State    = array<float, 5>;
using Window   = array<float, 4>;
using Point    = array<float, 2>;
using Control  = array<float, 2>;

class Dynamic_Window_Approach 
{
    private:
        // Time
        float ts;
        float dt;
        float tf;
        // DWA
        // Dynamics
        float max_speed;
        float min_speed;
        float max_yawrate;
        float max_accel;
        // Robot
        float robot_radius;
        float max_dyawrate;
        float v_reso;
        float yawrate_reso;
        // DWA calculation
        float predict_time;
        float to_goal_cost_gain;
        float speed_cost_gain;
    private:
        State motion(State x, Control u);
        Window calc_dynamic_window(State x);
        float calc_to_goal_cost(Traj traj, Point goal);
        Traj calc_trajectory(State x, float v, float y);
        float calc_obstacle_cost(Traj traj, Obstacle ob);
        Traj dwa_control(State x, Control & u, Point goal, Obstacle ob);
        Traj calc_final_input(State x, Control& u, Window dw, Point goal, vector<array<float, 2>>ob);
    public:
        Dynamic_Window_Approach();
        ~Dynamic_Window_Approach();
        void simulation();
};

#endif // DYNAMIC_WINDOW_APPROACH

