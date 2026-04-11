#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <queue>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>

class PlannerNode : public rclcpp::Node
{
public:
    PlannerNode() : rclcpp::Node("planner_node")
    {
        RCLCPP_INFO(this->get_logger(), "Planner Node Started");

        // ── True world goal — fixed, NEVER replaced with interim targets ──
        goal_world_x_ = 100.0;
        goal_world_y_ = 50.0;

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/inflated_map", 10,
            std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&PlannerNode::imuCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        cmd_pub_  = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PlannerNode::controlLoop, this));

        replan_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2000),
            std::bind(&PlannerNode::triggerReplan, this));
    }

private:
    // ─────────────────── CONFIG ───────────────────────────────
    static constexpr double MAX_LINEAR      = 0.6;   // m/s hard cap
    static constexpr double WAYPOINT_THRESH = 0.25;  // m
    static constexpr double GOAL_BOX        = 5.0;   // ±5 m in x AND y
    static constexpr double LOOKAHEAD_DIST  = 1.2;   // m
    static constexpr int    OBSTACLE_THRESH = 50;
    static constexpr int    MAX_FAILURES    = 5;

    // ─────────────────── STATE ────────────────────────────────
    double goal_world_x_, goal_world_y_;

    nav_msgs::msg::OccupancyGrid map_;
    bool map_received_   = false;
    bool odom_received_  = false;   // ← guard: don't check goal until odom valid
    bool path_planned_   = false;
    bool goal_reached_   = false;
    bool need_replan_    = false;
    bool fallback_drive_ = false;

    double robot_x_ = 0.0, robot_y_ = 0.0;
    double yaw_     = 0.0;

    std::vector<std::pair<int,int>> global_path_;
    size_t path_index_        = 0;
    int    consecutive_fails_ = 0;

    // ─────────────────── ROS ──────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr replan_timer_;

    // ─────────────────── CALLBACKS ────────────────────────────

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_          = *msg;
        map_received_ = true;
        need_replan_  = true;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "Map received: %d x %d, res=%.3f  origin=(%.2f,%.2f)",
            map_.info.width, map_.info.height, map_.info.resolution,
            map_.info.origin.position.x, map_.info.origin.position.y);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_      = msg->pose.pose.position.x;
        robot_y_      = msg->pose.pose.position.y;
        odom_received_ = true;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double siny = 2.0 * (msg->orientation.w * msg->orientation.z +
                             msg->orientation.x * msg->orientation.y);
        double cosy = 1.0 - 2.0 * (msg->orientation.y * msg->orientation.y +
                                    msg->orientation.z * msg->orientation.z);
        yaw_ = std::atan2(siny, cosy);
    }

    void triggerReplan()
    {
        if (map_received_ && odom_received_ && !goal_reached_)
            need_replan_ = true;
    }

    // ─────────────────── GOAL CHECK ───────────────────────────
    //
    // Completely independent of the map, A*, or path endpoint.
    // Uses raw world coordinates from odom vs the fixed true goal.
    // Fires as soon as robot is within ±GOAL_BOX in BOTH x and y.

    bool goalReached()
    {
        if (!odom_received_) return false;   // no pose yet — can't be there

        double ex = std::abs(robot_x_*20 - goal_world_x_);
        double ey = std::abs(robot_y_*20 - goal_world_y_);

        // Log proximity every second so we can see it closing in
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(), 1000,
            "Goal check: robot=(%.2f,%.2f)  goal=(%.1f,%.1f)  "
            "err_x=%.2f err_y=%.2f  box=±%.1f  %s",
            robot_x_, robot_y_,
            goal_world_x_, goal_world_y_,
            ex, ey, GOAL_BOX,
            (ex <= GOAL_BOX && ey <= GOAL_BOX) ? ">>> REACHED <<<" : "not yet");

        return (ex <= GOAL_BOX && ey <= GOAL_BOX);
    }

    // ─────────────────── A* GOAL CELL ─────────────────────────
    //
    // goal_world_x_/y_ is NEVER written to after construction.
    //
    // Returns which MAP CELL to pass to A* this cycle:
    //   • Goal inside map  → that cell (BFS-snap if obstacle).
    //   • Goal outside map → furthest FREE cell on the ray robot→goal.
    //
    // Either way the robot drives toward the true goal.
    // goalReached() catches arrival independently of this cell.

    std::pair<int,int> computeAstarGoal()
    {
        double ox  = map_.info.origin.position.x;
        double oy  = map_.info.origin.position.y;
        double res = map_.info.resolution;
        int W = (int)map_.info.width;
        int H = (int)map_.info.height;

        double gx_f = (goal_world_x_ - ox) / res;
        double gy_f = (goal_world_y_ - oy) / res;

        RCLCPP_DEBUG(this->get_logger(),
            "Goal world=(%.1f,%.1f) → cell=(%.1f,%.1f)  map WxH=%dx%d",
            goal_world_x_, goal_world_y_, gx_f, gy_f, W, H);

        // ── Case A: goal is inside current map ────────────────
        if (gx_f >= 0.0 && gx_f < (double)W &&
            gy_f >= 0.0 && gy_f < (double)H)
        {
            int gx = (int)gx_f;
            int gy = (int)gy_f;
            int val = map_.data[gy * W + gx];

            RCLCPP_INFO(this->get_logger(),
                "Goal cell (%d,%d) val=%d %s",
                gx, gy, val,
                val > OBSTACLE_THRESH ? "BLOCKED→BFS snap" : "free");

            if (val <= OBSTACLE_THRESH)
                return {gx, gy};

            // Goal cell blocked — BFS to nearest free cell
            return bfsNearestFree(gx, gy);
        }

        // ── Case B: goal is outside current map ───────────────
        // Walk ray from robot toward true goal at 0.5-cell steps.
        // Pick the last FREE cell before leaving the map edge.
        int sx = std::clamp((int)((robot_x_ - ox) / res), 0, W - 1);
        int sy = std::clamp((int)((robot_y_ - oy) / res), 0, H - 1);

        double dx  = gx_f - sx;
        double dy  = gy_f - sy;
        double len = std::hypot(dx, dy);
        if (len < 1e-6) return {sx, sy};
        dx /= len;
        dy /= len;

        std::pair<int,int> last_free = {sx, sy};

        for (double t = 0.5; t <= len + 1.0; t += 0.5)
        {
            int cx = sx + (int)std::round(dx * t);
            int cy = sy + (int)std::round(dy * t);
            if (cx < 0 || cy < 0 || cx >= W || cy >= H) break;
            if (map_.data[cy * W + cx] <= OBSTACLE_THRESH)
                last_free = {cx, cy};
        }

        RCLCPP_INFO(this->get_logger(),
            "True goal (%.1f,%.1f) outside map — A* endpoint: (%d,%d)",
            goal_world_x_, goal_world_y_, last_free.first, last_free.second);

        return {goal_world_x_, goal_world_y_};
    }

    std::pair<int,int> bfsNearestFree(int gx, int gy)
    {
        int W = (int)map_.info.width;
        int H = (int)map_.info.height;
        auto idx    = [&](int x, int y){ return y * W + x; };
        auto isFree = [&](int x, int y) -> bool {
            if (x < 0 || y < 0 || x >= W || y >= H) return false;
            return map_.data[idx(x, y)] <= OBSTACLE_THRESH;
        };

        if (isFree(gx, gy)) return {gx, gy};

        std::queue<std::pair<int,int>> q;
        std::unordered_map<int,bool>   seen;
        q.push({gx, gy});
        seen[idx(gx, gy)] = true;

        const std::vector<std::pair<int,int>> d4 = {{1,0},{-1,0},{0,1},{0,-1}};
        while (!q.empty())
        {
            auto [cx, cy] = q.front(); q.pop();
            for (auto &[ddx, ddy] : d4)
            {
                int nx = cx + ddx, ny = cy + ddy;
                if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                int i = idx(nx, ny);
                if (seen.count(i)) continue;
                seen[i] = true;
                if (isFree(nx, ny)) return {nx, ny};
                q.push({nx, ny});
            }
        }
        return {gx, gy};
    }

    // ─────────────────── PLANNING ─────────────────────────────

    void planPath()
    {
        if (!map_received_ || !odom_received_) return;
        need_replan_ = false;

        double ox  = map_.info.origin.position.x;
        double oy  = map_.info.origin.position.y;
        double res = map_.info.resolution;
        int W = (int)map_.info.width;
        int H = (int)map_.info.height;

        int sx = std::clamp((int)((robot_x_ - ox) / res), 0, W - 1);
        int sy = std::clamp((int)((robot_y_ - oy) / res), 0, H - 1);

        auto [gx, gy] = computeAstarGoal();

        int sv = map_.data[sy * W + sx];
        int gv = map_.data[gy * W + gx];
        RCLCPP_INFO(this->get_logger(),
            "A* (%d,%d)[v=%d] → (%d,%d)[v=%d]  true goal=(%.1f,%.1f)",
            sx, sy, sv, gx, gy, gv, goal_world_x_, goal_world_y_);

        auto new_path = astar(sx, sy, gx, gy);

        if (new_path.empty())
        {
            consecutive_fails_++;
            RCLCPP_WARN(this->get_logger(),
                "No path found (%d/%d)", consecutive_fails_, MAX_FAILURES);
            if (consecutive_fails_ >= MAX_FAILURES)
            {
                RCLCPP_WARN(this->get_logger(),
                    "Fallback: direct drive toward TRUE goal (%.1f,%.1f)",
                    goal_world_x_, goal_world_y_);
                fallback_drive_ = true;
            }
            return;
        }

        consecutive_fails_ = 0;
        fallback_drive_    = false;
        global_path_       = new_path;
        path_index_        = nearestPathIndex();
        path_planned_      = true;

        publishPath(global_path_);
        RCLCPP_INFO(this->get_logger(),
            "Path planned: %ld cells (resuming index %ld)",
            global_path_.size(), path_index_);
    }

    size_t nearestPathIndex()
    {
        double ox  = map_.info.origin.position.x;
        double oy  = map_.info.origin.position.y;
        double res = map_.info.resolution;

        double best_d = 1e9;
        size_t best_i = 0;
        for (size_t i = 0; i < global_path_.size(); i++)
        {
            double wx = global_path_[i].first  * res + ox;
            double wy = global_path_[i].second * res + oy;
            double d  = std::hypot(wx - robot_x_, wy - robot_y_);
            if (d < best_d) { best_d = d; best_i = i; }
        }
        return best_i;
    }

    // ─────────────────── A* ───────────────────────────────────

    struct ANode {
        int x, y;
        float cost, priority;
        bool operator<(const ANode &o) const { return priority > o.priority; }
    };

    std::vector<std::pair<int,int>> astar(int sx, int sy, int gx, int gy)
    {
        int W = (int)map_.info.width;
        int H = (int)map_.info.height;
        auto index = [&](int x, int y){ return y * W + x; };

        std::priority_queue<ANode> open;
        std::unordered_map<int, float> cost;
        std::unordered_map<int, std::pair<int,int>> parent;
        std::unordered_map<int, bool> visited;

        open.push({sx, sy, 0.0f, 0.0f});
        cost[index(sx, sy)] = 0.0f;

        const std::vector<std::tuple<int,int,float>> dirs = {
            {1, 0, 1.000f}, {-1, 0, 1.000f}, {0, 1, 1.000f}, {0,-1, 1.000f},
            {1, 1, 1.414f}, {-1, 1, 1.414f}, {1,-1, 1.414f}, {-1,-1, 1.414f}
        };

        while (!open.empty())
        {
            ANode cur = open.top(); open.pop();
            int cur_id = index(cur.x, cur.y);
            if (visited[cur_id]) continue;
            visited[cur_id] = true;

            if (cur.x == gx && cur.y == gy) break;

            for (auto &[ddx, ddy, step] : dirs)
            {
                int nx = cur.x + ddx;
                int ny = cur.y + ddy;
                if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;

                int idx = index(nx, ny);
                int val = map_.data[idx];

                // -1 (unknown) → free (cost same as clear). Only >50 blocked.
                if (val > OBSTACLE_THRESH) continue;

                float cell_cost = (val < 0)
                    ? 1.0f
                    : 1.0f + (val / (float)OBSTACLE_THRESH) * 4.0f;

                float new_cost = cur.cost + step * cell_cost;
                if (!cost.count(idx) || new_cost < cost[idx])
                {
                    cost[idx] = new_cost;
                    float h   = std::hypot(gx - nx, gy - ny);
                    open.push({nx, ny, new_cost, new_cost + h});
                    parent[idx] = {cur.x, cur.y};
                }
            }
        }

        std::vector<std::pair<int,int>> path;
        if (!parent.count(index(gx, gy)) && !(gx == sx && gy == sy))
        {
            RCLCPP_WARN(this->get_logger(),
                "A* could not connect (%d,%d)→(%d,%d)", sx, sy, gx, gy);
            return path;
        }

        int cx = gx, cy = gy;
        while (!(cx == sx && cy == sy))
        {
            path.push_back({cx, cy});
            auto it = parent.find(index(cx, cy));
            if (it == parent.end()) break;
            cx = it->second.first;
            cy = it->second.second;
        }
        path.push_back({sx, sy});
        std::reverse(path.begin(), path.end());
        return path;
    }

    // ─────────────────── OBSTACLE CHECK ───────────────────────

    bool obstacleAhead()
    {
        if (!map_received_ || global_path_.empty()) return false;

        int W = (int)map_.info.width;
        int H = (int)map_.info.height;
        size_t lookahead = (size_t)(LOOKAHEAD_DIST / map_.info.resolution);

        for (size_t i = path_index_;
             i < global_path_.size() && i < path_index_ + lookahead; i++)
        {
            int cx = global_path_[i].first;
            int cy = global_path_[i].second;
            if (cx < 0 || cy < 0 || cx >= W || cy >= H) continue;
            int val = map_.data[cy * W + cx];
            if (val > OBSTACLE_THRESH)
            {
                RCLCPP_WARN(this->get_logger(),
                    "Obstacle (val=%d) at (%d,%d) — replanning", val, cx, cy);
                return true;
            }
        }
        return false;
    }

    // ─────────────────── CONTROL LOOP ─────────────────────────

    void controlLoop()
    {
        // ── GOAL CHECK: runs every tick, uses only odom + true goal ──
        // Completely independent of map, A*, or where the path ends.
        // This is the ONLY place goal_reached_ is set to true.
        if (!goal_reached_ && goalReached())
        {
            stopRobot();
            goal_reached_ = true;
            RCLCPP_INFO(this->get_logger(),
                "*** TRUE GOAL (%.1f,%.1f) REACHED ***  "
                "robot=(%.2f,%.2f)  within ±%.1f m box",
                goal_world_x_, goal_world_y_,
                robot_x_, robot_y_, GOAL_BOX);
            return;
        }

        if (goal_reached_ || !map_received_ || !odom_received_) return;

        if (fallback_drive_)
        {
            directDriveTowardGoal();
            return;
        }

        if (need_replan_ || obstacleAhead())
        {
            planPath();
            return;
        }

        if (!path_planned_ || global_path_.empty()) return;

        // Advance waypoints already within reach
        while (path_index_ < global_path_.size())
        {
            double ox  = map_.info.origin.position.x;
            double oy  = map_.info.origin.position.y;
            double res = map_.info.resolution;
            double wx  = global_path_[path_index_].first  * res + ox;
            double wy  = global_path_[path_index_].second * res + oy;
            if (std::hypot(wx - robot_x_, wy - robot_y_) > WAYPOINT_THRESH)
                break;
            path_index_++;
        }

        if (path_index_ >= global_path_.size())
        {
            RCLCPP_INFO(this->get_logger(),
                "Segment done — replanning toward true goal (%.1f,%.1f)",
                goal_world_x_, goal_world_y_);
            need_replan_ = true;
            return;
        }

        // Pure pursuit
        double ox  = map_.info.origin.position.x;
        double oy  = map_.info.origin.position.y;
        double res = map_.info.resolution;

        double tx = global_path_[path_index_].first  * res + ox;
        double ty = global_path_[path_index_].second * res + oy;

        double dx   = tx - robot_x_;
        double dy   = ty - robot_y_;
        double dist = std::hypot(dx, dy);

        double angle_to_wp = std::atan2(dy, dx);
        double heading_err = std::atan2(
            std::sin(angle_to_wp - yaw_),
            std::cos(angle_to_wp - yaw_));

        double turn_factor = std::max(0.1, 1.0 - std::abs(heading_err) / M_PI);
        double linear_vel  = std::min(MAX_LINEAR, dist) * turn_factor;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = linear_vel;
        cmd.angular.z = 2.5 * heading_err;
        cmd_pub_->publish(cmd);
    }

    void directDriveTowardGoal()
    {
        // goalReached() already checked at top of controlLoop
        double dx   = goal_world_x_ - robot_x_;
        double dy   = goal_world_y_ - robot_y_;
        double dist = std::hypot(dx, dy);

        double angle = std::atan2(dy, dx);
        double err   = std::atan2(
            std::sin(angle - yaw_),
            std::cos(angle - yaw_));

        double turn_factor = std::max(0.1, 1.0 - std::abs(err) / M_PI);
        double speed       = std::min(MAX_LINEAR * 0.5, dist) * turn_factor;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = speed;
        cmd.angular.z = 2.5 * err;
        cmd_pub_->publish(cmd);
    }

    void stopRobot()
    {
        cmd_pub_->publish(geometry_msgs::msg::Twist{});
    }

    // ─────────────────── PUBLISH PATH ─────────────────────────

    void publishPath(const std::vector<std::pair<int,int>> &path)
    {
        nav_msgs::msg::Path msg;
        msg.header.frame_id = "map";
        msg.header.stamp    = this->now();

        double ox  = map_.info.origin.position.x;
        double oy  = map_.info.origin.position.y;
        double res = map_.info.resolution;

        for (auto &p : path)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = msg.header;
            pose.pose.position.x    = p.first  * res + ox;
            pose.pose.position.y    = p.second * res + oy;
            pose.pose.orientation.w = 1.0;
            msg.poses.push_back(pose);
        }

        path_pub_->publish(msg);
    }
};

// ─────────────────── MAIN ─────────────────────────────────────

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}