#include <ros/ros.h>
#include <ros/package.h>
#include <algorithm>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "path_evaluator/evaluator.h"
#include "path_evaluator/metrics/smoothness_metric.h"
#include "path_evaluator/metrics/tracking_error_metric.h"

ros::Publisher path_pub;
ros::Publisher nav_goal_pub;
nav_msgs::Path path;
double total_distance = 0.0; // Biến lưu tổng quãng đường
geometry_msgs::PoseStamped last_pose; // Lưu vị trí trước đó
bool is_first_pose = true; // Để kiểm tra lần đầu tiên
bool goal_received = false; // Cờ để theo dõi khi goal được nhận
bool goal_reached = false; // Cờ để theo dõi trạng thái của mục tiêu
bool data_saved = false; // Cờ để theo dõi khi dữ liệu đã được lưu
double move_start_time = 0.0; // Thời gian bắt đầu di chuyển
int total_points = 0;
uint32_t previous_seq = 0;
double total_turning_angle_sq = 0.0;   // ∑ θ^2

// --- path_evaluator ---
// Evaluator is deliberately planner-agnostic: every published plan creates an
// episode and odometry samples are assigned to the most recently published
// episode.  This works equally for a one-shot global plan and a sub-goal
// planner that replans often.
using rmp::common::geometry::Point3d;
using rmp::common::geometry::Points3d;

constexpr double kEvaluationSpacingM = 0.05;

struct PlanEpisode
{
    ros::Time received_at;
    Points3d raw_plan;
    Points3d normalized_plan;
    Points3d executed_trajectory;
};

std::vector<PlanEpisode> plan_episodes;
int active_plan_episode = -1;
Points3d traj_map;  // all actual positions, in the active plan frame
std::string plan_frame = "map";                // frame that lay tu header cua plan
int replan_count = 0;                          // so lan move_base hoach dinh lai
int tf_drop_count = 0;                         // so diem odom bi bo vi thieu tf
std::unique_ptr<tf2_ros::Buffer> tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_listener;
std::string output_file;
std::string goal_file;

double pointDistance(const Point3d& a, const Point3d& b)
{
    return std::hypot(a.x() - b.x(), a.y() - b.y());
}

double pathLength(const Points3d& path)
{
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
        length += pointDistance(path[i - 1], path[i]);
    return length;
}

double wrapToPi(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

// Use the same spatial sampling for every planner.  Curvature estimated from
// raw waypoints is otherwise biased by each planner's waypoint density.
Points3d resampleByArcLength(const Points3d& path, double spacing)
{
    Points3d clean;
    clean.reserve(path.size());
    for (const auto& point : path)
    {
        if (clean.empty() || pointDistance(clean.back(), point) > 1e-6)
            clean.push_back(point);
    }
    if (clean.size() < 2 || spacing <= 0.0)
        return clean;

    Points3d sampled;
    sampled.push_back(clean.front());
    double distance_from_start = 0.0;
    double next_sample = spacing;

    for (size_t i = 1; i < clean.size(); ++i)
    {
        const Point3d& a = clean[i - 1];
        const Point3d& b = clean[i];
        const double segment_length = pointDistance(a, b);
        if (segment_length <= 1e-9)
            continue;

        while (next_sample <= distance_from_start + segment_length + 1e-9)
        {
            const double t = (next_sample - distance_from_start) / segment_length;
            sampled.emplace_back(a.x() + t * (b.x() - a.x()),
                                 a.y() + t * (b.y() - a.y()), 0.0);
            next_sample += spacing;
        }
        distance_from_start += segment_length;
    }

    const double final_gap = pointDistance(sampled.back(), clean.back());
    if (final_gap > 1e-6)
    {
        // Do not create a nearly-zero final segment. Such a segment makes
        // kappa = dtheta / ds explode even though the robot/path is smooth.
        if (sampled.size() >= 2 && final_gap < 0.5 * spacing)
            sampled.back() = clean.back();
        else
            sampled.push_back(clean.back());
    }
    return sampled;
}

double turnDensity(const Points3d& path)
{
    const double length = pathLength(path);
    if (path.size() < 3 || length <= 1e-9)
        return 0.0;

    double total_turn = 0.0;
    double previous_heading = std::atan2(path[1].y() - path[0].y(),
                                         path[1].x() - path[0].x());
    for (size_t i = 1; i + 1 < path.size(); ++i)
    {
        const double heading = std::atan2(path[i + 1].y() - path[i].y(),
                                          path[i + 1].x() - path[i].x());
        total_turn += std::abs(wrapToPi(heading - previous_heading));
        previous_heading = heading;
    }
    return total_turn / length;
}

struct PathProjection
{
    double distance = 0.0;
    double arc_length = 0.0;
};

// Unlike nearest-distance alone, arc_length lets the report expose reversals
// along the currently active reference path.
PathProjection projectToPath(const Point3d& point, const Points3d& path)
{
    PathProjection best;
    if (path.empty())
        return best;
    if (path.size() == 1)
    {
        best.distance = pointDistance(point, path.front());
        return best;
    }

    best.distance = std::numeric_limits<double>::infinity();
    double accumulated_length = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
    {
        const Point3d& a = path[i - 1];
        const Point3d& b = path[i];
        const double dx = b.x() - a.x(), dy = b.y() - a.y();
        const double segment_length = std::hypot(dx, dy);
        if (segment_length <= 1e-9)
            continue;
        const double t = std::max(0.0, std::min(1.0,
            ((point.x() - a.x()) * dx + (point.y() - a.y()) * dy) /
                (segment_length * segment_length)));
        const double projected_x = a.x() + t * dx, projected_y = a.y() + t * dy;
        const double distance = std::hypot(point.x() - projected_x, point.y() - projected_y);
        if (distance < best.distance)
        {
            best.distance = distance;
            best.arc_length = accumulated_length + t * segment_length;
        }
        accumulated_length += segment_length;
    }
    return best;
}


// Hàm tính khoảng cách giữa 2 điểm
double calculateDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
{
    double dx = pose1.pose.position.x - pose2.pose.position.x;
    double dy = pose1.pose.position.y - pose2.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Do muot
double computeTurningAngle(
    const geometry_msgs::PoseStamped& p1,
    const geometry_msgs::PoseStamped& p2,
    const geometry_msgs::PoseStamped& p3)
{
    double v1x = p2.pose.position.x - p1.pose.position.x;
    double v1y = p2.pose.position.y - p1.pose.position.y;
    double v2x = p3.pose.position.x - p2.pose.position.x;
    double v2y = p3.pose.position.y - p2.pose.position.y;

    double norm1 = std::sqrt(v1x * v1x + v1y * v1y);
    double norm2 = std::sqrt(v2x * v2x + v2y * v2y);

    if (norm1 < 1e-6 || norm2 < 1e-6)
        return 0.0;

    double dot = v1x * v2x + v1y * v2y;
    double cos_theta = dot / (norm1 * norm2);

    // Clamp để tránh lỗi số học
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));

    return std::acos(cos_theta);
}


// Callback nhan duong da hoach dinh tu global planner.  Do not special-case a
// planner: every plan update becomes the active reference for later odometry.
void planCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if (!goal_received || goal_reached) return;

    ++replan_count;
    if (msg->poses.empty()) return;

    if (!msg->header.frame_id.empty())
        plan_frame = msg->header.frame_id;

    PlanEpisode episode;
    episode.received_at = ros::Time::now();
    episode.raw_plan.reserve(msg->poses.size());
    for (const auto& p : msg->poses)
    {
        // theta khong duoc metric nao doc (huong suy ra tu vi tri lien tiep) -> 0.0
        episode.raw_plan.emplace_back(p.pose.position.x, p.pose.position.y, 0.0);
    }
    episode.normalized_plan = resampleByArcLength(episode.raw_plan, kEvaluationSpacingM);
    plan_episodes.push_back(std::move(episode));
    active_plan_episode = static_cast<int>(plan_episodes.size()) - 1;

    ROS_INFO("path_trace_node: bat plan #%zu (%zu raw, %zu resampled, frame '%s')",
             plan_episodes.size(), plan_episodes.back().raw_plan.size(),
             plan_episodes.back().normalized_plan.size(), plan_frame.c_str());
}

// Hàm callback cho odometry để vẽ path và tính khoảng cách di chuyển
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (!goal_received || goal_reached) return;
 // Chỉ bắt đầu khi đã chọn goal

    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;

    // Nếu không phải là điểm đầu tiên, tính khoảng cách từ điểm trước đến điểm hiện tại
    if (!is_first_pose)
    {
        double dist = calculateDistance(last_pose, pose);
        total_distance += dist; // Cộng vào tổng quãng đường
    }
    else
    {
        is_first_pose = false; // Lần đầu tiên nhận vị trí
    }

    // Cập nhật vị trí cuối cùng
    last_pose = pose;
    path.poses.push_back(pose);

    // Doi diem odom sang frame cua plan de tinh tracking error.
    // Giu `path` nguyen o frame odom: odom muot, con map bi nhay moi khi AMCL
    // hieu chinh -> dung map cho total_distance se lam phong dai quang duong.
    try
    {
        geometry_msgs::PoseStamped pose_in_plan_frame;
        tf_buffer->transform(pose, pose_in_plan_frame, plan_frame, ros::Duration(0.1));
        traj_map.emplace_back(pose_in_plan_frame.pose.position.x,
                              pose_in_plan_frame.pose.position.y, 0.0);
        if (active_plan_episode >= 0)
            plan_episodes[active_plan_episode].executed_trajectory.push_back(traj_map.back());
    }
    catch (const tf2::TransformException& ex)
    {
        ++tf_drop_count;
        ROS_WARN_THROTTLE(5.0, "path_trace_node: bo qua diem odom, chua co tf '%s' -> '%s': %s",
                          pose.header.frame_id.c_str(), plan_frame.c_str(), ex.what());
    }
    // Tính turning angle khi có ít nhất 3 điểm
    int n = path.poses.size();
    if (n >= 3)
    {
        double theta = computeTurningAngle(
            path.poses[n - 3],
            path.poses[n - 2],
            path.poses[n - 1]
        );

        total_turning_angle_sq += theta * theta;
    }

    path.header.stamp = ros::Time::now();

     // Xuất path ra publisher
    path_pub.publish(path);

    // Xuất tổng quãng đường di chuyển
    // ROS_INFO("Total distance travelled: %f meters", total_distance);
}

// Lay ra mot truong trong `details` cua metric, tra ve 0 neu khong co.
double metricDetail(const rmp::path_evaluator::MetricResult& r, const std::string& key)
{
    auto it = r.details.find(key);
    return (it == r.details.end()) ? 0.0 : it->second;
}

// Tim ket qua theo ten metric trong danh sach tra ve tu PathEvaluator.
bool findMetric(const std::vector<rmp::path_evaluator::MetricResult>& results,
                const std::string& name, rmp::path_evaluator::MetricResult& out)
{
    for (const auto& r : results)
    {
        if (r.name == name) { out = r; return true; }
    }
    return false;
}

// Launch chi set dung MOT trong ba param nay (cac guard if= trong
// move_base.launch.xml loai tru nhau), nen thu lan luot roi lay cai dau tien.
std::string queryPlannerName()
{
    const char* keys[] = { "/move_base/SamplePlanner/planner_name",
                           "/move_base/GraphPlanner/planner_name",
                           "/move_base/EvolutionaryPlanner/planner_name" };
    std::string value;
    for (const char* key : keys)
    {
        if (ros::param::get(key, value) && !value.empty())
            return value;
    }
    return "unknown";
}

// Doc gia tri tu YAML::Node kem mac dinh. Mot so key co the vang mat (khi
// findMetric khong tim thay), nen khong dung as<T>() tran - no se nem exception.
double yamlNum(const YAML::Node& node, const std::string& key)
{
    return node[key] ? node[key].as<double>() : 0.0;
}

std::string yamlStr(const YAML::Node& node, const std::string& key)
{
    return node[key] ? node[key].as<std::string>() : std::string("?");
}

// Callback để kiểm tra trạng thái của mục tiêu
void goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
    if (!msg->status_list.empty())
    {
        // Kiểm tra nếu trạng thái là SUCCEEDED (đã đạt mục tiêu)
        for (const auto& status : msg->status_list)
        {
            if (status.status == actionlib_msgs::GoalStatus::SUCCEEDED && !data_saved)
            {
                goal_reached = true;
                double move_duration = ros::Time::now().toSec() - move_start_time; // Tính thời gian di chuyển
                
                
                // Lưu thông tin vào tệp YAML
                YAML::Node data;
                // Normalize smoothness theo chiều dài path
                double smoothness_norm = 0.0;
                if (total_distance > 1e-6)
                    smoothness_norm = total_turning_angle_sq / total_distance;

                data["time(s)"] = move_duration;
                data["cost(m)"] = total_distance;
                data["nodes"] = total_points;
                data["smoothness_turning_angle"] = total_turning_angle_sq;
                data["smoothness_norm"] = smoothness_norm;

                // ---- path_evaluator ----
                // Dang ky metric mot lan; them metric moi sau nay chi can them mot
                // dong registerMetric, khong phai sua PathEvaluator (OCP).
                rmp::path_evaluator::PathEvaluator evaluator;
                evaluator.registerMetric(std::make_shared<rmp::path_evaluator::SmoothnessMetric>());
                evaluator.registerMetric(std::make_shared<rmp::path_evaluator::TrackingErrorMetric>());

                // Evaluate every published plan after identical spatial
                // resampling.  An episode ends when a newer plan arrives.
                double detour_sum = 0.0, turn_density_sum = 0.0;
                double curvature_sum = 0.0, max_curvature = 0.0;
                double tracking_sq_sum = 0.0, tracking_sum = 0.0, tracking_max = 0.0;
                double tracking_backtrack_sum = 0.0;
                size_t evaluated_plans = 0, tracking_points = 0, tracking_episodes = 0;
                YAML::Node episode_reports(YAML::NodeType::Sequence);

                for (const auto& episode : plan_episodes)
                {
                    YAML::Node report;
                    report["raw_plan_points"] = static_cast<int>(episode.raw_plan.size());
                    report["resampled_plan_points"] = static_cast<int>(episode.normalized_plan.size());
                    report["executed_points"] = static_cast<int>(episode.executed_trajectory.size());

                    const double length = pathLength(episode.normalized_plan);
                    const double direct_distance = episode.normalized_plan.size() >= 2
                        ? pointDistance(episode.normalized_plan.front(), episode.normalized_plan.back()) : 0.0;
                    const double detour_ratio = direct_distance > 1e-9 ? length / direct_distance : 0.0;
                    report["length_m"] = length;
                    report["detour_ratio"] = detour_ratio;
                    report["turn_density_rad_per_m"] = turnDensity(episode.normalized_plan);

                    rmp::path_evaluator::MetricResult m;
                    if (findMetric(evaluator.evaluatePath(episode.normalized_plan), "smoothness", m))
                    {
                        report["mean_curvature_1_per_m"] = metricDetail(m, "mean_curvature");
                        report["max_curvature_1_per_m"] = metricDetail(m, "max_curvature");
                        detour_sum += detour_ratio;
                        turn_density_sum += report["turn_density_rad_per_m"].as<double>();
                        curvature_sum += metricDetail(m, "mean_curvature");
                        max_curvature = std::max(max_curvature, metricDetail(m, "max_curvature"));
                        ++evaluated_plans;
                    }

                    if (!episode.executed_trajectory.empty() &&
                        findMetric(evaluator.evaluateTracking(episode.normalized_plan,
                                                              episode.executed_trajectory),
                                   "tracking_error", m))
                    {
                        const size_t n = episode.executed_trajectory.size();
                        report["tracking_rmse_m"] = m.value;
                        report["tracking_mean_error_m"] = metricDetail(m, "mean_error");
                        report["tracking_max_error_m"] = metricDetail(m, "max_error");
                        tracking_sq_sum += m.value * m.value * n;
                        tracking_sum += metricDetail(m, "mean_error") * n;
                        tracking_max = std::max(tracking_max, metricDetail(m, "max_error"));
                        tracking_points += n;
                        ++tracking_episodes;

                        double previous_progress = projectToPath(
                            episode.executed_trajectory.front(), episode.normalized_plan).arc_length;
                        double backtrack_distance = 0.0;
                        for (size_t i = 1; i < episode.executed_trajectory.size(); ++i)
                        {
                            const double progress = projectToPath(
                                episode.executed_trajectory[i], episode.normalized_plan).arc_length;
                            backtrack_distance += std::max(0.0, previous_progress - progress);
                            previous_progress = progress;
                        }
                        report["final_progress_ratio"] = length > 1e-9 ? previous_progress / length : 0.0;
                        report["backtrack_distance_m"] = backtrack_distance;
                        tracking_backtrack_sum += backtrack_distance;
                    }
                    episode_reports.push_back(report);
                }

                // The complete executed trajectory is also resampled so odom
                // publish frequency cannot dominate its curvature estimate.
                const Points3d normalized_trajectory = resampleByArcLength(traj_map, kEvaluationSpacingM);
                rmp::path_evaluator::MetricResult m;
                if (findMetric(evaluator.evaluatePath(normalized_trajectory), "smoothness", m))
                {
                    data["traj_smoothness_mean_curvature"] = metricDetail(m, "mean_curvature");
                    data["traj_smoothness_max_curvature"] = metricDetail(m, "max_curvature");
                }

                data["evaluation_spacing_m"] = kEvaluationSpacingM;
                data["plan_updates"] = static_cast<int>(plan_episodes.size());
                data["plan_evaluated"] = static_cast<int>(evaluated_plans);
                data["plan_mean_detour_ratio"] = evaluated_plans ? detour_sum / evaluated_plans : 0.0;
                data["plan_mean_turn_density_rad_per_m"] =
                    evaluated_plans ? turn_density_sum / evaluated_plans : 0.0;
                data["plan_mean_curvature_1_per_m"] = evaluated_plans ? curvature_sum / evaluated_plans : 0.0;
                data["plan_max_curvature_1_per_m"] = max_curvature;
                data["active_tracking_episodes"] = static_cast<int>(tracking_episodes);
                data["active_tracking_points"] = static_cast<int>(tracking_points);
                data["active_tracking_rmse_m"] = tracking_points ? std::sqrt(tracking_sq_sum / tracking_points) : 0.0;
                data["active_tracking_mean_error_m"] = tracking_points ? tracking_sum / tracking_points : 0.0;
                data["active_tracking_max_error_m"] = tracking_max;
                data["active_tracking_backtrack_m"] = tracking_backtrack_sum;
                data["plan_episodes"] = episode_reports;

                // Bo di kem de doc so lieu tren cho dung
                data["traj_raw_points"] = static_cast<int>(traj_map.size());
                data["traj_points"] = static_cast<int>(normalized_trajectory.size());
                data["replan_count"] = replan_count;
                data["tf_dropped_points"] = tf_drop_count;
                data["eval_frame"] = plan_frame;

                if (plan_episodes.empty())
                    ROS_WARN("path_trace_node: khong nhan duoc plan nao, cac chi so plan/tracking bang 0");
                if (tf_drop_count > 0)
                    ROS_WARN("path_trace_node: %d diem odom bi bo vi thieu tf -> tracking error tinh tren %zu diem",
                             tf_drop_count, traj_map.size());

                // Dung mot lan roi dung chung cho ca file lan terminal, de hai noi
                // khong bao gio lech nhau
                std::stringstream yaml_text;
                yaml_text << data;

                std::ofstream fout(output_file);
                bool file_ok = fout.is_open();
                if (file_ok)
                {
                    fout << yaml_text.str(); // Ghi dữ liệu vào tệp YAML
                    fout.close();
                }
                else
                {
                    ROS_ERROR("path_trace_node: khong mo duoc file de ghi: %s", output_file.c_str());
                }

                // In bang gon ra terminal dang chay rosrun. Moi con so deu doc
                // NGUOC ra tu `data` - dung object vua ghi xuong file - nen so
                // tren terminal khong the lech voi so trong file.
                std::ostringstream rpt;
                rpt << std::fixed;
                rpt << "\n===== KET QUA: " << queryPlannerName() << " =====\n"
                    << " file: " << output_file << "\n\n"
                    << " TONG QUAN\n"
                    << std::setprecision(3)
                    << "   time           " << std::setw(10) << yamlNum(data, "time(s)") << " s\n"
                    << "   cost           " << std::setw(10) << yamlNum(data, "cost(m)") << " m\n"
                    << std::setprecision(0)
                    << "   nodes          " << std::setw(10) << yamlNum(data, "nodes") << "\n"
                    << "   plan_updates   " << std::setw(10) << yamlNum(data, "plan_updates") << "\n"
                    << "   replan_count   " << std::setw(10) << yamlNum(data, "replan_count") << "\n\n"
                    << std::setprecision(4)
                    << " PLAN (resample " << yamlNum(data, "evaluation_spacing_m") << " m)\n"
                    << "   detour ratio   " << std::setw(10) << yamlNum(data, "plan_mean_detour_ratio") << "\n"
                    << "   turn density   " << std::setw(10) << yamlNum(data, "plan_mean_turn_density_rad_per_m") << " rad/m\n"
                    << "   curvature mean/max " << std::setw(10) << yamlNum(data, "plan_mean_curvature_1_per_m")
                    << " / " << yamlNum(data, "plan_max_curvature_1_per_m") << " 1/m\n"
                    << "   robot mean/max " << std::setw(10) << yamlNum(data, "traj_smoothness_mean_curvature")
                    << " / " << yamlNum(data, "traj_smoothness_max_curvature") << "\n\n"
                    << " BAM PLAN ACTIVE (m)\n"
                    << "   rmse           " << std::setw(10) << yamlNum(data, "active_tracking_rmse_m") << "\n"
                    << "   mean / max     " << std::setw(10) << yamlNum(data, "active_tracking_mean_error_m")
                    << " / " << yamlNum(data, "active_tracking_max_error_m") << "\n"
                    << "   backtrack      " << std::setw(10) << yamlNum(data, "active_tracking_backtrack_m") << "\n\n"
                    << std::setprecision(0)
                    << "   (episodes " << yamlNum(data, "plan_evaluated") << ", active tracking "
                    << yamlNum(data, "active_tracking_episodes") << ", traj "
                    << yamlNum(data, "traj_points") << " diem, tf bo "
                    << yamlNum(data, "tf_dropped_points") << ", frame " << yamlStr(data, "eval_frame") << ")\n"
                    << "=========================================\n";
                std::cout << rpt.str() << std::endl;

                data_saved = true; // Đánh dấu rằng dữ liệu đã được lưu
                ROS_INFO("Goal reached!%s", file_ok ? " File saved" : " (ghi file THAT BAI)");
                break;
            }
        }
    }
}

// Callback để nhận goal từ RViz
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    ROS_INFO("New goal received!");

    // Reset path và các biến trạng thái khi nhận goal mới
    goal_received = true;  // Đặt cờ khi goal được nhận
    goal_reached = false;  // Reset cờ mục tiêu đạt được
    data_saved = false;    // Reset cờ lưu dữ liệu
    is_first_pose = true;  // Reset trạng thái để bắt đầu tính toán từ đầu
    total_distance = 0.0;  // Reset tổng quãng đường
    total_turning_angle_sq = 0.0; // Do muot
    total_points = 0;
    previous_seq = 0;
    plan_episodes.clear(); // Reset du lieu cua path_evaluator
    traj_map.clear();
    active_plan_episode = -1;
    replan_count = 0;
    tf_drop_count = 0;
    path.poses.clear();    // Xóa các điểm cũ trong path
    path.header.frame_id = "odom"; // Đảm bảo frame_id đúng cho path
    path.header.stamp = ros::Time::now(); // Reset thời gian cho path
    move_start_time = ros::Time::now().toSec(); // Ghi lại thời gian bắt đầu di chuyển

    // Xuất thông báo để biết path đã được reset
    ROS_INFO("Path and total distance reset for new goal.");
}

// Hàm để đặt một điểm NavGoal từ tọa độ x, y
void setNavGoal(double x, double y)
{
    geometry_msgs::PoseStamped nav_goal;
    nav_goal.header.frame_id = "map"; // Frame_id phù hợp
    nav_goal.header.stamp = ros::Time::now();
    nav_goal.pose.position.x = x;
    nav_goal.pose.position.y = y;
    nav_goal.pose.position.z = 0.0;
    nav_goal.pose.orientation.x = 0.0;
    nav_goal.pose.orientation.y = 0.0;
    nav_goal.pose.orientation.z = -0.7080808546550887;
    nav_goal.pose.orientation.w = 0.7061313640328682;

    // Gửi NavGoal tới topic /move_base_simple/goal
    nav_goal_pub.publish(nav_goal);

    ROS_INFO("NavGoal set at (x: %f, y: %f)", x, y);
}

// Hàm callback để xử lý thông báo từ topic /tree
void treeCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    // SamplePlanner publishes one LINE_LIST for one completed expansion tree.
    // Every edge is represented by exactly two points, so this is the number
    // of expanded tree edges (equivalently, non-root inserted nodes).
    total_points += static_cast<int>(msg->points.size() / 2);
    previous_seq = msg->header.seq;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_trace_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Ten planner co the doi (SamplePlanner / GraphPlanner / ...) nen de thanh tham so.
    std::string plan_topic;
    private_nh.param<std::string>("plan_topic", plan_topic, "move_base/SamplePlanner/plan");

    // Do not depend on the terminal's current directory or an old machine's
    // absolute path.  Both files are addressed relative to this ROS package:
    // path_trace_node -> ../../ is the workspace's src directory.
    const std::string package_path = ros::package::getPath("path_trace_node");
    if (package_path.empty()) {
        ROS_FATAL("Khong tim thay package path_trace_node. Hay source devel/setup.bash truoc khi chay node.");
        return 1;
    }
    const std::string src_path = package_path + "/../..";
    private_nh.param<std::string>("output_file", output_file,
        src_path + "/custom_node/logdata/maze_11_data.yaml");
    private_nh.param<std::string>("goal_file", goal_file,
        src_path + "/user_config/goal_config.yaml");

    // Can tf de doi quy dao odom sang frame cua plan truoc khi tinh tracking error
    tf_buffer = std::make_unique<tf2_ros::Buffer>();
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

    // Khởi tạo path publisher
    path_pub = nh.advertise<nav_msgs::Path>("robot_path", 10);
    
    // Khởi tạo NavGoal publisher
    nav_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    // ROS_INFO("Goal published to /move_base_simple/goal");
    
    // Đăng ký callback cho odometry, trạng thái của mục tiêu và goal từ RViz
    ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback);
    ros::Subscriber goal_status_sub = nh.subscribe("move_base/status", 10, goalStatusCallback);
    ros::Subscriber goal_sub = nh.subscribe("move_base_simple/goal", 10, goalCallback); // Lắng nghe khi goal được chọn từ RViz

    //expand
    ros::Subscriber tree_sub = nh.subscribe("move_base/SamplePlanner/tree", 10, treeCallback);

    // Duong da hoach dinh: lam tham chieu cho tracking error va do muot cua planner
    ros::Subscriber plan_sub = nh.subscribe(plan_topic, 10, planCallback);
    ROS_INFO("path_trace_node: lang nghe plan tren '%s', ghi log ra '%s'",
             plan_topic.c_str(), output_file.c_str());

    path.header.frame_id = "odom";  // Đặt frame_id phù hợp

    try {
        YAML::Node config = YAML::LoadFile(goal_file);
        double x = config["goal"]["x"].as<double>();
        double y = config["goal"]["y"].as<double>();
        setNavGoal(0.0, 0.0);
        ros::Duration(1.0).sleep();
        setNavGoal(x, y);
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to load goal file '%s': %s", goal_file.c_str(), e.what());
    }
    
    ros::spin();
    return 0;
}
