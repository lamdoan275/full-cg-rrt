#include <ros/ros.h>
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
#include <memory>
#include <sstream>
#include <string>
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
// Duong da hoach dinh (frame "map") va quy dao thuc te da doi ve CUNG frame do.
// Plan duoc publish trong frame map, con /odom o frame odom; AMCL lam 2 frame nay
// lech nhau va troi theo thoi gian, nen bat buoc phai doi frame truoc khi so sanh.
rmp::common::geometry::Points3d plan_points;   // duong tham chieu, frame map
rmp::common::geometry::Points3d traj_map;      // quy dao thuc te, frame map
std::string plan_frame = "map";                // frame that lay tu header cua plan
bool plan_captured = false;                    // chi giu plan DAU TIEN sau moi goal
int replan_count = 0;                          // so lan move_base hoach dinh lai
int tf_drop_count = 0;                         // so diem odom bi bo vi thieu tf
std::unique_ptr<tf2_ros::Buffer> tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_listener;
std::string output_file;


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


// Callback nhan duong da hoach dinh tu global planner.
// move_base hoach dinh lai lien tuc; ta chi giu lai plan DAU TIEN sau moi goal vi
// do moi la loi giai that su cua planner (thu dung de so sanh rrt_cut vs informed_rrt).
// Cac lan sau chi dem vao replan_count de biet planner phai sua bao nhieu lan.
void planCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if (!goal_received || goal_reached) return;

    ++replan_count;
    if (plan_captured || msg->poses.empty()) return;

    if (!msg->header.frame_id.empty())
        plan_frame = msg->header.frame_id;

    plan_points.clear();
    plan_points.reserve(msg->poses.size());
    for (const auto& p : msg->poses)
    {
        // theta khong duoc metric nao doc (huong suy ra tu vi tri lien tiep) -> 0.0
        plan_points.emplace_back(p.pose.position.x, p.pose.position.y, 0.0);
    }
    plan_captured = true;
    ROS_INFO("path_trace_node: da bat plan dau tien (%zu diem, frame '%s')",
             plan_points.size(), plan_frame.c_str());
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

                rmp::path_evaluator::MetricResult m;

                // Do muot cua DUONG DA HOACH DINH -> chinh la thu de so sanh planner
                if (findMetric(evaluator.evaluatePath(plan_points), "smoothness", m))
                {
                    data["plan_smoothness_mean_curvature"] = metricDetail(m, "mean_curvature");
                    data["plan_smoothness_max_curvature"] = metricDetail(m, "max_curvature");
                }

                // Do muot cua QUY DAO ROBOT THUC TE chay
                if (findMetric(evaluator.evaluatePath(traj_map), "smoothness", m))
                {
                    data["traj_smoothness_mean_curvature"] = metricDetail(m, "mean_curvature");
                    data["traj_smoothness_max_curvature"] = metricDetail(m, "max_curvature");
                }

                // Sai so bam duong: quy dao thuc te lech bao xa so voi plan
                if (findMetric(evaluator.evaluateTracking(plan_points, traj_map), "tracking_error", m))
                {
                    data["tracking_rmse"] = m.value;
                    data["tracking_max_error"] = metricDetail(m, "max_error");
                    data["tracking_mean_error"] = metricDetail(m, "mean_error");
                }

                // Bo di kem de doc so lieu tren cho dung
                data["plan_points"] = static_cast<int>(plan_points.size());
                data["traj_points"] = static_cast<int>(traj_map.size());
                data["replan_count"] = replan_count;
                data["tf_dropped_points"] = tf_drop_count;
                data["eval_frame"] = plan_frame;

                if (plan_points.empty())
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
                    << "   replan_count   " << std::setw(10) << yamlNum(data, "replan_count") << "\n\n"
                    << std::setprecision(4)
                    << " DO MUOT (curvature, 1/m)\n"
                    << "   plan  mean/max " << std::setw(10) << yamlNum(data, "plan_smoothness_mean_curvature")
                    << " / " << yamlNum(data, "plan_smoothness_max_curvature") << "\n"
                    << "   robot mean/max " << std::setw(10) << yamlNum(data, "traj_smoothness_mean_curvature")
                    << " / " << yamlNum(data, "traj_smoothness_max_curvature") << "\n\n"
                    << " BAM DUONG (m)\n"
                    << "   rmse           " << std::setw(10) << yamlNum(data, "tracking_rmse") << "\n"
                    << "   mean / max     " << std::setw(10) << yamlNum(data, "tracking_mean_error")
                    << " / " << yamlNum(data, "tracking_max_error") << "\n\n"
                    << std::setprecision(0)
                    << "   (plan " << yamlNum(data, "plan_points") << " diem, traj "
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
    plan_points.clear();   // Reset du lieu cua path_evaluator
    traj_map.clear();
    plan_captured = false;
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
    // Lấy seq của thông điệp mới
    uint32_t current_seq = msg->header.seq;

    // Chỉ cộng dồn nếu seq của thông điệp mới lớn hơn seq trước đó
    if (current_seq > previous_seq) {
        // Cộng dồn số điểm vào tổng
        total_points += msg->points.size();

        // Cập nhật seq trước đó
        previous_seq = current_seq;

        // In ra tổng số điểm
        // ROS_INFO("Total points accumulated in tree: %d", total_points);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_trace_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Ten planner co the doi (SamplePlanner / GraphPlanner / ...) nen de thanh tham so.
    std::string plan_topic;
    private_nh.param<std::string>("plan_topic", plan_topic, "move_base/SamplePlanner/plan");
    private_nh.param<std::string>("output_file", output_file,
        "/home/roab_lab/ros_motion_planning-master/src/custom_node/logdata/maze_11_data.yaml");

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
        YAML::Node config = YAML::LoadFile("/home/roab_lab/ros_motion_planning-master/src/user_config/goal_config.yaml");
        double x = config["goal"]["x"].as<double>();
        double y = config["goal"]["y"].as<double>();
        setNavGoal(0.0, 0.0);
        ros::Duration(1.0).sleep();
        setNavGoal(x, y);
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to load goal.yaml: %s", e.what());
    }
    
    ros::spin();
    return 0;
}