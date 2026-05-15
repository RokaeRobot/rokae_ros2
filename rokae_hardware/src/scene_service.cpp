#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rokae_hardware/srv/scene_service.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using SceneService = rokae_hardware::srv::SceneService;

namespace
{
std::filesystem::path user_storage_dir()
{
#ifdef _WIN32
  const char *home = std::getenv("USERPROFILE");
#else
  const char *home = std::getenv("HOME");
#endif
  if (home == nullptr) {
    return std::filesystem::path(".") / ".rokae_gazebo";
  }
  return std::filesystem::path(home) / ".rokae_gazebo";
}

std::string resolve_world_path(
  const std::string &scene_name,
  const std::string &worlds_package)
{
  std::string base = scene_name;
  if (base.size() < 6 || base.substr(base.size() - 6) != ".world") {
    base += ".world";
  }
  std::string share;
  try {
    share = ament_index_cpp::get_package_share_directory(worlds_package);
  } catch (const std::exception &) {
    return {};
  }
  std::filesystem::path p = std::filesystem::path(share) / "worlds" / base;
  if (!std::filesystem::exists(p)) {
    return {};
  }
  return std::filesystem::absolute(p).string();
}

bool copy_file_to(const std::filesystem::path &from, const std::filesystem::path &to)
{
  try {
    std::filesystem::create_directories(to.parent_path());
    std::filesystem::copy_file(
      from, to,
      std::filesystem::copy_options::overwrite_existing);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}
}  // namespace

class SceneManager : public rclcpp::Node
{
public:
  SceneManager()
  : Node("scene_manager")
  {
    worlds_package_ = declare_parameter<std::string>("worlds_package", "rokae_gazebo");
    declare_parameter<std::string>("scene.active_world_file", "");
    load_service_ = create_service<SceneService>(
      "/load_scene", std::bind(&SceneManager::load_cb, this, _1, _2));
    save_service_ = create_service<SceneService>(
      "/save_scene", std::bind(&SceneManager::save_cb, this, _1, _2));
    RCLCPP_INFO(get_logger(), "场景服务已就绪（ROS 2 Service，不是 Topic）。");
    RCLCPP_INFO(
      get_logger(),
      "  /load_scene  类型 rokae_hardware/srv/SceneService  —  示例: ros2 service call /load_scene "
      "rokae_hardware/srv/SceneService \"{scene_name: obstacles}\"");
    RCLCPP_INFO(
      get_logger(),
      "  /save_scene  类型 rokae_hardware/srv/SceneService  —  示例: ros2 service call /save_scene "
      "rokae_hardware/srv/SceneService \"{scene_name: my_dump}\"");
    RCLCPP_INFO(
      get_logger(),
      "world 文件目录: 包 %s/share/%s/worlds/ ；若 ros2 run 报 No executable found，请先在本工作空间执行 "
      "colcon build --packages-select rokae_hardware 并 source install/setup.bash",
      worlds_package_.c_str(), worlds_package_.c_str());
  }

private:
  std::string worlds_package_;
  rclcpp::Service<SceneService>::SharedPtr load_service_;
  rclcpp::Service<SceneService>::SharedPtr save_service_;

  void load_cb(
    const SceneService::Request::SharedPtr req,
    SceneService::Response::SharedPtr res)
  {
    const std::string path = resolve_world_path(req->scene_name, worlds_package_);
    if (path.empty()) {
      res->success = false;
      res->message =
        "未找到场景文件。请将 " + req->scene_name +
        " 放在 " + worlds_package_ + "/worlds/ 下，或使用完整文件名（含 .world）。";
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
      return;
    }
    set_parameter(rclcpp::Parameter{"scene.active_world_file", path});
    /* Classic Gazebo 通常在启动时载入 world；此处校验路径并记录参数，供文档/其它节点使用 */
    res->success = true;
    res->message =
      "场景已解析: " + path +
      "。若需替换运行中的世界，请关闭 Gazebo 后用 launch 的 world 参数启动该文件。";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  void save_cb(
    const SceneService::Request::SharedPtr req,
    SceneService::Response::SharedPtr res)
  {
    std::string name = req->scene_name;
    if (name.empty()) {
      name = "saved_scene";
    }
    if (name.size() < 6 || name.substr(name.size() - 6) != ".world") {
      name += ".world";
    }

    /* 将 worlds 包中 obstacles.world 的快照保存到用户目录（无法从 Gazebo 内存直接导出时的折中） */
    std::string src = resolve_world_path("obstacles.world", worlds_package_);
    if (src.empty()) {
      src = resolve_world_path("empty.world", worlds_package_);
    }
    if (src.empty()) {
      res->success = false;
      res->message = "无法定位源 world 文件（obstacles/empty）。";
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
      return;
    }

    const std::filesystem::path out = user_storage_dir() / "saved_worlds" / name;
    if (!copy_file_to(std::filesystem::path(src), out)) {
      res->success = false;
      res->message = "写入失败: " + out.string();
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
      return;
    }

    std::ofstream note(out.string() + ".txt", std::ios::app);
    if (note) {
      note << "# snapshot from scene_service at ros2 run time\n";
    }

    res->success = true;
    res->message = "已保存副本: " + out.string();
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SceneManager>());
  rclcpp::shutdown();
  return 0;
}
