#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 自瞄链路公共类型：逐层帧、装甲板、IMU 样本与 Topic 查找 / Shared auto-aim types for stage frames, armors, IMU samples and Topic lookup
depends:
- id: QDU-Robomaster/CameraBase
  ref: same-or-dev
standalone: false
=== END MANIFEST === */
// clang-format on

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "CameraBase.hpp"
#include "libxr_def.hpp"
#include "logger.hpp"
#include "message.hpp"

/**
 * @brief 自瞄链路的公共类型 / Shared types of the auto-aim chain.
 *
 * 每一层把上一层的整帧加上本层输出发给下一层：
 * `SyncedFrame` → `DetectedFrame` → `TrackedFrame` → `AimedFrame`。
 * 每层收进一帧必发出一帧，结果为空也照发；Topic 名为 `<相机名>_<层>`，由生产者创建。
 * Each stage passes the previous stage's whole frame plus its own output downstream.
 * Every stage publishes one frame per accepted input, empty results included. Topics
 * are named `<camera>_<stage>` and created by the producer.
 */
namespace AutoAim
{
/// 各层 Topic 后缀 / Stage Topic suffixes.
inline constexpr std::string_view STAGE_SYNCED = "synced";
inline constexpr std::string_view STAGE_DETECTED = "detected";
inline constexpr std::string_view STAGE_TRACKED = "tracked";
inline constexpr std::string_view STAGE_AIMED = "aimed";

/// 与图像同步的 IMU 样本，公共机体系 x 右、y 前、z 上 / IMU sample synced with an
/// image, body frame x right, y forward, z up.
struct ImuSample
{
  LibXR::MicrosecondTimestamp timestamp_us;      ///< MCU 时间 / MCU time
  std::array<float, 4> rotation_wxyz;            ///< 姿态四元数 / Attitude quaternion
  std::array<float, 3> angular_velocity_xyz;     ///< rad/s
  std::array<float, 3> linear_acceleration_xyz;  ///< m/s²
};

/// 同步层：图像与对应 IMU / Synced stage: an image and its IMU sample.
struct SyncedFrame
{
  uint64_t sequence;  ///< CFS 分配，单调递增 / Assigned by CFS, increasing
  SharedFrame image;
  ImuSample imu;
};
}  // namespace AutoAim

// 以下枚举沿用原 ArmorDetectorTypes 的名字与取值，Aimer 依赖它们。
// The enums keep the names and values of the former ArmorDetectorTypes; Aimer uses them.

enum class ArmorColor : uint8_t
{
  RED = 0,
  BLUE = 1,
  UNKNOWN = 2,
};

enum class ArmorType : uint8_t
{
  SMALL = 0,
  LARGE = 1,
};

enum class ArmorNumber : uint8_t
{
  ONE = 0,
  TWO = 1,
  THREE = 2,
  FOUR = 3,
  FIVE = 4,
  OUTPOST = 5,
  GUARD = 6,
  BASE = 7,
  NEGATIVE = 8,  ///< 不是装甲板或无法识别 / Not an armor or unrecognised
  UNKNOWN = NEGATIVE,
  INVALID = NEGATIVE,
};

inline constexpr std::array<std::string_view, 9> ARMOR_NUMBER_NAMES = {
    "one", "two", "three", "four", "five", "outpost", "guard", "base", "negative"};

namespace AutoAim
{
/// 单个像素点 / One pixel point.
struct Point2f
{
  float x;
  float y;
};

/**
 * @brief 检测到的一块装甲板 / One detected armor plate.
 *
 * 角点为灯条四端点（lightbar4），顺序左上、左下、右下、右上（以板自身为准），原生像素。
 * Corners are the four light-bar end points (lightbar4) ordered top-left, bottom-left,
 * bottom-right, top-right on the plate itself, in native pixels.
 */
struct Armor
{
  ArmorColor color;
  ArmorNumber number;
  ArmorType type;
  float confidence;
  std::array<Point2f, 4> corners;
};

/// 检测层 / Detected stage.
struct DetectedFrame
{
  SyncedFrame synced;
  std::vector<Armor> armors;
};

/// lightbar4 物体点：半宽、半高，米。按 v4 模型的角点位置用测距集（2–6 m 正对）拟合；
/// 换检测模型要重新拟合。大板没有实测，按小板外扩量 +1.4 mm 推算。
///
/// lightbar4 object points: half width and half height in metres, fitted to where
/// the v4 model puts the corners on the range set (2–6 m, facing); refit for another
/// detector model. The large plate is not measured: small-plate bloom +1.4 mm.
inline constexpr double SMALL_ARMOR_HALF_WIDTH = 0.0663;
inline constexpr double LARGE_ARMOR_HALF_WIDTH = 0.1138;
inline constexpr double ARMOR_HALF_HEIGHT = 0.0283;
}  // namespace AutoAim

/**
 * @brief 跟踪器给 Aimer 的目标状态（字段与语义沿用原 ArmorTrackerTarget）。
 *        Target state from the tracker to Aimer (fields and meaning unchanged).
 */
struct ArmorTrackerTarget
{
  uint64_t image_timestamp_us{};  ///< 同步帧 IMU 时间 / IMU time of the synced frame
  bool tracking{};
  ArmorNumber id{ArmorNumber::INVALID};
  int armors_num{};  ///< 装甲面数，通常 1、3 或 4 / Plates, usually 1, 3 or 4
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  ///< 整车中心 / Centre, m
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  ///< m/s
  double yaw{};                                        ///< rad
  double v_yaw{};                                      ///< rad/s
  double radius_1{};  ///< 偶数面半径 / Even-plate radius, m
  double radius_2{};  ///< 奇数面半径 / Odd-plate radius, m
  double dz{};        ///< 奇偶面高度差 / Height difference, m
  int tracked_face_index{0};
  int outpost_height_phase{0};
  bool face_switch_observed{false};
};

namespace AutoAim
{
/// 跟踪层 / Tracked stage.
struct TrackedFrame
{
  DetectedFrame detected;
  ArmorTrackerTarget target;
  /// 输出系到相机系的旋转（行优先）与平移，供预览投影 / Output-to-camera transform
  /// for preview projection.
  std::array<double, 9> output_to_camera_rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
  std::array<double, 3> output_to_camera_translation{0, 0, 0};
};

/// Aimer 的结果 / Aimer result.
struct AimResult
{
  bool control;  ///< 是否控制云台 / Whether the gimbal is commanded
  bool fire;
  float yaw;                                            ///< rad
  float pitch;                                          ///< rad
  Eigen::Vector3d aim_point = Eigen::Vector3d::Zero();  ///< 输出系，m / Output frame
  int plate;                                            ///< 目标板序号，-1 为无
};

/// 瞄准层 / Aimed stage.
struct AimedFrame
{
  TrackedFrame tracked;
  AimResult aim;
};

/**
 * @brief 查找已存在的 Topic 并校验载荷类型；找不到即致命退出（不等待、不代建）。
 *        Find an existing Topic and check its payload type; a missing Topic is fatal
 *        (no waiting, no placeholder).
 */
template <typename Payload>
LibXR::Topic RequireTopic(const std::string& name, LibXR::Topic::Domain* domain = nullptr)
{
  if (LibXR::Topic::Find(name.c_str(), domain) == nullptr)
  {
    XR_LOG_ERROR("required Topic %s does not exist; check the module order",
                 name.c_str());
    REQUIRE(false);
  }
  // Topic 已存在，FindOrCreate 只做类型校验 / The Topic exists, so this only checks
  // the payload type.
  return LibXR::Topic(LibXR::Topic::FindOrCreate<Payload>(name.c_str(), domain));
}

/// 限频：前 5 次与此后每 100 次返回 true / True for the first 5 and every 100th call.
inline bool ShouldLog(uint64_t count) { return count <= 5 || count % 100 == 0; }
}  // namespace AutoAim
