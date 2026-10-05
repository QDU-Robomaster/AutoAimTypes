#include <cstdio>
#include <cstdlib>
#include <type_traits>

#include "AutoAimTypes.hpp"
#include "libxr.hpp"

namespace
{
void Expect(bool condition, const char* message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

// 每一层都完整包含上一层 / Every stage contains the whole previous stage.
static_assert(
    std::is_same_v<decltype(AutoAim::DetectedFrame::synced), AutoAim::SyncedFrame>);
static_assert(
    std::is_same_v<decltype(AutoAim::TrackedFrame::detected), AutoAim::DetectedFrame>);
static_assert(
    std::is_same_v<decltype(AutoAim::AimedFrame::tracked), AutoAim::TrackedFrame>);
// Aimer 依赖的名字与取值不变 / Names and values Aimer relies on are unchanged.
static_assert(static_cast<int>(ArmorNumber::OUTPOST) == 5);
static_assert(ArmorNumber::INVALID == ArmorNumber::NEGATIVE);
static_assert(ARMOR_NUMBER_NAMES[5] == "outpost");

void TestRequireTopic()
{
  const std::string name = StageTopicName("gimbal", AutoAim::STAGE_DETECTED);
  Expect(name == "gimbal_detected", "stage topic name");
  LibXR::Topic created =
      LibXR::Topic::CreateTopic<const AutoAim::DetectedFrame*>(name.c_str());
  LibXR::Topic found = AutoAim::RequireTopic<const AutoAim::DetectedFrame*>(name);
  Expect(found.GetKey() == created.GetKey(), "RequireTopic returns the existing Topic");
}

void TestShouldLog()
{
  int printed = 0;
  for (uint64_t i = 1; i <= 1000; ++i)
  {
    printed += AutoAim::ShouldLog(i) ? 1 : 0;
  }
  Expect(printed == 5 + 10, "first five and every hundredth");
}
}  // namespace

int main()
{
  LibXR::PlatformInit();
  TestRequireTopic();
  TestShouldLog();
  std::puts("autoaim_types_test passed");
  return 0;
}
