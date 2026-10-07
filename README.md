# AutoAimTypes

自瞄链路公共类型：逐层帧、装甲板、IMU 样本与 Topic 查找 / Shared auto-aim types for stage frames, armors, IMU samples and Topic lookup

## 1. 模块作用 / Purpose

AutoAimTypes 是库型模块（`standalone: false`），定义自瞄链路各层之间传递的帧类型。每一层把上一层的整帧加上本层的输出发给下一层，所以最后一层的帧里有原图、IMU、检测结果、跟踪目标和瞄准结果，排查问题时只看一个帧即可。

AutoAimTypes is a library Module (`standalone: false`) that defines the frames passed between the stages of the auto-aim chain. Each stage passes the previous stage's whole frame plus its own output, so the last frame holds the image, the IMU sample, the detections, the tracked target and the aim result.

## 2. 逐层帧 / Stage Frames

| 层 / Stage | 帧 / Frame | Topic | 生产者 / Producer |
| --- | --- | --- | --- |
| 同步 / synced | `SyncedFrame{sequence, image, imu}` | `<相机名>_synced` | CameraFrameSync |
| 检测 / detected | `DetectedFrame{synced, armors}` | `<相机名>_detected` | ArmorDetector |
| 跟踪 / tracked | `TrackedFrame{detected, target}` | `<相机名>_tracked` | ArmorTracker |
| 瞄准 / aimed | `AimedFrame{tracked, aim}` | `<相机名>_aimed` | Aimer |

Topic 载荷是指向帧的指针，只在同步回调期间有效。每层收进一帧必发出一帧，结果为空也照发：检测为空、未跟踪、不控制都照样发布，帧只会在某层入口被丢弃或某层卡住时中断。

The Topic payload is a pointer to the frame, valid only during the synchronous callback. Every stage publishes one frame for every frame it accepts, including empty results, so a frame stops only when a stage drops it at admission or a stage stalls.

`Armor` 的四个角点是灯条四端点（lightbar4），顺序为左上、左下、右下、右上（以装甲板自身为准），单位为原生像素。`ArmorTrackerTarget` 的字段与语义沿用原跟踪器的输出，Aimer 直接读取。

The four `Armor` corners are the light-bar end points (lightbar4) in the order top-left, bottom-left, bottom-right, top-right on the plate itself, in native pixels. `ArmorTrackerTarget` keeps the fields and meaning of the former tracker output, which Aimer reads directly.

## 3. 工具 / Helpers

- `RequireTopic<Payload>(name)`：查找已存在的 Topic 并校验载荷类型，找不到即致命退出。消费者用它接线，生产者用 `LibXR::Topic::CreateTopic` 创建；按 YAML 顺序，生产者先于消费者构造。
- `ShouldLog(count)`：限频日志，前 5 次与此后每 100 次返回 true。
- lightbar4 物体点常量：小板半宽 66.9 mm、大板半宽 114.4 mm、半高 29.3 mm。按 v7 模型的角点位置用测距集拟合，换检测模型要重新拟合；大板未实测，由小板外扩量推算。

- `RequireTopic<Payload>(name)`: finds an existing Topic and checks its payload type; a missing Topic is fatal. Consumers wire up with it and producers create Topics with `LibXR::Topic::CreateTopic`; following the YAML order, producers are constructed before consumers.
- `ShouldLog(count)`: rate limiting for logs, true for the first 5 calls and every 100th after that.
- lightbar4 object point constants: small plate half width 66.9 mm, large plate half width 114.4 mm, half height 29.3 mm, fitted to where the v7 model puts the corners on the range set; refit for another detector model. The large plate is not measured and follows the small plate's bloom.

## 4. 依赖 / Dependencies

CameraBase、LibXR、Eigen。

CameraBase, LibXR, Eigen.
