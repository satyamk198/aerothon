cmake_minimum_required(VERSION 3.8)
project(mission_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # flight_controller
  "action/Takeoff.action"
  "action/GoTo.action"
  "action/Land.action"
  # qr_detector
  "action/CaptureReference.action"
  # task behaviors (goal/result fields to be filled in per task)
  "action/AlignWithCorridor.action"
  "action/NavigateCorridor.action"
  "action/SearchTarget.action"
  "action/CenterOnTarget.action"
  "action/DeliverPayload.action"
  # camera
  "srv/SetZoom.srv"
  "srv/SetPhotoTask.srv"
  # perception outputs
  "msg/QrCode.msg"
  "msg/QrDetection.msg"
  "msg/RedPatch.msg"
  "msg/BannerDetection.msg"
  "msg/RestrictedZones.msg"
  "msg/Obstacles.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
