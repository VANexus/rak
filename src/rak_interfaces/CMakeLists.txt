cmake_minimum_required(VERSION 3.22)
project(rak_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraspCandidate.msg"
  "msg/GraspCandidates.msg"
  "srv/SelectGrasp.srv"
  "msg/TrackedObject.msg"
  "msg/TrackedObjects.msg"
  "msg/Action.msg"
  "msg/Decision.msg"
  "msg/ToolProposal.msg"
  "msg/ToolResult.msg"
  "msg/TaskStatus.msg"
  "msg/BaseTelemetry.msg"
  "msg/WakeEvent.msg"
  "msg/HarnessRequest.msg"
  "msg/HarnessProposal.msg"
  "msg/HarnessEvent.msg"
  "action/ExecuteTask.action"
  "srv/TaskControl.srv"
  "srv/Say.srv"
  "srv/SendAtomicAction.srv"
  DEPENDENCIES std_msgs geometry_msgs vision_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
