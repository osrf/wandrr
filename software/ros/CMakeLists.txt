cmake_minimum_required(VERSION 2.8.3)
project(wandrr)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation)

#add_message_files(FILES Parameter.msg JointTarget.msg RobotTarget.msg)
#add_service_files(FILES GetParameters.srv SetParameter.srv)

generate_messages()
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES beef_driver
  CATKIN_DEPENDS roscpp message_runtime
  #DEPENDS system_lib
)

add_library(wandrr chain.cpp wandrr.cpp serial_router.cpp mega_state.cpp steppr_interface.cpp wandrr_controller.cpp mc.cpp power_router.cpp local_mcast_tx.cpp local_mcast_rx.cpp)
target_link_libraries(wandrr yaml-cpp ${catkin_LIBRARIES})

add_library(wandrr_ros wandrr_ros.cpp)
target_link_libraries(wandrr_ros wandrr)

add_executable(led led.cpp)
target_link_libraries(led wandrr ${catkin_LIBRARIES})

add_executable(reset_overheat_latch reset_overheat_latch.cpp)
target_link_libraries(reset_overheat_latch wandrr ${catkin_LIBRARIES})

add_executable(led_chase led_chase.cpp)
target_link_libraries(led_chase wandrr ${catkin_LIBRARIES})

add_executable(led_chase_all_chains led_chase_all_chains.cpp)
target_link_libraries(led_chase_all_chains wandrr ${catkin_LIBRARIES})

add_executable(play_scripted_currents play_scripted_currents.cpp)
target_link_libraries(play_scripted_currents wandrr wandrr_ros ${catkin_LIBRARIES})

add_executable(chain_params chain_params.cpp)
target_link_libraries(chain_params wandrr ${catkin_LIBRARIES})

#add_executable(tx_mcu tx_mcu.cpp)
#target_link_libraries(tx_mcu wandrr ${catkin_LIBRARIES})

add_executable(flash flash.cpp)
target_link_libraries(flash wandrr ${catkin_LIBRARIES})

add_executable(brake brake.cpp)
target_link_libraries(brake wandrr wandrr_ros ${catkin_LIBRARIES})

add_executable(usb usb.cpp)
target_link_libraries(usb wandrr wandrr_ros ${catkin_LIBRARIES})

add_executable(serial_router_cli serial_router_cli.cpp)
target_link_libraries(serial_router_cli wandrr ${catkin_LIBRARIES})

add_executable(power_router_cli power_router_cli.cpp)
target_link_libraries(power_router_cli wandrr ${catkin_LIBRARIES})

##add_executable(pwm pwm.cpp)
#target_link_libraries(pwm wandrr wandrr_ros ${catkin_LIBRARIES})

add_executable(software_router software_router.cpp)
target_link_libraries(software_router wandrr wandrr_ros ${catkin_LIBRARIES})

add_executable(whole_body_control whole_body_control.cpp)
target_link_libraries(whole_body_control wandrr ${catkin_LIBRARIES})

add_executable(whole_body_cmd_blaster whole_body_cmd_blaster.cpp)
target_link_libraries(whole_body_cmd_blaster wandrr ${catkin_LIBRARIES})


add_executable(wandrr_test_packets wandrr_test_packets.cpp)
target_link_libraries(wandrr_test_packets wandrr wandrr_ros ${catkin_LIBRARIES})

add_executable(play_dmxl_script play_dmxl_script.cpp)
target_link_libraries(play_dmxl_script wandrr wandrr_ros ${catkin_LIBRARIES})

add_executable(arm_controller arm_controller.cpp)
target_link_libraries(arm_controller wandrr wandrr_ros ${catkin_LIBRARIES})

add_executable(booth_arm_controller booth_arm_controller.cpp)
target_link_libraries(booth_arm_controller wandrr wandrr_ros ${catkin_LIBRARIES})

