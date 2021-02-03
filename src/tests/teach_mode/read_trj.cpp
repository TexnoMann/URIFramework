#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include "CSVWriter.h"
#include "spdlog/spdlog.h"

#include <chrono>

using namespace std::chrono;
using namespace ur_rtde;


uint16_t Nmes = 500;

uint16_t time_delay_ms = 200;

int main(int argc, char* argv[]) {

    if(argc != 2){
      spdlog::error("Invalid count arguments");
    }

    std::string csv_filename_out = argv[1];

    RTDEControlInterface rtde_control("192.168.88.5");
    RTDEReceiveInterface rtde_receive("192.168.88.5");

    CSVWriter csv_out(",");

    std::vector<double> joint_q = {0.0, -3.14 / 2, 0.0, -3.14 / 2, 0.0, 0.0};
//    Move to initial joint position with a regular moveJ
    spdlog::info("[Target] Go to initial position");
    rtde_control.moveJ(joint_q);

    std::vector<double> cart_pos = rtde_receive.getActualTCPPose();
    spdlog::info("[State] Cartesian position: {0} {1} {2}", cart_pos[0], cart_pos[1], cart_pos[2]);

    spdlog::info("[Ident] Start teach mode and writing pose to file: " + csv_filename_out);
    rtde_control.teachMode();

    for(int i = 0; i < Nmes; i++){
        joint_q = rtde_receive.getActualQ();
        spdlog::info("[Read] Joint position i={6}: {0} {1} {2} {3} {4} {5}", joint_q[0], joint_q[1], joint_q[2], joint_q[3], joint_q[4], joint_q[5], i);
        csv_out.newRow()<<joint_q[0]<<joint_q[1]<<joint_q[2]<<joint_q[3]<<joint_q[4]<<joint_q[5];
        std::this_thread::sleep_for(std::chrono::milliseconds(time_delay_ms));
    }
    rtde_control.endTeachMode();
    rtde_control.stopScript();
    csv_out.writeToFile(csv_filename_out, true);
    return 0;
}
