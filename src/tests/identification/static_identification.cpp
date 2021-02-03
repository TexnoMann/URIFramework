#include <iostream>
#include <fstream>
#include <string>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <iostream>
#include <dirent.h>
#include <thread>
#include "csv.h"
#include "CSVWriter.h"
#include "spdlog/spdlog.h"

#include <chrono>

using namespace std::chrono;
using namespace ur_rtde;

double velocity = 0.4;
double acceleration = 0.05;
double dt = 1.0/500; // 2ms
double lookahead_time = 0.1;
double gain = 300;
uint16_t Nmes = 10;

bool goto_ident_point(RTDEControlInterface & rtde_control, RTDEReceiveInterface & rtde_receive, std::vector<double> & joint_pose);

int main(int argc, char* argv[]) {

    if(argc !=3 ){
      spdlog::error("Invalid count arguments");
    }
    std::string csv_filename = argv[1];
    std::string csv_filename_out = argv[2];

    RTDEControlInterface rtde_control("192.168.88.5");
    RTDEReceiveInterface rtde_receive("192.168.88.5");

    io::CSVReader<6> csv_in(csv_filename);
    CSVWriter csv_out(",");

    std::vector<double> joint_q = {0.0, -3.14 / 2, 0.0, -3.14 / 2, 0.0, 0.0};
//    Move to initial joint position with a regular moveJ
    rtde_control.moveJ(joint_q);

    std::vector<double> cart_pos = rtde_receive.getActualTCPPose();
    spdlog::info("[State] Cartesian position: {0} {1} {2}", cart_pos[0], cart_pos[1], cart_pos[2]);

    spdlog::info("[Ident] Start static identification from file: " + csv_filename);

    std::vector<double> current;
    while(csv_in.read_row(joint_q[0], joint_q[1], joint_q[2], joint_q[3], joint_q[4], joint_q[5])) {
        goto_ident_point(rtde_control, rtde_receive, joint_q);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        for(int i = 0; i < Nmes; i++){
            current = rtde_receive.getActualCurrent();
            spdlog::info("[Read] Joint current: {0} {1} {2} {3} {4} {5}", current[0], current[1], current[2], current[3], current[4], current[5]);
            csv_out.newRow()<<current[0]<<current[1]<<current[2]<<current[3]<<current[4]<<current[5];
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

    }
    csv_out.writeToFile(csv_filename_out,true);
    rtde_control.stopScript();
    return 0;
}


bool goto_ident_point(RTDEControlInterface & rtde_control, RTDEReceiveInterface & rtde_receive, std::vector<double> & joint_pose){
    spdlog::info("[Target] Move joint: {0} {1} {2} {3} {4} {5}", joint_pose[0], joint_pose[1], joint_pose[2], joint_pose[3], joint_pose[4], joint_pose[5]);
    rtde_control.moveJ(joint_pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    return true;
}
