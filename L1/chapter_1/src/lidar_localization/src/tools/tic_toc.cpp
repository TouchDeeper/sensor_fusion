//
// Created by wang on 20-1-4.
//
#include "lidar_localization/tools/tic_toc.h"
namespace td{
    void TicToc::delay_s(int second){
        sleep(second);
    }
    void TicToc::delay_s(float second){
        usleep(1000000 * second);
    }
}

