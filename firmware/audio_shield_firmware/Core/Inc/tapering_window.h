#ifndef __TAPERING_WINDOW_H 
#define __TAPERING_WINDOW_H

float32_t tukey_window[1024] = {1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000, 1.000000000000000}; 

#endif /* __FFT_BIN_DATA_H */