
#ifndef CAR_INIT_DEF_H__
#define CAR_INIT_DEF_H__

namespace car_setting
{
    static float RemoteForwardSpeed = 0.25;     //前进和后退时的线速度，单位m/s
    static float RemoteLeftSpeed = 0.15;        //左转和右转的线速度，单位m/s  
    static float RemoteLeftAngularSpeed = 0.7;  //左转和右转的角速度，单位rad/s

    static float RemoteForwardSpeed_B = 0.25;     //前进和后退时的线速度，单位m/s
    static float RemoteLeftSpeed_B = 0.15;        //左转和右转的线速度，单位m/s  
    static float RemoteLeftAngularSpeed_B = 0.7;  //左转和右转的角速度，单位rad/s

    static float RemoteForwardSpeed_V = 0.25;     //前进和后退时的线速度，单位m/s
    static float RemoteLeftSpeed_V = 0.15;        //左转和右转的线速度，单位m/s  
    static float RemoteLeftAngularSpeed_V = 0.7;  //左转和右转的角速度，单位rad/s

    static float WHEEL_RADIUS=0.0375;            //轮胎半径,单位m

    static float Wheel_spacing=0.18;            //小车(左右)轮距,单位m
    static float Wheel_axlespacing=0.18;         //小车(前后)轴距,单位m

    static float Wheel_spacing_B=0.144;            //小车(左右)轮距,单位m
    static float Wheel_axlespacing_B=0.16;         //小车(前后)轴距,单位m



}

#endif