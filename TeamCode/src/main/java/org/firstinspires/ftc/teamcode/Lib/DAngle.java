package org.firstinspires.ftc.teamcode.Lib;
public enum DAngle {
    ang_0,
    ang_60,
    ang_120,
    ang_180,
    ang_240,
    ang_300;

//public enum DAngle {
//    ANG_0,
//    ANG_45,
//    ANG_90,
//    ANG_135,
//    ANG_180,
//    ANG_225,
//    ANG_270,
//    ANG_315;
//
    public static double getAngle(DAngle _ang){
        double ang = 0;
        switch (_ang){

            case ang_0:
                ang = 0.0;
                break;
            case ang_60:
                ang = 60.0;
                break;
            case ang_120:
                ang = 120.0;
                break;
            case ang_180:
                ang = 180.0;
                break;
            case ang_240:
                ang = 240.0;
                break;
            case ang_300:
                ang = 300.0;
                break;
        }
        return ang;
    }
}
