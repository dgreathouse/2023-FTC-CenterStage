package org.firstinspires.ftc.teamcode.Lib;

public class MyMath {
    public static double clamp(double _input, double _min, double _max){
        double rtn = _input;
        if(_input < _min) {
            rtn = _min;
        }else if(_input > _max){
            rtn = _max;
        }
        return rtn;
    }
    public static double applyDeadband(double _value, double _deadband, double _magnitude){

        if(Math.abs(_value)> _deadband) {
            if (_magnitude / _deadband > 1.0e12) {
                return _value > 0.0 ? _value - _deadband : _value + _deadband;
            }
            if (_value > 0.0) {
                return _magnitude * (_value - _deadband) / (_magnitude - _deadband);
            }else {
                return _magnitude * (_value + _deadband) / (_magnitude - _deadband);
            }
        }else{
            return 0.0;
        }
    }
}
