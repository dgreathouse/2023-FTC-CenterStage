package org.firstinspires.ftc.teamcode.Segments;

import org.firstinspires.ftc.teamcode.Lib.Segment;

import java.util.ArrayList;
import java.util.List;

public class Segments {
    public static List<Segment> figure8 = new ArrayList<Segment>(){
        {
            add(new Segment(30.0, 0.5, 0.0, 2000));
            add(new Segment(30.0, 0.5, 0.0, 2000));
            add(new Segment(30.0, 0.5, 0.0, 2000));
            add(new Segment(30.0, 0.5, 0.0, 2000));
            add(new Segment(30.0, 0.5, 0.0, 2000));
            add(new Segment(30.0, 0.5, 0.0, 2000));
            add(new Segment(30.0, 0.5, 0.0, 2000));
        }
    };
}