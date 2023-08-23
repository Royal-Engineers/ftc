package org.firstinspires.ftc.teamcode.team20936.OpenCv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team20936.Utils.globals;
import org.firstinspires.ftc.teamcode.team20936.ftc.Robot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConeDetection extends OpenCvPipeline {

    private Robot robot;

    private Mat m_mat = null;

    public ConeDetection(){

    }

    @Override
    public Mat processFrame(Mat input){

     return input;
    }

}
