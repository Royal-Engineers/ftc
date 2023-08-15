package org.firstinspires.ftc.teamcode.team20936.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class pipeline1 extends OpenCvPipeline {

     Mat m_mat = new Mat();
    static final Rect m_ROI = new Rect(new Point(1, 500), new Point(1,500));


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, m_mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(7 , 125, 110);
        Scalar highHSV = new Scalar(16, 255, 255);

      Core.inRange(m_mat, lowHSV, highHSV, m_mat);
/*
        Mat ROIMat = m_mat.submat(m_ROI);

        double value = Core.sumElems(ROIMat).val[0]/m_ROI.area();*/
        /*ROIMat.release();*/

        return m_mat;
    }
}
