package org.firstinspires.ftc.teamcode.team20936.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

enum Cazuri
{
    Dicuri0,
    Dicuri1,
    Discuri4
}
public class pipeline1 extends OpenCvPipeline {

     Mat m_mat = new Mat();//

    Telemetry telemetry;
    static final Rect m_ROI = new Rect(new Point(120, 70), new Point(200,170));

    public pipeline1(Telemetry t){telemetry = t;}
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, m_mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(0, 50, 80);
        Scalar highHSV = new Scalar(20, 155, 255);

        Core.inRange(m_mat, lowHSV, highHSV, m_mat);

        Mat ROIMat = m_mat.submat(m_ROI);


        int pixels = (int)Core.sumElems(ROIMat).val[0]/255;

/*        telemetry.addData("White pixels in ROI: ", pixels);
        telemetry.addData("Black pixels in ROI: ", m_ROI.area()- pixels);*/

        Cazuri tag;

        if ( pixels < 50 )
            tag = Cazuri.Dicuri0;
        else if ( pixels < 300 )
            tag = Cazuri.Dicuri1;
        else
            tag = Cazuri.Discuri4;

        String sCaz = "";

        switch ( tag )
        {
            case Dicuri0:
                sCaz = "0 discuri";
                break;
            case Dicuri1:
                sCaz = "1 disc";
                break;
            case Discuri4:
                sCaz = "4 discuri";
                break;
            default:
                sCaz = "Failed to detect case!";
        }

        telemetry.addData(sCaz, "");

        Mat edges = new Mat();
        Imgproc.Canny(m_mat, edges, 0, 0);

        return m_mat;
    }
}
