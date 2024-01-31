package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class PipelineStanga extends OpenCvPipeline {

    static public int Zone = 2;
    public enum team{
        rosu, albastru
    }

    public enum regions{
        none, left, middle, right
    }

    public static regions region_of_interest = regions.right;
    private int width, height;
    private Telemetry telemetry;

    private Scalar lowHSV, highHSV, lowHSV1, highHSV1;

    private Mat m_mat = new Mat();



    private team culoare;


    public PipelineStanga(Telemetry telemetry, team culoare)
    {
        this.telemetry = telemetry;        this.culoare = culoare;

    }


    public void init(){
        Zone = 2;

        if ( culoare == team.rosu )
        {
            lowHSV = new Scalar(175, 110, 100);
            highHSV = new Scalar(180, 255, 255);

            lowHSV1 = new Scalar(0, 110, 100);
            highHSV1 = new Scalar(7, 255, 255);
            return;
        }

        lowHSV = new Scalar(100, 100, 50);
        highHSV = new Scalar(130, 255, 255);

        lowHSV1 = new Scalar(0, 0, 0);
        highHSV1 = new Scalar(0, 0, 0);

    }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, m_mat, Imgproc.COLOR_RGB2HSV);
        width = m_mat.cols();
        height = m_mat.rows();
        init();

        Mat mask1 = new Mat(), mask2 = new Mat();
        Core.inRange(m_mat, lowHSV, highHSV, mask1);
        Core.inRange(m_mat, lowHSV1, highHSV1, mask2);

        Core.bitwise_xor(mask1, mask2, m_mat);

        Point P1L =  new Point(0, height/2 - 100), P1R = new Point(width/2 - 50,height - 100);
        Point P2L =  new Point(width/2+1, height/2 - 100), P2R = new Point(width - 50,height - 100);

        final Rect rect_left = new Rect(P1L, P1R);
        final Rect rect_right = new Rect(P2L, P2R);


        Imgproc.rectangle(input, P1L, P1R, new Scalar(0, 255, 0, 255), 5);
        Imgproc.rectangle(input, P2L, P2R, new Scalar(0, 255, 0, 255), 5);


        Mat MatLeft = m_mat.submat(rect_left);
        Mat MatRight = m_mat.submat(rect_right);

        int valLeft = (int)Core.sumElems(MatLeft).val[0]/255;
        int valRight = (int)Core.sumElems(MatRight).val[0]/255;

        telemetry.addData("valLeft: ", valLeft);
        telemetry.addData("valRight: ", valRight);

        if ( valLeft > 2000 ){
            region_of_interest = regions.left; }
        else if ( valRight > 1500){
            region_of_interest = regions.middle;}
        else
            region_of_interest = regions.right;

        telemetry.addData("ROI: ", region_of_interest);
        telemetry.update();
        return input;

    }
}
