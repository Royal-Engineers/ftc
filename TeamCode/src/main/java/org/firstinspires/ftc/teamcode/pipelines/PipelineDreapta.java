package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class PipelineDreapta extends OpenCvPipeline {

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


    public PipelineDreapta(Telemetry telemetry, team culoare)
    {
        this.telemetry = telemetry;        this.culoare = culoare;

    }


    public void init(){
        Zone = 2;

        lowHSV = new Scalar(175, 110, 100);
        highHSV = new Scalar(180, 255, 255);

        lowHSV1 = new Scalar(0, 110, 100);
        highHSV1 = new Scalar(7, 255, 255);

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

        Point P1L =  new Point(0, 0), P1R = new Point(width/3,height);
        Point P2L =  new Point(width/3+1, 0), P2R = new Point(width/3 * 2,height);
        Point P3L = new Point(width/3*2, 0), P3R = new Point(width,height);

        final Rect rect_left = new Rect(P1L, P1R);
        final Rect rect_middle = new Rect(P2L, P2R);
        final Rect rect_right = new Rect(P3L, P3R);


        Imgproc.rectangle(input, P1L, P1R, new Scalar(0, 255, 0, 255), 5);
        Imgproc.rectangle(input, P2L, P2R, new Scalar(0, 255, 0, 255), 5);
        Imgproc.rectangle(input, P3L, P3R, new Scalar(0, 255, 0, 255), 5);

        Mat MatLeft = m_mat.submat(rect_left);
        Mat MatMiddle = m_mat.submat(rect_middle);
        Mat MatRight = m_mat.submat(rect_right);

        int valLeft = (int)Core.sumElems(MatLeft).val[0]/255;
        int valMiddle = (int)Core.sumElems(MatMiddle).val[0]/255;
        int valRight = (int)Core.sumElems(MatRight).val[0]/255;

        telemetry.addData("valLeft: ", valLeft);
        telemetry.addData("valMiddle: ", valMiddle);
        telemetry.addData("valRight: ", valRight);
        int vmax = 1000;

         if ( valMiddle > vmax ){
            region_of_interest = regions.middle; }
        else if ( valRight > vmax){
            region_of_interest = regions.right;}
        else
            region_of_interest = regions.left;

        telemetry.addData("ROI: ", region_of_interest);
        telemetry.update();
        return input;

    }
}
