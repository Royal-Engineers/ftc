package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class Pipeline extends OpenCvPipeline {

    public enum team{
        rosu, albastru
    }

    public enum regions{
        none, left, middle, right
    }

    public static regions region_of_interest = regions.none;
    private int width, height;
    private Telemetry telemetry;

    private Scalar lowHSV, highHSV, lowHSV1, highHSV1;

    private Mat m_mat = new Mat();



    private team culoare;


    public Pipeline(Telemetry telemetry, team culoare)
    {
        this.telemetry = telemetry;        this.culoare = culoare;

    }


    public void init(){


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

        final Rect rect_left = new Rect(new Point(0, 0), new Point(width/3,height));
        final Rect rect_middle = new Rect(new Point(width/3+1, 0), new Point(width/3 * 2,height));
        final Rect rect_right = new Rect(new Point(width/3*2, 0), new Point(width,height));



        Mat MatLeft = m_mat.submat(rect_left);
        Mat MatMiddle = m_mat.submat(rect_middle);
        Mat MatRight = m_mat.submat(rect_right);

        int valLeft = (int)Core.sumElems(MatLeft).val[0]/255;
        int valMiddle = (int)Core.sumElems(MatMiddle).val[0]/255;
        int valRight = (int)Core.sumElems(MatRight).val[0]/255;

        telemetry.addData("valLeft: ", valLeft);
        telemetry.addData("valMiddle: ", valMiddle);
        telemetry.addData("valRight: ", valRight);
        int vmax = 0;
        if ( valLeft > vmax ){
            region_of_interest = regions.left; vmax = valLeft;}
         if ( valMiddle > vmax ){
            region_of_interest = regions.middle; vmax = valMiddle;}
        if ( valRight > vmax){
            region_of_interest = regions.right;vmax = valRight;}

        telemetry.addData("ROI: ", region_of_interest);
        telemetry.update();
        return input;

    }
}
