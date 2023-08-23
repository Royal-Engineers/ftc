package org.firstinspires.ftc.teamcode.team20936.OpenCv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team20936.Utils.globals;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class GetFrameImage extends OpenCvPipeline {

    public double proc = 0;

    public double posDown = 0;
    public boolean hasCon = false;

    public boolean picioare = false;
    public class Pair<T,U> {
        public final T key;
        public final U value;

        public Pair(T key, U value) {
            this.key = key;
            this.value = value;
        }

        public T getKey() {
            return this.key;
        }

        public U getValue() {
            return this.value;
        }
    }


    Mat m_mat = new Mat();//

    Telemetry telemetry;
    private  Scalar lowHSV;
    private  Scalar highHSV;

    private Scalar lowHSV1;
    private Scalar highHSV1;
    static final Rect m_ROI = new Rect(new Point(120, 70), new Point(200,170));

    private int LeftBound, UpperBound, RightBound, LowerBound;

    private final int BoundSize = 5;
    private final int BoundDensity = 50;

    private final int ConeDistance = 20;

    private ArrayList<Pair<Point, Point>> ConeList = new ArrayList<>();
    public GetFrameImage(Telemetry t){telemetry = t;}

    public int width = -1, height = -1;

    private boolean isUp(){

        double dif = LowerBound - UpperBound;
        Rect r1 = new Rect(new Point(LeftBound, UpperBound), new Point(RightBound, UpperBound + dif/4));
        Rect r2 = new Rect(new Point(LeftBound, LowerBound - dif/4), new Point(RightBound, LowerBound));

        Mat m1 = m_mat.submat(r1);
        Mat m2 = m_mat.submat(r2);

        double val1 = -1, val2 = -1;

        val1 = (int)Core.sumElems(m1).val[0]/255;
        val2 = (int)Core.sumElems(m2).val[0]/255;
        if ( val2 * 0.3 < val1 && val1 < val2 * 0.6 )
            return true;
        return false;

    }
    private void FindBounds(int col){

        assert (width != -1 && height != -1);

        Rect vef = new Rect(new Point(col, 0), new Point( col + BoundSize, m_mat.rows()));
        if ( (int)Core.sumElems( m_mat.submat(vef)).val[0]/255 < BoundDensity - 10)
            return;

        UpperBound = -1;
        LowerBound = -1;
        RightBound = -1;
        LeftBound = -1;

        for ( int i = col; i <= width - BoundSize; ++i )
        {
            RightBound = i;
            Rect bound = new Rect(new Point(i, 0), new Point( i + BoundSize, height));
            Mat b_mat = m_mat.submat(bound);

            int val = (int)Core.sumElems(b_mat).val[0]/255;
            if ( val < 50 )
                break;
        }

        for ( int i = col + 5; i >= BoundSize; --i )
        {
            LeftBound = i;
            Rect bound = new Rect(new Point(i-BoundSize, 0), new Point(i, height));
            Mat b_mat = m_mat.submat(bound);

            int val = (int)Core.sumElems(b_mat).val[0]/255;
            if ( val < 50 )
                break;
        }
        if ( RightBound == -1 || LeftBound == -1 ) {
            return;
        }

        for ( int i = height; i > BoundSize; --i )
        {
            LowerBound = i;
            Rect bound = new Rect(new Point(LeftBound, i - BoundSize), new Point(RightBound, i));
            Mat b_mat = m_mat.submat(bound);
            int val = (int)Core.sumElems(b_mat).val[0]/255;

            if ( val > (RightBound - LeftBound)*0.8 )
                break;
        }

        if ( LowerBound == -1 )
            return;

        for ( int i = LowerBound - 20 ; i > BoundSize; --i )
        {
            UpperBound = i;
            Rect bound = new Rect(new Point(LeftBound, i - BoundSize), new Point(RightBound, i));
            Mat b_mat = m_mat.submat(bound);

            int val = (int)Core.sumElems(b_mat).val[0]/255;

            if ( val < 10)
                break;
        }
    }

    private void ProcessImage(){
        for ( int i = 1; i < m_mat.cols() - 5; i++)
        {
            Rect bound = new Rect(new Point(i, 0), new Point(i+5, height));

            Mat b_mat = m_mat.submat(bound);
            int val = (int)Core.sumElems(b_mat).val[0]/255;


            if ( val > BoundDensity ) {
                FindBounds(i);
                if( RightBound != -1)
                    i = RightBound;
                i+=ConeDistance;

                boolean Ebun;
                if ( RightBound != -1 && LeftBound != -1 && UpperBound != -1 && LowerBound != -1 && RightBound - LeftBound > 5 /*&& isUp()*/) {
                    ConeList.add(new Pair<Point, Point>(new Point(LeftBound, UpperBound), new Point(RightBound, LowerBound)));
                }
            }
        }
    }
    private void init()
    {
        width = m_mat.cols();
        height = m_mat.rows();

        lowHSV = new Scalar(175, 110, 100);
        highHSV = new Scalar(180, 255, 255);

        lowHSV1 = new Scalar(0, 110, 100);
        highHSV1 = new Scalar(7, 255, 255);

        ConeList.clear();
    }

    private void draw(Mat input)
    {
        int max = 0, idmax = 0;
        for ( int i = 0; i < ConeList.size(); ++i )
            if ( (int)ConeList.get(i).value.y > max ) {
                max = (int) ConeList.get(i).value.y;
                idmax = i;
            }
        for ( int i = 0; i < ConeList.size(); ++i )
            if ( i == idmax ) {
                Imgproc.rectangle(input, ConeList.get(i).key, ConeList.get(i).value, new Scalar(40, 252, 3));
                posDown = ConeList.get(i).value.y;
            }
            else
                Imgproc.rectangle(input,ConeList.get(i).key, ConeList.get(i).value, new Scalar(255, 243, 0));

        if ( ConeList.isEmpty()) {
            proc = 0;
            hasCon = false;
            return;

        }
        hasCon = true;
        Point p1 = ConeList.get(idmax).key;
        Point p2 = ConeList.get(idmax).value;

        int mid = width/2;
        int pos = (int)(p1.x + p2.x)/2;

        int procentaj = pos * 100 / mid;
        procentaj-=100;
        proc = procentaj;
    }

    @Override
    public Mat processFrame(Mat input) {
        globals.CameraFrame = input;
        init();

        Imgproc.cvtColor(input, m_mat, Imgproc.COLOR_RGB2HSV);

        Mat mask1 = new Mat(), mask2 = new Mat();
        Core.inRange(m_mat, lowHSV, highHSV, mask1);
        Core.inRange(m_mat, lowHSV1, highHSV1, mask2);

        Core.bitwise_xor(mask1, mask2, m_mat);






       /* Mat edges = new Mat();
        Imgproc.Canny(m_mat, edges, 0, 0);*/
        ProcessImage();
        draw(input);



        return input;
    }
}
