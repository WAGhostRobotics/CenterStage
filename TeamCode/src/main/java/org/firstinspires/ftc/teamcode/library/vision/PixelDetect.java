package org.firstinspires.ftc.teamcode.library.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PixelDetect extends OpenCvPipeline {
    Mat mat = new Mat();

    //Threshold to determine if there is an object
    final static double PERCENT_COLOR_THRESHOLD = 0.1;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //Hue, Saturation, Value ranges
        Scalar lowHSV = new Scalar(12, 180, 130);//change color HSV
        Scalar highHSV = new Scalar(26, 255, 255);//change color HSV


        //creats submatrices
        Core.inRange(mat, lowHSV, highHSV, mat);


        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for ( int contourIdx=0; contourIdx < contours.size(); contourIdx++ )
        {
            //Minimun size allowed for consideration
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(contourIdx).toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f,true)*0.02;
            Imgproc.approxPolyDP(contour2f,approxCurve,approxDistance,true);

            //convert to MatofPoint
            MatOfPoint point = new MatOfPoint(approxCurve.toArray());

            //get boundingrect from contour
            Rect rect = Imgproc.boundingRect(point);


            Imgproc.rectangle(mat,rect, new Scalar(255, 0, 0),3);
            //bisa Imgproc.rectangle(mRgba, rect.tl(), rect.br(), new Scalar(255, 0, 0),1, 8,0);

        }

        hierarchy.release();


        //stuff to make the gray scale appear on on robot phone
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        return mat;
    }
}