package org.firstinspires.ftc.teamcode.library.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SpikeDetect extends OpenCvPipeline {
    Mat mat = new Mat();
    Scalar lowHSV;
    Scalar highHSV;
    private double contourArea;
    private Point propCenter = new Point(-1, -1);
    private boolean left;

    //HSV Ranges for red and blue
    //TODO: tune if necessary
    public SpikeDetect(boolean redAlliance, boolean left) {
        if (redAlliance) {
            lowHSV = new Scalar(-10,40, 16);
            highHSV = new Scalar(15, 255, 255);
        } else {
            lowHSV = new Scalar(105, 112, 64);
            highHSV = new Scalar(120, 255, 255);
        }
        this.left = left;
    }

    //locations
    public enum Location {
        LEFT,
        MID,
        RIGHT
    }

    //location
    private SpikeDetect.Location location;

    //constants
    //TODO: change constants, add independent gap for right if necessary
    final static double LEFT_SPIKE_GAP_X = 535;
    final static double CONTOUR_AREA_THRESHOLD = 20000;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //creates submatrices
        Core.inRange(mat, lowHSV, highHSV, mat);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        int largestContourIdx = 0;

        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //skips contour calculations if there are no contours
        if (contours.size() == 0) {
            contourArea = 0;
            propCenter = new Point(-1, -1);
            hierarchy.release();

            //stuff to make the gray scale appear on on robot phone
            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

            return input;
        }

        //finds the largest contour
        for (int contourIdx=0; contourIdx < contours.size(); contourIdx++) {
            if (Imgproc.contourArea(contours.get(contourIdx)) > Imgproc.contourArea(contours.get(largestContourIdx))) {
                largestContourIdx = contourIdx;
            }
        }

        contourArea = Imgproc.contourArea(contours.get(largestContourIdx));

        //gets moments and center from contour
        Moments moments = Imgproc.moments(contours.get(largestContourIdx));
        propCenter = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());

        hierarchy.release();

        //select location region
        location = left ? leftGetLocation() : rightGetLocation();
        //draw rectangle on region if it is within view
        if (propCenter.x >= 0 && contourArea >= CONTOUR_AREA_THRESHOLD) {
            Imgproc.drawContours(input, contours, largestContourIdx, new Scalar(0, 255, 0), 3);
            Imgproc.circle(input, propCenter, 5, new Scalar(0,255,0), 3);
        }

        return input;
    }

    //some methods to get constants and vars
    public SpikeDetect.Location getLocation() {
        return location;
    }

    //use when aligning with left side of tile
    public SpikeDetect.Location leftGetLocation() {
        if (contourArea < CONTOUR_AREA_THRESHOLD || propCenter.x < 0) {
            return SpikeDetect.Location.RIGHT;
        }
        else if (propCenter.x >= LEFT_SPIKE_GAP_X) {
            return SpikeDetect.Location.MID;
        }
        return SpikeDetect.Location.LEFT;
    }

    //use when aligning with right side of tile
    public SpikeDetect.Location rightGetLocation() {
        if (contourArea < CONTOUR_AREA_THRESHOLD || propCenter.x < 0) {
            return SpikeDetect.Location.LEFT;
        }
        else if (propCenter.x <= 1280 - LEFT_SPIKE_GAP_X) {
            return SpikeDetect.Location.MID;
        }
        return SpikeDetect.Location.RIGHT;
    }

    public double getContourArea() { return contourArea; }

    public Point getPropCenter() { return propCenter; }
}
