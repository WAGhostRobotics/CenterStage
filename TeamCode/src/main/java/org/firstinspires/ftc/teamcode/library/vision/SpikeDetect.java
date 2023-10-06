package org.firstinspires.ftc.teamcode.library.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class SpikeDetect extends OpenCvPipeline {
    Mat mat = new Mat();

    Scalar lowHSV;
    Scalar highHSV;

    public SpikeDetect(boolean redAlliance) {
        if (redAlliance) {
            lowHSV = new Scalar(235, 200, 200);
            highHSV = new Scalar(20, 255, 255);
        } else {
            lowHSV = new Scalar(150, 200, 200);
            highHSV = new Scalar(190, 255, 255);
        }
    }

    //locations
    public enum Location {
        LEFT,
        RIGHT,
        MID
    }

    double leftValue;
    double midValue;

    //location
    private Location location = Location.RIGHT;

    //Region of interest coordinates
    public static final Rect MID_ROI = new Rect(
            new Point(5, 80),
            new Point(85, 150));
    public static final Rect LEFT_ROI = new Rect(
            new Point(95, 80),
            new Point(175, 150));

    //Threshold to determine if there is an object
    final static double PERCENT_COLOR_THRESHOLD = 0.1;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //creates submatrices
        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat mid = mat.submat(MID_ROI);
        Mat left = mat.submat(LEFT_ROI);

        //finds the raw value that fits in HSV range
        midValue = Core.sumElems(mid).val[0] / MID_ROI.area() / 255;
        leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;

        mid.release();
        left.release();

        //determines if its above threshold
        boolean objectMid = midValue > PERCENT_COLOR_THRESHOLD;
        boolean objectLeft = leftValue > PERCENT_COLOR_THRESHOLD;

        //chooses location
        if (objectMid) {
            location = Location.MID;
        } else if (objectLeft) {
            location = Location.LEFT;
        } else {
            location = Location.RIGHT;
        }

        //stuff to make the gray scale appear on on robot phone
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorObject = new Scalar(255, 0, 0);
        Scalar colorNoObject = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, MID_ROI, location == Location.MID ? colorNoObject : colorObject);
        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT ? colorNoObject : colorObject);

        return mat;
    }

    //some methods to get constants and vars
    public Location getLocation() {
        return location;
    }

    public String getLeft() {
        return Math.round(leftValue * 100) + "%";
    }

    public String getMid() {
        return Math.round(midValue * 100) + "%";
    }
}