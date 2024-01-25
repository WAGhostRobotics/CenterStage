package org.firstinspires.ftc.teamcode.component;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.teamcode.library.vision.AprilTagDetect;
import org.firstinspires.ftc.teamcode.library.vision.PixelDetect;
import org.firstinspires.ftc.teamcode.library.vision.SpikeDetect;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class Webcam {
    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera
    OpenCvWebcam webcam;

    SpikeDetect spikePipe;
    // AprilTagDetect aprilTagPipe;
    PixelDetect pixelPipe;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 822.317;
    double fy = 822.317f;
    double cx = 319.495;
    double cy = 242.502;

    // UNITS ARE METERS
    double tagsize = 0.166;

    AprilTagDetection tagOfInterest = null; // tag you want

    AprilTagLocation aprilTagLocation = null; // tag in general

    SpikeDetect.Location location = null;

    private boolean redAlliance;

    public Webcam(boolean redAlliance){
        this.redAlliance = redAlliance;
    }

    public void init(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        // pipelines
        spikePipe = new SpikeDetect(redAlliance);
        // aprilTagPipe = new AprilTagDetect(tagsize, fx, fy, cx, cy);
        pixelPipe = new PixelDetect();

        webcam.setPipeline(spikePipe);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed", "");
                telemetry.update();
            }
        });
    }

    public void scanForTags(AprilTagDetect aprilTagPipe){
        webcam.setPipeline(aprilTagPipe);
        ArrayList<AprilTagDetection> currentDetections = aprilTagPipe.getLatestDetections();

//        if (currentDetections.size() != 0) {
//
//            for (AprilTagDetection tag : currentDetections) {
//                if (tag.id == AprilTagLocation.ONE.getId()) {
//                    aprilTagLocation = AprilTagLocation.ONE;
//                    tagOfInterest = tag;
//                    break;
//                } else if (tag.id == AprilTagLocation.TWO.getId()) {
//                    aprilTagLocation = AprilTagLocation.TWO;
//                    tagOfInterest = tag;
//                    break;
//                } else if (tag.id == AprilTagLocation.THREE.getId()) {
//                    aprilTagLocation = AprilTagLocation.THREE;
//                    tagOfInterest = tag;
//                    break;
//                }
//            }
//        }

        // hopefully works
        if (currentDetections.size() != 0) {
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == aprilTagLocation.getId()) {
                    tagOfInterest = tag;
                }
            }
        }
    }

    public void scanForLocation() {
        //webcam.setPipeline(spikePipe);
        location = spikePipe.getLocation();

        switch (location) {
            case LEFT:
                aprilTagLocation = redAlliance ? AprilTagLocation.ONE : AprilTagLocation.FOUR;
                break;
            case MID:
                aprilTagLocation = redAlliance ? AprilTagLocation.TWO : AprilTagLocation.FIVE;
                break;
            case RIGHT:
                aprilTagLocation = redAlliance ? AprilTagLocation.THREE : AprilTagLocation.SIX;
                break;
        }
    }

    public AprilTagLocation getAprilTagLocation() {
        return aprilTagLocation;
    }

    public SpikeDetect.Location getLocation(){
        return spikePipe.getLocation();
    }

    public AprilTagDetection getTagOfInterest(){
        return tagOfInterest;
    }

    public double getTranslationX() {
        return tagOfInterest.pose.x;
    }

    public double getTranslationY() {
        return tagOfInterest.pose.y;
    }

    public double getTranslationZ() {
        return tagOfInterest.pose.z;
    }

    public MatrixF getRotation() {
        return tagOfInterest.pose.R;
    }

    public void stopStreaming(){
        webcam.stopStreaming();
    }

    public enum AprilTagLocation {
        ONE (1),
        TWO (2),
        THREE (3),
        FOUR (4),
        FIVE (5),
        SIX (6);

        int id;

        AprilTagLocation(int id){
            this.id = id;
        }

        int getId(){
            return id;
        }
    }
}