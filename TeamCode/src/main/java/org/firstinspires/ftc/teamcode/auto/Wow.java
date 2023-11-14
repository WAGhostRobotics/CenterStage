package org.firstinspires.ftc.teamcode.auto;

import androidx.core.app.NotificationCompat;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.library.vision.AprilTagDetect;
import org.firstinspires.ftc.teamcode.library.vision.PixelDetect;
import org.firstinspires.ftc.teamcode.library.vision.SpikeDetect;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "CV Test")
public class Wow extends LinearOpMode {

    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera
    OpenCvWebcam webcam;
    // OpenCvPipeline pipe;
    AprilTagDetect pipe;

    public void display() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // pipe = new SpikeDetect(true);
        // pipe = new PixelDetect();
        pipe = new AprilTagDetect(0.1016, 822.317, 822.317f, 319.495, 242.502);

        webcam.setPipeline(pipe);
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

    @Override
    public void runOpMode() throws InterruptedException {
        display();
        waitForStart();
        while (!isStarted() && !isStopRequested());

        while(opModeIsActive()) {
//            // Test SpikeDetect pipeline
//            telemetry.addData("Location", pipe.getLocation());
//            telemetry.addData("Left Region", pipe.getLeft());
//            telemetry.addData("Mid Region", pipe.getMid());
//            telemetry.addData("Right Region", pipe.getRight());
//
//            // Test PixelDetect pipeline
//            telemetry.addData("Contour Area", pipe.getContourArea());
//            telemetry.addData("X Center", pipe.getCenter().x);
//            telemetry.addData("Y Center", pipe.getCenter().y);

            // Test AprilTagDetect pipeline
            for (AprilTagDetection detection: pipe.getLatestDetections()) {
                telemetry.addData("ID " + detection.id, detection.center.x);
            }

            telemetry.update();
        }

        webcam.stopStreaming();
    }
}