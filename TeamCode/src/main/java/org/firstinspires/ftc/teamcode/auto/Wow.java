package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.library.vision.PixelDetect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Wow extends LinearOpMode {

    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera
    OpenCvCamera webcam;

    public void display() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        PixelDetect pipe = new PixelDetect();

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
        waitForStart();
        display();
        while (!isStarted() && !isStopRequested()) {

        }

        webcam.stopStreaming();
    }
}