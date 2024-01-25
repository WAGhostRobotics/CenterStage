package org.firstinspires.ftc.teamcode.core;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.component.Imu;
import org.firstinspires.ftc.teamcode.component.Intake;
import org.firstinspires.ftc.teamcode.component.Launcher;
import org.firstinspires.ftc.teamcode.component.MainSail;
import org.firstinspires.ftc.teamcode.component.Slides;
import org.firstinspires.ftc.teamcode.component.Webcam;

public class Gnocchi {

    public static HardwareMap hardwareMap;

    public static DcMotorEx frontLeft;
    public static DcMotorEx frontRight;
    public static DcMotorEx backLeft;
    public static DcMotorEx backRight;

    public static Webcam webcam;

    public static MainSail mainSail; // outtake
    public static Intake intake;
    public static Slides slides;
//    public static Launcher launcher;

    public static void init(HardwareMap hwMap, boolean redAlliance) {
        hardwareMap = hwMap;

        frontLeft = hwMap.get(DcMotorEx.class, "lf");
        frontRight = hwMap.get(DcMotorEx.class, "rf");
        backLeft = hwMap.get(DcMotorEx.class, "lb");
        backRight = hwMap.get(DcMotorEx.class, "rb");

        mainSail = new MainSail();
        mainSail.init(hwMap);

        intake = new Intake();
        intake.init(hwMap);

        slides = new Slides();
        slides.init(hwMap, false);

        webcam = new Webcam(redAlliance);

//        launcher = new Launcher();
//        launcher.init(hwMap);
    }

    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
     * simple shorthand for the operating-system-provided {@link Thread#sleep(long) sleep()} method.
     *
     * @param milliseconds amount of time to sleep, in milliseconds
     * @see Thread#sleep(long)
     */
    public static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
