package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.DriveStyle;

@TeleOp(name = "Driver Oriented", group = "competition")
public class DriverOrientedTeleop extends TeleOpParent {

    @Override
    public void runOpMode() throws InterruptedException {
        super.type = DriveStyle.DriveType.DRIVERORIENTED;
        super.runOpMode();
    }
}
