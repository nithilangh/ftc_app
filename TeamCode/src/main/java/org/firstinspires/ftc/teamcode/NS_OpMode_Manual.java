package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Nithya on 11/24/2017.
 */
@TeleOp(name = "NS: OpMode Manual", group = "Competition")

public class NS_OpMode_Manual extends LinearOpMode {
    private final class Regulator {
        static final double FULL = 1.0;
        static final double HALF = 0.5;
        static final double FOURTH = 0.25;
        static final double TENTH = 0.10;
    }
    private enum DriveMode {
        RC_DRIVE, TANK_DRIVE
    }
    NS_Robot_GoldenGears GGRobot = null;
    DriveMode driveMode = DriveMode.RC_DRIVE;
    double driveRegulator = Regulator.FOURTH;
    double armRegulator = Regulator.FOURTH;

    @Override
    public void runOpMode() throws InterruptedException {
        GGRobot = new NS_Robot_GoldenGears(hardwareMap);
        telemetry.addData("Status", "Robot Initalized");
        telemetry.addData("Drive Mode", "Initalized to RC Drive");
        telemetry.addData("Drive Speed", "Initialized to FOURTH");

        waitForStart();
        GGRobot.Start();
        telemetry.addData("Status", "Robot Started");

        while (opModeIsActive()) {
            if (gamepad1.right_stick_button == true) {
                driveMode = DriveMode.RC_DRIVE;
                telemetry.addData("Drive Mode", "Changed to RC Drive");
            }
            else if (gamepad1.left_stick_button == true) {
                driveMode = DriveMode.TANK_DRIVE;
                telemetry.addData("Drive Mode", "Changed to Tank Drive");
            }

            if (gamepad1.a == true) {
                driveRegulator = Regulator.FULL;
                telemetry.addData("Drive Speed", "FULL");
            }
            else if (gamepad1.b == true) {
                driveRegulator = Regulator.HALF;
                telemetry.addData("Drive Speed", "HALF");
            }
            else if (gamepad1.x == true) {
                driveRegulator = Regulator.FOURTH;
                telemetry.addData("Drive Speed", "FOURTH");
            }
            else if (gamepad1.y == true) {
                driveRegulator = Regulator.TENTH;
                telemetry.addData("Drive Speed", "TENTH");
            }

            double leftPower = 0.0;
            double rightPower = 0.0;
            if (driveMode == DriveMode.RC_DRIVE) {
                double drive = -gamepad1.left_stick_y;
                double turn  =  gamepad1.right_stick_x;
                leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
                rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            }
            else {
                leftPower = -gamepad1.left_stick_y;
                rightPower = -gamepad1.right_stick_y;
            }
            GGRobot.DriveTank(leftPower * driveRegulator,
                    rightPower * driveRegulator);

            double armPower = -gamepad2.left_stick_y;
            GGRobot.RotateArm(armPower * armRegulator);

            double advance = 0.005;
            if (gamepad2.right_stick_x < 0) {
                GGRobot.ActuateClaw(advance);
            }
            else if (gamepad2.right_stick_x > 0) {
                GGRobot.ActuateClaw(-advance);
            }

            if (gamepad2.dpad_up == true) {
                GGRobot.ActuateJewelArm(0.0005);
            }
            else if (gamepad2.dpad_down == true) {
                GGRobot.ActuateJewelArm(-0.0005);
            }

            if (gamepad2.x == true) {
                GGRobot.positionJewelArm(0.5);
            }
            else if (gamepad2.y == true) {
                GGRobot.positionJewelArm(Servo.MAX_POSITION);
            }

            if (gamepad1.right_bumper == true) {
                GGRobot.SwitchLightState(true);
            }
            else if (gamepad1.right_bumper == false) {
                GGRobot.SwitchLightState(false);
            }
        }

        GGRobot.Stop();
        while (GGRobot.IsBusy())  {
            sleep(50);
            idle();
        }
    }
}
