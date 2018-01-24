package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Nithya on 10/29/2017.
 * This class implements Golden Gears robot.
 * Any OpMode can use this class to control the hardware.
 */

public class NS_Robot_GoldenGears {
    // Hardware Map of robot
    private HardwareMap hardwareMap = null;

    private DcMotor driveLeftMotor = null;
    private DcMotor driveRightMotor = null;
    private double drivePower = 0.0;
    private double driveDistance = 0.0;

    private DcMotor armElevationMotor = null;
    private double armMotorPower = 0.0;

    private Servo clawLeftServo = null;
    private Servo clawRightServo = null;
    private double clawPosition = Servo.MAX_POSITION;
    private final double clawThreshold = 0.05;

    private Servo jewelServo = null;
    private ColorSensor jewelSensor = null;

    private final double encoderTotalPulses = 1440;
    private final double turnsShaftToWheel = 2;
    private final double wheelCircumference = 4 * Math.PI;
    private final double encoderPulsesPerInch = encoderTotalPulses
            * turnsShaftToWheel
            / wheelCircumference;

    private ModernRoboticsI2cGyro driveGyro = null;


    // The IMU sensor object
    private BNO055IMU imu;
    private final int imuPollInterval = 50; // In milliseconds

    // State used for updating telemetry
    private Orientation orientationAngles;
    // private Acceleration gravity;


    NS_Robot_GoldenGears(HardwareMap hm) {
        hardwareMap = hm;

        driveLeftMotor = hardwareMap.dcMotor.get("driveLeftMotor");
        driveRightMotor = hardwareMap.dcMotor.get("driveRightMotor");
        armElevationMotor = hardwareMap.dcMotor.get("armElevationMotor");

        clawLeftServo = hardwareMap.servo.get("clawLeftServo");
        clawRightServo = hardwareMap.servo.get("clawRightServo");

        jewelServo = hardwareMap.servo.get("jewelServo");
        jewelSensor = hardwareMap.colorSensor.get("jewelSensor");

        // driveGyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("driveGyro");
        //driveGyro.calibrate();

        this.ResetIMU();
        this.Reset();
    }

    public void Reset() {
        this.ResetDrive();
        this.ResetGlyphHand();
        this.ResetJewelHand();
        this.jewelSensor.enableLed(false);
    }

    public void ResetDrive() {
        this.ResetDriveMotors();
        this.ResetDriveEncoders();
    }

    public void ResetDriveMotors() {
        driveRightMotor.setDirection(DcMotor.Direction.REVERSE);

        driveLeftMotor.setPower(0);
        driveRightMotor.setPower(0);
    }

    public void ResetDriveEncoders() {
        driveLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void ResetGlyphHand() {
        this.ResetGlyphArm();
        this.ResetGlyphClaw();
    }

    public void ResetGlyphClaw() {
        clawRightServo.setDirection(Servo.Direction.REVERSE);

        // clawLeftServo.scaleRange(Servo.MIN_POSITION, Servo.MAX_POSITION);
        // clawRightServo.scaleRange(Servo.MIN_POSITION, Servo.MAX_POSITION);

        // clawRightServo.setPosition(0.6);
        // clawLeftServo.setPosition(0.6);
    }

    public void ResetGlyphArm() {
        armElevationMotor.setPower(0);
        // armElevationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void ResetJewelHand() {
        jewelServo.setDirection(Servo.Direction.REVERSE);
        this.ResetJewelArm();
    }

    public void ResetJewelArm() {
        // jewelServo.setPosition(Servo.MAX_POSITION);
    }

    public void ResetIMU() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void Start() {
        this.Reset();
        imu.startAccelerationIntegration(new Position(), new Velocity(), imuPollInterval);
        armElevationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Stop() {
        this.Reset();
        armElevationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public boolean IsBusy() {
        // return true if robot is idle
        return (this.IsDriving() || armElevationMotor.isBusy());
        // return false; // !driveGyro.isCalibrating();
    }

    public boolean IsDriving() {
        /* This is important to have the && logic instead of || logic
         * for the determination that desired travel was achieved
         * for the given angle.
         */
        return (driveLeftMotor.isBusy() && driveRightMotor.isBusy());
    }

    public void DriveTank(double leftPower, double rightPower) {
        // Normalize speeds if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        driveLeftMotor.setPower(leftPower);
        driveRightMotor.setPower(rightPower);
    }

    public void RotateArm(double armPower) {
        armElevationMotor.setPower(armPower);
    }

    public void ActuateClaw(double advance) {
        double position = clawLeftServo.getPosition() + advance;
        PositionClaw(position);
    }

    public void ActuateJewelArm(double delta) {
        double jewelPosition = jewelServo.getPosition() + delta;
        jewelPosition = Range.clip(jewelPosition, 0.5, Servo.MAX_POSITION);
        jewelServo.setPosition(jewelPosition);
    }

    public void PositionClaw(double position) {
        clawPosition = Range.clip(position, Servo.MIN_POSITION, 0.6);

        clawLeftServo.setPosition(clawPosition);
        clawRightServo.setPosition(clawPosition);
    }

    public void SwitchLightState(boolean state) {
        jewelSensor.enableLed(state);
    }

    public void OnDriveDistance(double distance, double power) {
        driveLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Determine new target position, and pass to motor controller
         * for all given angles. Alternate is to calculate the target
         * position based on the drive angle and set them as position.
         */
        int moveCounts = (int) (distance * encoderPulsesPerInch);
        int newLeftTarget = driveLeftMotor.getCurrentPosition() + moveCounts;
        int newRightTarget = driveRightMotor.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        driveLeftMotor.setTargetPosition(newLeftTarget);
        driveRightMotor.setTargetPosition(newRightTarget);

        driveLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        driveDistance = distance;
        drivePower = Range.clip(Math.abs(power), 0.0, 1.0);
        this.DriveTank(drivePower, drivePower);
    }

    public void positionJewelArm(double position) {
        jewelServo.setPosition(Range.clip(position, 0.45, Servo.MAX_POSITION));
    }

    public boolean isJewelRed() {
        return (jewelSensor.red() > jewelSensor.blue());
    }

    public boolean KnockRed() {
        jewelServo.setPosition(0.5);

        if (jewelSensor.red() > jewelSensor.blue()) {
            return true;
        } else {
            return false;
        }
    }

    public boolean KnockBlue() {
        jewelServo.setPosition(0.5);

        if (jewelSensor.blue() > jewelSensor.red()) {
            return true;
        } else {
            return false;
        }
    }

    public void OnDriveAngle(double angle, double driveCoeff) {
        double max;
        double error;
        double steer;
        double leftPower;
        double rightPower;

        // adjust relative speed based on heading error.
        error = angularError(angle);
        steer = angularSteer(error, driveCoeff);

        // if driving in reverse, the motor correction also needs to be reversed
        if (driveDistance < 0)
            steer *= -1.0;

        leftPower = drivePower - steer;
        rightPower = drivePower + steer;

        this.DriveTank(leftPower, rightPower);

        // Display drive status for the driver.
        /*
        telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
        telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
        telemetry.addData("Actual",  "%7d:%7d",      robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
        telemetry.update();
        */
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param power     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param turnCoeff Proportional Gain coefficient
     * @return
     */
    public boolean OnDriveTurn(double power, double angle, double turnCoeff, double threshold) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftPower;
        double rightPower;

        // determine turn power based on +/- error
        error = angularError(angle);

        if (Math.abs(error) <= threshold) {
            steer = 0.0;
            leftPower = 0.0;
            rightPower = 0.0;
            onTarget = true;
        } else {
            steer = angularSteer(error, turnCoeff);
            rightPower = power * steer;
            leftPower = -rightPower;
        }

        // Send desired speeds to motors.
        this.DriveTank(leftPower, rightPower);

        // Display it for the driver.
        /*
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        */
        return onTarget;
    }

    /**
     * angularError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double angularError(double targetAngle) {

        double robotError;

        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        orientationAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // gravity  = imu.getGravity();

        // calculate error in -179 to +180 range  (
        // robotError = targetAngle - driveGyro.getIntegratedZValue();
        robotError = targetAngle - orientationAngles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    private double angularSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
