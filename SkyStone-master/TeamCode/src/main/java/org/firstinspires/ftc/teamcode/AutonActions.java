package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

//import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

/**
 * Created by jtuuk on 11/7/2017.
 */

public class AutonActions {

    public AutonActions(){

    }

    // Auton state enum list
    public enum State {
        DEP_MOVE, RESET_TIME, RESET_ENCODERS, ALL_STOP, TIME_DELAY, LIFT_UP, LIFT_DOWN, INDEP_MOVE
    }

    // Auton variables
    private static ElapsedTime runtime = new ElapsedTime();
    private static HardwareMap hwMap =  null;
    private static Telemetry telemetry = null;

    private static DcMotor leftDrive = null;
    private static DcMotor rightDrive = null;
    private static DcMotor leftIntake = null;
    private static DcMotor rightIntake = null;
    private static DcMotor lift = null;
    private static Servo tilt = null;
    private static Servo arm = null;

    // Set up tilt servo values
    static final int TILT_MAX_POS_DEG = 1260;     // Maximum rotational position
    static final int TILT_MIN_POS_DEG = 0;     // Minimum rotational position
    static final double TILT_MAX_POS = 1.0;     // Maximum rotational position
    static final double TILT_MIN_POS = 0.0;     // Minimum rotational position
    static final int TILT_DOWN_DEG = 383;
    static final int TILT_CARRY_DEG = 450;  //Originally:  475  // After 1st comp:  460
    static final int TILT_UP_DEG = 533;

    static int tiltPosDeg;
    static double tiltPos;

    // Set up arm servo values
    static final int ARM_MAX_POS_DEG = 180;
    static final int ARM_MIN_POS_DEG = 0;
    static final double ARM_MAX_POS = 1.0;
    static final double ARM_MIN_POS = 0.0;
    static final int ARM_DOWN_DEG = 50;
    static final int ARM_UP_DEG = 180;

    static int armPosDeg;
    static double armPos;

    // Set up color sensors
    private static DeviceInterfaceModule dim = null;
    private static ColorSensor rgbSensorFront = null;
    private static final int FRONT_LED_CHANNEL = 4;
    private static float[] hsvValuesFront = {0F, 0F, 0F};
    private static boolean ledOn = TRUE;

    // Set up encoder values
    static final double PI = 3.14159;
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Neverest 40 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = .6666666666;     // This is < 1.0 if geared UP     MULT BY .66 FOR NEW SPROCKETS
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);

    static int leftDriveEncCount;
    static int rightDriveEncCount;
    static final int ENC_ERR_RNG = 50;

    static double leftDistTotal;
    static double rightDistTotal;


    public int runAuton(int index, State state, double leftDriveDist, double rightDriveDist,
                        double leftDrivePwr, double rightDrivePwr){
        // Declare local variables
        double leftDist = 0;
        double rightDist = 0;
        double leftDrivePower = 0;
        double rightDrivePower = 0;
        double leftIntakePower = 0;
        double rightIntakePower = 0;
        double liftPower = 0;

        switch (state) {
            case RESET_TIME:
                runtime.reset();
                index++;
                break;
            case RESET_ENCODERS:
                leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                index++;
                break;
            case DEP_MOVE:
                leftDist = leftDistTotal + leftDriveDist;
                rightDist = rightDistTotal + rightDriveDist;
                telemetry.addData("leftDist", leftDriveDist);
                telemetry.addData("rightDist", rightDriveDist);

                setEncDrive(leftDist, rightDist);

                if (!leftInRange()) {
                    leftDrivePower = leftDrivePwr;
                }
                else {
                    leftDrivePower = 0;
                }

                if (!rightInRange()) {
                    rightDrivePower = rightDrivePwr;
                }
                else {
                    rightDrivePower = 0;
                }

                if (leftInRange() && rightInRange()) {
                    leftDistTotal += leftDriveDist;
                    rightDistTotal += rightDriveDist;
                    index++;
                }
                break;
            case ALL_STOP:
                leftDrivePower = 0;
                rightDrivePower = 0;
                leftIntakePower = 0;
                rightIntakePower = 0;
                liftPower = 0;
                break;
            case TIME_DELAY:
                if (runtime.milliseconds() >= 5000) {
                    index++;
                }
                break;
            case LIFT_UP:
                liftPower = .75;
                if (runtime.milliseconds() >= 250) {
                    liftPower = 0;
                    index++;
                }
                break;
            case INDEP_MOVE:
                leftDist = leftDistTotal + leftDriveDist;
                rightDist = rightDistTotal + rightDriveDist;

                setEncDrive(leftDist, rightDist);

                if (!leftInRange()) {
                    leftDrivePower = leftDrivePwr;
                }
                else {
                    leftDrivePower = 0;
                }

                if (!rightInRange()) {
                    rightDrivePower = rightDrivePwr;
                }
                else {
                    rightDrivePower = 0;
                }

                if (leftInRange() && rightInRange()) {
                    leftDistTotal += leftDriveDist;
                    rightDistTotal += rightDriveDist;
                    index++;
                }
                break;
            case LIFT_DOWN:
                liftPower = -.75;
                if (runtime.milliseconds() >= 250) {
                    liftPower = 0;
                    index++;
                }
                break;
            default:
                leftDrivePower = 0;
                rightDrivePower = 0;
                leftIntakePower = 0;
                rightIntakePower = 0;
                liftPower = 0;
                telemetry.addData("Status", "state error: %s", state);
        }

        // Map servo values
        tiltPos = map(tiltPosDeg, TILT_MIN_POS_DEG, TILT_MAX_POS_DEG, TILT_MIN_POS, TILT_MAX_POS);
        armPos = map(armPosDeg, ARM_MIN_POS_DEG, ARM_MAX_POS_DEG, ARM_MIN_POS, ARM_MAX_POS);

        // Set powers for motors and positions for servos
        leftDrive.setPower(leftDrivePower);
        rightDrive.setPower(rightDrivePower);
        leftIntake.setPower(leftIntakePower);
        rightIntake.setPower(rightIntakePower);
        lift.setPower(liftPower);
        tilt.setPosition(tiltPos);
        arm.setPosition(armPos);
        dim.setDigitalChannelState(FRONT_LED_CHANNEL, ledOn);


        // Send telemetry data
        telemetry.addData("State", "(%s)", state);
        telemetry.addData("Drive", "left: (%.2f), right: (%.2f)", leftDrivePower, rightDrivePower);
        telemetry.addData("Intake", "left: (%.2f), right: (%.2f)", leftIntakePower, rightIntakePower);
        telemetry.addData("Tilt Angle", "Degrees: %d    Pos Value: %.2f", tiltPosDeg, tiltPos);
        telemetry.addData("Arm Angle", "Degrees: %d    Pos Value: %.2f", armPosDeg, armPos);

        return index;
    }

    public static void initRobot(HardwareMap ahwMap, Telemetry atelemetry){
        telemetry = atelemetry;
        hwMap = ahwMap;

        // Set up motors and servos
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftIntake = hwMap.get(DcMotor.class, "left_intake");
        rightIntake = hwMap.get(DcMotor.class, "right_intake");
        lift = hwMap.get(DcMotor.class, "lift");
        tilt = hwMap.get(Servo.class, "tilt");
        arm = hwMap.get(Servo.class, "arm");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDistTotal = 0;
        rightDistTotal = 0;

        telemetry.addData("Status", "Initialized");
    }

    public static boolean leftInRange() {
        if ((leftDrive.getCurrentPosition() >= leftDriveEncCount - ENC_ERR_RNG) &&
                (leftDrive.getCurrentPosition() <= leftDriveEncCount + ENC_ERR_RNG)) {
            return TRUE;
        } else {
            return FALSE;
        }
    }

    public static boolean rightInRange() {
        if ((rightDrive.getCurrentPosition() >= rightDriveEncCount - ENC_ERR_RNG) &&
                (rightDrive.getCurrentPosition() <= rightDriveEncCount + ENC_ERR_RNG)) {
            return TRUE;
        } else {
            return FALSE;
        }
    }

    public static double map(int input, int inMin, int inMax, double outMin, double outMax) {
        double output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
        return output;
    }

    public static void setEncDrive(double leftDriveDist, double rightDriveDist) {
        leftDriveEncCount = (int) Math.round(leftDriveDist * COUNTS_PER_INCH);
        rightDriveEncCount = (int) Math.round(rightDriveDist * COUNTS_PER_INCH);

        leftDrive.setTargetPosition(leftDriveEncCount);
        rightDrive.setTargetPosition(rightDriveEncCount);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
