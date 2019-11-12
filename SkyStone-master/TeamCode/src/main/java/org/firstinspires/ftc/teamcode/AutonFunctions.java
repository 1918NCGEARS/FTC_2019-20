package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutonFunctions {
    // System Variables
    private static ElapsedTime runtime = new ElapsedTime();
    private static HardwareMap hwMap = null;
    private static Telemetry telemetry = null;

    // Drive variables
    private static DcMotor leftDrive = null;
    private static DcMotor rightDrive = null;
    static double leftDrivePower = 0;
    static double rightDrivePower = 0;
    static double leftTotalDist = 0;
    static double rightTotalDist = 0;
    static final double DRIVE_ENC_ERR_RANGE = 50;
    static final double DRIVE_COUNTS_PER_MOTOR_REV = 1120;  // Per Rev Documentation, 1120 counts per output shaft revolution
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 3.5;
    static final double DRIVE_COUNTS_PER_INCH = (DRIVE_COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Elev Variables
    private static DcMotor elev = null;
    static double elevPower = 0;
    static final double ELEV_ENC_ERR_RANGE = 25;
    static final double ELEV_ENC_MID = 300;

    // Arm Variables
    private static DcMotor arm = null;
    static double armPower = 0;

    // Limit switch Variabless
    private static DigitalChannel lowerLimit = null;
    private static DigitalChannel upperLimit = null;

    // Index
    public static int index = 0;

    // Timer reset flag
    private static boolean isNew = true;


    public static void initRobot(HardwareMap ahwMap, Telemetry atelemetry){
        telemetry = atelemetry;
        hwMap = ahwMap;

        // Set drive parameters
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);  // Opposite of teleop becuase of joystick values
        rightDrive.setDirection(DcMotor.Direction.FORWARD);   // Opposite of teleop becuase of joystick values
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set elevator parameters
        elev = hwMap.get(DcMotor.class, "elev");
        elev.setDirection(DcMotor.Direction.FORWARD);
        elev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elev.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set arm parameters
        arm = hwMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set limit switch parameters
        lowerLimit = hwMap.digitalChannel.get("lower_limit");
        upperLimit = hwMap.digitalChannel.get("upper_limit");
        lowerLimit.setMode(DigitalChannel.Mode.INPUT);
        upperLimit.setMode(DigitalChannel.Mode.INPUT);

        index = 0;
        leftTotalDist = 0;
        rightTotalDist = 0;
        isNew = true;

    }

    public static void drive(double leftDistInInch, double rightDistInInch, double motorPwr) {
//        telemetry.addData("Hello", "world");
        int leftDriveEncCount;
        int rightDriveEncCount;
        boolean leftDriveFinished = false;
        boolean rightDriveFinished = false;

        if (isNew) {
            leftTotalDist += leftDistInInch;
            rightTotalDist += rightDistInInch;
            isNew = false;
        }
        leftDriveEncCount = (int) Math.round(leftTotalDist * DRIVE_COUNTS_PER_INCH);
        rightDriveEncCount = (int) Math.round(rightTotalDist * DRIVE_COUNTS_PER_INCH);
        telemetry.addData("left enc", "(%d)", leftDriveEncCount);
        telemetry.addData("right enc", "(%d)", rightDriveEncCount);
        if ((leftDrive.getCurrentPosition() >= leftDriveEncCount - DRIVE_ENC_ERR_RANGE) &&
                (leftDrive.getCurrentPosition() <= leftDriveEncCount + DRIVE_ENC_ERR_RANGE)) {
            leftDrivePower = 0;
            leftDriveFinished = true;
        }
        else{
            leftDrive.setTargetPosition(leftDriveEncCount);
            leftDrivePower = motorPwr;
        }

        if ((rightDrive.getCurrentPosition() >= rightDriveEncCount - DRIVE_ENC_ERR_RANGE) &&
                (rightDrive.getCurrentPosition() <= rightDriveEncCount + DRIVE_ENC_ERR_RANGE)) {
            rightDrivePower = 0;
            rightDriveFinished = true;
        }
        else{
            rightDrive.setTargetPosition(rightDriveEncCount);
            rightDrivePower = motorPwr;
        }

        if (leftDriveFinished && rightDriveFinished) {
            isNew = true;
            index++;
        }
        leftDrive.setPower(leftDrivePower);
        rightDrive.setPower(rightDrivePower);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static void elev(int elevTarget){
        boolean lowerLimitPressed = !lowerLimit.getState();
        boolean upperLimitPressed = !upperLimit.getState();

//        if (lowerLimitPressed) {rightElev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        telemetry.addData("Booleans", "(%b), (%b), (%b), (%b)", elev.getCurrentPosition() >= elevTarget - ELEV_ENC_ERR_RANGE,
                elev.getCurrentPosition() <= elevTarget + ELEV_ENC_ERR_RANGE,
                elevTarget > ELEV_ENC_MID && upperLimitPressed, elevTarget < ELEV_ENC_MID && lowerLimitPressed);

        if (((elev.getCurrentPosition() >= elevTarget - ELEV_ENC_ERR_RANGE)
                && (elev.getCurrentPosition() <= elevTarget + ELEV_ENC_ERR_RANGE))
                || (elevTarget > ELEV_ENC_MID && upperLimitPressed) || (elevTarget < ELEV_ENC_MID && lowerLimitPressed)) {
            elevPower = 0;
            index++;
        }
        else {
            elev.setTargetPosition(elevTarget);
            elevPower = 1;
        }

//        leftElev.setPower(leftElevPower);
        telemetry.addData("Elev Power", "(%.2f)", elevPower);
        elev.setPower(elevPower);
    }

    public static void setElevZeroPwr(DcMotor.ZeroPowerBehavior mode) {
        elev.setZeroPowerBehavior(mode);
    }

    public static void delay(double durationInMilliseconds) {
        if (isNew) {
            runtime.reset();
            isNew = false;
        }
        if (runtime.milliseconds() >= durationInMilliseconds) {
            isNew = true;
            index++;
        }
    }

    public static void allStop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        elev.setPower(0);
        arm.setPower(0);
    }

    public static void sendTelemetry() {
        telemetry.addData("Drive", "left:  (%.2f), right:  (%.2f)", leftDrivePower, rightDrivePower);
        telemetry.addData("Drive Enc", "left:  (%d), right:  (%d)", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.addData("Elev Pwr", elevPower);
        telemetry.addData("Elev Enc", elev.getCurrentPosition());
        telemetry.addData("State", index);
    }

    public static void toggleLatch() {
        // Code to toggle Foundation grabber
    }

}
