package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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
    static final double DRIVE_ENC_ERR_RANGE = 100;
    static final double DRIVE_COUNTS_PER_MOTOR_REV = 1120;  // Per Rev Documentation, 1120 counts per output shaft revolution
    static final double DRIVE_COUNTS_PER_MOTOR_REV_20to1 = 560;  // Per Rev Documentation, 1120 counts per output shaft revolution
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 3.5;
    static final double DRIVE_COUNTS_PER_INCH = (DRIVE_COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_COUNTS_PER_INCH_20to1 = (DRIVE_COUNTS_PER_MOTOR_REV_20to1 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Elev Variables
    private static DcMotor elev = null;
    static double elevPower = 0;
    static final double ELEV_ENC_ERR_RANGE = 25;
    static final double ELEV_ENC_MID = 300;

    // Arm Variables
    private static DcMotor arm = null;
    static double armPower = 0;

    // Limit switch Variables
    private static DigitalChannel lowerLimit = null;
    private static DigitalChannel upperLimit = null;

    // Set up hook servo variables
    public static Servo hook = null;
    static final int HOOK_MAX_POS_DEG    =  180;   // Maximum rotational position
    static final int HOOK_MIN_POS_DEG    =  0;     // Minimum rotational position
    static final double HOOK_MAX_POS     =  1.0;   // Maximum rotational position
    static final double HOOK_MIN_POS     =  0.0;   // Minimum rotational position
    static final int HOOK_UP             = 180;    // Hook Up position in degrees
    static final int HOOK_DOWN           = 85;     // Hook down position in degrees

    // Set up claw servo variables
    private static Servo claw = null;
    static final int CLAW_MAX_POS_DEG    =  180;   // Maximum rotational position
    static final int CLAW_MIN_POS_DEG    =  0;     // Minimum rotational position
    static final double CLAW_MAX_POS     =  1.0;   // Maximum rotational position
    static final double CLAW_MIN_POS     =  0.0;   // Minimum rotational position
    static final int CLAW_OPEN           = 170;    // Claw Open position in degrees
    static final int CLAW_CLOSED         = 5;     // Claw Closed position in degrees

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

        // Set elevator parameters
        elev = hwMap.get(DcMotor.class, "elev");
        elev.setDirection(DcMotor.Direction.FORWARD);
        elev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        // Set hook parameters
        hook = hwMap.get(Servo.class,"hook");

        // Set claw parameters
        claw = hwMap.get(Servo.class,"claw");

        index = 0;
        leftTotalDist = 0;
        rightTotalDist = 0;
        isNew = true;

    }

    public static void drive(double leftDistInInch, double rightDistInInch, double leftMotorPwr, double rightMotorPwr) {
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
            leftDrivePower = leftMotorPwr;
        }

        if ((rightDrive.getCurrentPosition() >= rightDriveEncCount - DRIVE_ENC_ERR_RANGE) &&
                (rightDrive.getCurrentPosition() <= rightDriveEncCount + DRIVE_ENC_ERR_RANGE)) {
            rightDrivePower = 0;
            rightDriveFinished = true;
        }
        else{
            rightDrive.setTargetPosition(rightDriveEncCount);
            rightDrivePower = rightMotorPwr;
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

    public static void drive(double leftDistInInch, double rightDistInInch, double motorPwr){
        drive(leftDistInInch, rightDistInInch, motorPwr, motorPwr);
    }

    public static void drive20to1(double leftDistInInch, double rightDistInInch, double leftMotorPwr, double rightMotorPwr) {
        int leftDriveEncCount;
        int rightDriveEncCount;
        boolean leftDriveFinished = false;
        boolean rightDriveFinished = false;

        if (isNew) {
            leftTotalDist -= leftDistInInch;
            rightTotalDist -= rightDistInInch;
            isNew = false;
        }

        leftDriveEncCount = (int) Math.round(leftTotalDist * DRIVE_COUNTS_PER_INCH_20to1);
        rightDriveEncCount = (int) Math.round(rightTotalDist * DRIVE_COUNTS_PER_INCH_20to1);
        telemetry.addData("left enc", "(%d)", leftDriveEncCount);
        telemetry.addData("right enc", "(%d)", rightDriveEncCount);
        if ((leftDrive.getCurrentPosition() >= leftDriveEncCount - DRIVE_ENC_ERR_RANGE) &&
                (leftDrive.getCurrentPosition() <= leftDriveEncCount + DRIVE_ENC_ERR_RANGE)) {
            leftDrivePower = 0;
            leftDriveFinished = true;
        }
        else{
            leftDrive.setTargetPosition(leftDriveEncCount);
            leftDrivePower = leftMotorPwr * 0.7;
        }

        if ((rightDrive.getCurrentPosition() >= rightDriveEncCount - DRIVE_ENC_ERR_RANGE) &&
                (rightDrive.getCurrentPosition() <= rightDriveEncCount + DRIVE_ENC_ERR_RANGE)) {
            rightDrivePower = 0;
            rightDriveFinished = true;
        }
        else{
            rightDrive.setTargetPosition(rightDriveEncCount);
            rightDrivePower = rightMotorPwr * 0.7;
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

    public static void drive20to1(double leftDistInInch, double rightDistInInch, double motorPwr){
        drive20to1(leftDistInInch, rightDistInInch, motorPwr, motorPwr);
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

    public static double map(int input, int inMin, int inMax, double outMin, double outMax) {
        double output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
        return output;
    }

    public static void hookUp() {
        double hookPos = map(HOOK_UP,HOOK_MIN_POS_DEG,HOOK_MAX_POS_DEG,HOOK_MIN_POS,HOOK_MAX_POS);
        hook.setPosition(hookPos);
        index++;
    }

    public static void hookDown() {
        double hookPos = map(HOOK_DOWN,HOOK_MIN_POS_DEG,HOOK_MAX_POS_DEG,HOOK_MIN_POS,HOOK_MAX_POS);
        hook.setPosition(hookPos);
        index++;
    }

    public static void clawOpen() {
        double clawPos = map(CLAW_OPEN,CLAW_MIN_POS_DEG,CLAW_MAX_POS_DEG,CLAW_MIN_POS,CLAW_MAX_POS);
        claw.setPosition(clawPos);
        index++;
    }

    public static void clawClose() {
        double clawPos = map(CLAW_CLOSED,CLAW_MIN_POS_DEG,CLAW_MAX_POS_DEG,CLAW_MIN_POS,CLAW_MAX_POS);
        claw.setPosition(clawPos);
        index++;
    }
}
