/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="7911 - Teleop 2019", group="Iterative Opmode")
//@Disabled
public class Teleop2019_7911 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor elev = null;
    private DcMotor arm = null;
    private Servo hook = null;
    private Servo claw = null;

    // Set up hook servo values
    static final int HOOK_MAX_POS_DEG    =  180;   // Maximum rotational position
    static final int HOOK_MIN_POS_DEG    =  0;     // Minimum rotational position
    static final double HOOK_MAX_POS     =  1.0;   // Maximum rotational position
    static final double HOOK_MIN_POS     =  0.0;   // Minimum rotational position
    static final int HOOK_UP             = 180;    // Hook Up position in degrees
    static final int HOOK_DOWN           = 85;     // Hook down position in degrees

    int hookPosDeg;
    double hookPos;

    // Set up claw servo values
    static final int CLAW_MAX_POS_DEG    =  180;   // Maximum rotational position
    static final int CLAW_MIN_POS_DEG    =  0;     // Minimum rotational position
    static final double CLAW_MAX_POS     =  1.0;   // Maximum rotational position
    static final double CLAW_MIN_POS     =  0.0;   // Minimum rotational position
    static final int CLAW_OPEN           = 170;    // Claw Open position in degrees
    static final int CLAW_CLOSED         = 5;     // Claw Closed position in degrees

    int clawPosDeg;
    double clawPos;

    // Set up arm encoder values
//    static final int ARM_LEFT   = 68;              // Arm left encoder position
//    static final int ARM_RIGHT  = -68;             // Arm right encoder position
//    static final int ARM_CENTER = 0;               // Arm center encoder position
//    static final int ARM_ERROR = 5;
//    int armTarget;

    private DigitalChannel lowerLimit = null;
    private DigitalChannel upperLimit = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        elev = hardwareMap.get(DcMotor.class, "elev");
        arm = hardwareMap.get(DcMotor.class,"arm");
        hook = hardwareMap.get(Servo.class,"hook");
        claw = hardwareMap.get(Servo.class,"claw");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        elev.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);
        elev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lowerLimit = hardwareMap.digitalChannel.get("lower_limit");
        upperLimit = hardwareMap.digitalChannel.get("upper_limit");
        lowerLimit.setMode(DigitalChannel.Mode.INPUT);
        upperLimit.setMode(DigitalChannel.Mode.INPUT);

//        rightElev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Encoder on Left Elev only

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        hookPosDeg = HOOK_UP;
        clawPosDeg = CLAW_OPEN;
        hookPos = map(hookPosDeg, HOOK_MIN_POS_DEG, HOOK_MAX_POS_DEG, HOOK_MIN_POS, HOOK_MAX_POS);
        clawPos = map(clawPosDeg, CLAW_MIN_POS_DEG, CLAW_MAX_POS_DEG, CLAW_MIN_POS, CLAW_MAX_POS);
        hook.setPosition(hookPos);
        claw.setPosition(clawPos);

//        armTarget = ARM_CENTER;
//        arm.setTargetPosition(armTarget);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setPower(.3);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        arm.setPower(0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftDrivePower;
        double rightDrivePower;
        double driveMultiplier;
        double slowMultiplier;
        double fastMultiplier;
        double BASE_DRIVE_SPEED = 0.5;
        double elevPower;
        double armPower;
        boolean lowerLimitPressed;
        boolean upperLimitPressed;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        armPower = -gamepad2.right_stick_x * 0.3;

        lowerLimitPressed = !lowerLimit.getState();
        upperLimitPressed = !upperLimit.getState();

        if((lowerLimitPressed && -gamepad2.left_stick_y < 0) || (upperLimitPressed && -gamepad2.left_stick_y > 0)){
            elevPower = 0;
        }
        else {
                elevPower = -gamepad2.left_stick_y * 1;
        }

        if (gamepad2.left_bumper){
            hookPosDeg = HOOK_UP;
        }
        else if (gamepad2.left_trigger > 0){
            hookPosDeg = HOOK_DOWN;
        }

        if (gamepad2.right_bumper){
            clawPosDeg = CLAW_OPEN;
        }
        else if (gamepad2.right_trigger > 0){
            clawPosDeg = CLAW_CLOSED;
        }

        if (gamepad1.left_trigger > 0){
            slowMultiplier = 0.15;
        }
        else
            slowMultiplier = 0;

        if (gamepad1.right_trigger > 0){
            fastMultiplier = 0.35;
        }
        else
            fastMultiplier = 0;

        driveMultiplier = BASE_DRIVE_SPEED + slowMultiplier + fastMultiplier;
        leftDrivePower  = -gamepad1.left_stick_y * driveMultiplier;
        rightDrivePower = -gamepad1.right_stick_y * driveMultiplier;

        // Send power to actuators
        leftDrive.setPower(leftDrivePower);
        rightDrive.setPower(rightDrivePower);
        elev.setPower(elevPower);
//        arm.setTargetPosition(armTarget);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(armPower);
        hookPos = map(hookPosDeg, HOOK_MIN_POS_DEG, HOOK_MAX_POS_DEG, HOOK_MIN_POS, HOOK_MAX_POS);
        clawPos = map(clawPosDeg, CLAW_MIN_POS_DEG, CLAW_MAX_POS_DEG, CLAW_MIN_POS, CLAW_MAX_POS);
        hook.setPosition(hookPos);
        claw.setPosition(clawPos);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive", "left (%.2f), right (%.2f)", leftDrivePower, rightDrivePower);
        telemetry.addData( "Elev", "power (%.2f)", elevPower);
        telemetry.addData("Limit", "lower (%b), upper (%b)", lowerLimitPressed, upperLimitPressed);
        telemetry.addData("Arm Enc", arm.getCurrentPosition());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    double map(int input, int inMin, int inMax, double outMin, double outMax) {
        double output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
        return output;
    }
    double mapd(double input, double inMin, double inMax, double outMin, double outMax) {
        double output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
        return output;
    }
}
