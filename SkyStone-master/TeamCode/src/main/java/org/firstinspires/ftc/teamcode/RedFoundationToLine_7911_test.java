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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.AutonFunctions.allStop;
import static org.firstinspires.ftc.teamcode.AutonFunctions.clawOpen;
import static org.firstinspires.ftc.teamcode.AutonFunctions.delay;
import static org.firstinspires.ftc.teamcode.AutonFunctions.drive20to1;
import static org.firstinspires.ftc.teamcode.AutonFunctions.hookDown;
import static org.firstinspires.ftc.teamcode.AutonFunctions.hookUp;
import static org.firstinspires.ftc.teamcode.AutonFunctions.index;
import static org.firstinspires.ftc.teamcode.AutonFunctions.initRobot;
import static org.firstinspires.ftc.teamcode.AutonFunctions.sendTelemetry;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="7911t - Red Foundation To Line", group="Iterative Opmode")
//@Disabled
public class RedFoundationToLine_7911_test extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

//    private static Servo hook = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.setMsTransmissionInterval(0);   // Default is 250
        telemetry.update();
        initRobot(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.setMsTransmissionInterval(100);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        hookUp();
        clawOpen();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        index = 0;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch(index) {
            case 0:
                drive20to1(34,34,0.8);
                break;
            case 1:
                hookDown();
                break;
            case 2:
                delay(500);
                break;
            case 3:
                drive20to1(-28,-40,0.8,1);
                delay(10000);
                break;
            case 4:
                hookUp();
                break;
            case 5:
                delay(500);
                break;
//            case 6:
//                drive20to1(-5, -5,1);
//                break;
            case 6:
                drive20to1(-8, 28, 1);
                delay(8000);
                break;
            case 7:
                drive20to1(30, 30, 0.9, 0.8);
                break;
            default:
                allStop();
        }
        sendTelemetry();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
