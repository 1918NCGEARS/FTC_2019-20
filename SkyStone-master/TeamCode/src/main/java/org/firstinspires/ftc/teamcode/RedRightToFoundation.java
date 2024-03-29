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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.AutonFunctions.allStop;
import static org.firstinspires.ftc.teamcode.AutonFunctions.delay;
import static org.firstinspires.ftc.teamcode.AutonFunctions.drive;
import static org.firstinspires.ftc.teamcode.AutonFunctions.index;
import static org.firstinspires.ftc.teamcode.AutonFunctions.sendTelemetry;
import static org.firstinspires.ftc.teamcode.VuforiaSkyStone.Pos;
import static org.firstinspires.ftc.teamcode.VuforiaSkyStone.findSkystone;
import static org.firstinspires.ftc.teamcode.VuforiaSkyStone.getSkystonePos;
import static org.firstinspires.ftc.teamcode.VuforiaSkyStone.initVuforia;
import static org.firstinspires.ftc.teamcode.VuforiaSkyStone.targetsSkyStone;


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

@Autonomous(name="Red Right To Foundation", group="Iterative Opmode")
@Disabled
public class RedRightToFoundation extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.setMsTransmissionInterval(0);   // Default is 250
        telemetry.update();
//        initRobot(hardwareMap, telemetry);
        initVuforia(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.setMsTransmissionInterval(250);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        switch(index) {
            case 0:
                drive(14,14,0.3);
                break;
            case 1:
                delay(1000);
                break;
            case 2:
                findSkystone();
                if (getSkystonePos() == Pos.LEFT) {
                    index++;
                }
                else if (getSkystonePos() == Pos.RIGHT) {
                    index = 5;
                }
                else {
                    index = 7;
                }
                break;
            case 3:    // Cases 3 - 8 are the LEFT path
                drive(0,5.5,0.3);
                break;
            case 4:
                drive(20,20,0.3);
                break;
            case 5:
                // pick up block
                break;
            case 6:
                drive(-4,-4,0.3);
                break;
            case 7:
                drive(16.5,0,0.3);
                break;
            case 8:
                drive(84.3,84.3,0.3);
                index = 20;
                break;
            case 9:    // Cases 9 - 14 are the RIGHT path
                drive(5.5,0,0.3);
                break;
            case 10:
                drive(20,20,0.3);
                break;
            case 11:
                // pick up block
                break;
            case 12:
                drive(-4,-4,0.3);
                break;
            case 13:
                drive(0,27.5,0.3);
                break;
            case 14:
                drive(76.3,76.3,0.3);
                index = 20;
                break;
            case 15:    // Cases 15 - 19 are the MIDDLE path
                drive(18,18,0.3);
                break;
            case 16:
                // pick up block
                break;
            case 17:
                drive(-4,-4,.3);
                break;
            case 18:
                drive(0,22,0.3);
                break;
            case 19:
                drive(80.3,80.3,0.5);
                break;
            case 20:
                drive(0,22,0.3);
                break;
            case 21:
                drive(4,4,0.3);
                break;
            case 22:
//                toggleLatch();
                break;
            case 23:
                drive(-32.75,-32.75,0.3);
                break;
            case 24:
//                toggleLatch();
                break;
            case 25:
                drive(0,22,0.3);
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
        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();        // Move to function after testing
    }

}
