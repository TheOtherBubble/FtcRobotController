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

package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.MecanumHardware;

import java.util.concurrent.TimeoutException;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Encoder Auton Driving Test", group="FullAuton")
//@Disabled
public class EncoderAutonDriveTest extends LinearOpMode {

    /* Declare OpMode members. */
    MecanumHardware robot = new MecanumHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.2;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        robot.fLMotor.setPower(FORWARD_SPEED);
        robot.fRMotor.setPower(FORWARD_SPEED);
        robot.bLMotor.setPower(FORWARD_SPEED);
        robot.bRMotor.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            //telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Runtime", runtime);
            telemetry.addData("Encoder port 1 back left",  robot.bLMotor.getCurrentPosition());
            telemetry.addData("Encoder port 2 front right", robot.fRMotor.getCurrentPosition());
            telemetry.addData("Encoder port 3 back right", robot.bRMotor.getCurrentPosition());
            telemetry.update();
        }

        robot.fLMotor.setPower(0);
        robot.fRMotor.setPower(0);
        robot.bLMotor.setPower(0);
        robot.bRMotor.setPower(0);

        //Right encoder - back right motor
        //Middle encoder - front right motor
        //Left encoder - back left motor

        sleep(50000);
//
//        // Step 2:  Spin right for 1.3 seconds
//        robot.fLMotor.setPower(FORWARD_SPEED);
//        robot.fRMotor.setPower(-FORWARD_SPEED);
//        robot.bLMotor.setPower(FORWARD_SPEED);
//        robot.bRMotor.setPower(-FORWARD_SPEED);
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
//            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        robot.fLMotor.setPower(0);
//        robot.fRMotor.setPower(0);
//        robot.bLMotor.setPower(0);
//        robot.bRMotor.setPower(0);
//
//        sleep(500);
//
//        // Step 3:  Drive Backwards for 1 Second
//        robot.fLMotor.setPower(-FORWARD_SPEED);
//        robot.fRMotor.setPower(-FORWARD_SPEED);
//        robot.bLMotor.setPower(-FORWARD_SPEED);
//        robot.bRMotor.setPower(-FORWARD_SPEED);
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 4:  Stop and close the claw.
//        robot.fLMotor.setPower(0);
//        robot.fRMotor.setPower(0);
//        robot.bLMotor.setPower(0);
//        robot.bRMotor.setPower(0);

//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        sleep(1000);
    }
}
