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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.Hardware;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auton Encoder Drive", group="EncoderDrive")
//@Disabled
public class AutonDriveEncoder extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot   = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.4 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at fL: %7d fR: %7d bL: %7d bR: %7d",
                          robot.fLMotor.getCurrentPosition(),
                          robot.fRMotor.getCurrentPosition(),
                          robot.bLMotor.getCurrentPosition(),
                          robot.bRMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
//        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED,'f', 18, 10);

        sleep(500);

        encoderDrive(DRIVE_SPEED,'f', 10, 10);
        encoderDrive(DRIVE_SPEED,'b', 10, 10);
        encoderDrive(DRIVE_SPEED,'f', 5,  10);
        encoderDrive(DRIVE_SPEED,'r', 10, 10);
        encoderDrive(DRIVE_SPEED,'l', 10, 10);

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, char direction, double inches, double timeoutS) {

        int newFrontLeftTarget = 0;
        int newBackLeftTarget = 0;
        int newFrontRightTarget = 0;
        int newBackRightTarget = 0;

        boolean directionIsTrue = true;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            switch (direction) {
                case 'f':
                    newFrontLeftTarget = robot.fLMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                    newFrontRightTarget = robot.fRMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                    newBackLeftTarget = robot.bLMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                    newBackRightTarget = robot.bRMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                    robot.fLMotor.setTargetPosition(newFrontLeftTarget);
                    robot.fRMotor.setTargetPosition(newFrontRightTarget);
                    robot.bLMotor.setTargetPosition(newBackLeftTarget);
                    robot.bRMotor.setTargetPosition(newBackRightTarget);
                    break;
                case 'b':
                    newFrontLeftTarget = robot.fLMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
                    newFrontRightTarget = robot.fRMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
                    newBackLeftTarget = robot.bLMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
                    newBackRightTarget = robot.bRMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
                    robot.fLMotor.setTargetPosition(newFrontLeftTarget);
                    robot.fRMotor.setTargetPosition(newFrontRightTarget);
                    robot.bLMotor.setTargetPosition(newBackLeftTarget);
                    robot.bRMotor.setTargetPosition(newBackRightTarget);
                    break;
                case 'l':
                    newFrontLeftTarget = robot.fLMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
                    newFrontRightTarget = robot.fRMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                    newBackLeftTarget = robot.bLMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                    newBackRightTarget = robot.bRMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
                    robot.fLMotor.setTargetPosition(newFrontLeftTarget);
                    robot.fRMotor.setTargetPosition(newFrontRightTarget);
                    robot.bLMotor.setTargetPosition(newBackLeftTarget);
                    robot.bRMotor.setTargetPosition(newBackRightTarget);
                    break;
                case 'r':
                    newFrontLeftTarget = robot.fLMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                    newFrontRightTarget = robot.fRMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
                    newBackLeftTarget = robot.bLMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
                    newBackRightTarget = robot.bRMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                    robot.fLMotor.setTargetPosition(newFrontLeftTarget);
                    robot.fRMotor.setTargetPosition(newFrontRightTarget);
                    robot.bLMotor.setTargetPosition(newBackLeftTarget);
                    robot.bRMotor.setTargetPosition(newBackRightTarget);
                    break;
                default:
                    directionIsTrue = false;
            }
            // Determine new target position, and pass to motor controller

            // Turn On RUN_TO_POSITION
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            if (directionIsTrue) {
                robot.fLMotor.setPower(Math.abs(speed));
                robot.fRMotor.setPower(Math.abs(speed));
                robot.bLMotor.setPower(Math.abs(speed));
                robot.bRMotor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (directionIsTrue) &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.fLMotor.isBusy() && robot.fRMotor.isBusy() && robot.bLMotor.isBusy() && robot.bRMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Target Positions",  "Running to fL: %7d fR: %7d bL: %7d bR: %7d",
                                            newFrontLeftTarget,
                                            newFrontRightTarget,
                                            newBackLeftTarget,
                                            newBackRightTarget);
                telemetry.addData("Current Positions",  "Running at fL: %7d fR: %7d bL: %7d bR: %7d",
                                            robot.fLMotor.getCurrentPosition(),
                                            robot.fRMotor.getCurrentPosition(),
                                            robot.bLMotor.getCurrentPosition(),
                                            robot.bRMotor.getCurrentPosition());
                telemetry.update();

                if (Math.abs(newFrontLeftTarget - robot.fLMotor.getCurrentPosition()) < 50 ||
                        Math.abs(newFrontRightTarget - robot.fRMotor.getCurrentPosition()) < 50 ||
                        Math.abs(newBackLeftTarget - robot.bLMotor.getCurrentPosition()) < 50 ||
                        Math.abs(newBackRightTarget - robot.bRMotor.getCurrentPosition()) < 50) {
                    break;
                }
            }

            // Stop all motion;
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);

            // Turn off RUN_TO_POSITIOn

            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move
        }
    }
}
