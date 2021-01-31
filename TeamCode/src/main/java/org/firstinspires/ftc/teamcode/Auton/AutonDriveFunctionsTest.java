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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutonDriving;
import org.firstinspires.ftc.teamcode.AutonDrivingWIP;
import org.firstinspires.ftc.teamcode.Hardware;

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

@Autonomous(name="Auton Driving Test With Function", group="FullAuton")
//@Disabled
public class AutonDriveFunctionsTest extends AutonDrivingWIP {





    /* Declare OpMode members. */
    //Hardware robot = new Hardware();   // Use a Pushbot's hardware

    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.4;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        //BasicDriveTest();

        //GyroDriveTest();

        //strafe(25, gyroDriveSpeedStrafe, false, 0, 500, 0, 0);

        gyroStrafeTest(10, 0, false, gyroDriveSpeed, 1);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    public void BasicDriveTest()
    {
        drive('f', 1.5);

        stopMotor();
        sleep(500);

        turn('r', 1.0);

        stopMotor();
        sleep(500);

        drive('b', 1.0);

        stopMotor();
    }

    public void gyroStrafeTest (double distance, double target, boolean initBoost, double speed, double speedMult)
    {
        stopAndReset();

        int fLTarget;
        int fRTarget;
        int bLTarget;
        int bRTarget;

        int fLInit = robot.fLMotor.getCurrentPosition();
        int fRInit = robot.fRMotor.getCurrentPosition();
        int bLInit = robot.bLMotor.getCurrentPosition();
        int bRInit = robot.bRMotor.getCurrentPosition();

        //int     newRightTarget;
        int     moveCounts;
        double  max;
        double angle = readAngle(xyz);
        double  error = angle - target;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            fLTarget = (robot.fLMotor.getCurrentPosition() + moveCounts);
            fRTarget = (robot.fRMotor.getCurrentPosition()) + moveCounts;
            bLTarget = (robot.bLMotor.getCurrentPosition() + moveCounts);
            bRTarget = (robot.bLMotor.getCurrentPosition() + moveCounts);

            // Set Target and Turn On RUN_TO_POSITION
            robot.fLMotor.setTargetPosition(fLTarget);
            robot.fRMotor.setTargetPosition(-fRTarget);
            robot.bLMotor.setTargetPosition(-bLTarget);
            robot.bRMotor.setTargetPosition(bRTarget);


            robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            //gyroDriveSpeed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.fLMotor.setPower(speed);
            robot.fRMotor.setPower(speed);
            robot.bLMotor.setPower(speed);
            robot.bRMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.fLMotor.isBusy() && robot.fRMotor.isBusy() && robot.bLMotor.isBusy() && robot.bRMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = readAngle(xyz) - target;

                if(Math.abs(error) < gyroDriveThreshold)
                {
                    error = 0;
                }
                steer = -getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed + steer;
                rightSpeed = speed - steer;

                //makes first 100 counts faster bc drive takes a lot of time to accelerate
                if(initBoost) {
                    if (Math.abs(robot.fLMotor.getCurrentPosition() - fLInit) < 100 && Math.abs(robot.bLMotor.getCurrentPosition() - bLInit) < 100)
                    {
                        leftSpeed += gyroDriveInitBoost;
                    }
                    if (Math.abs(robot.fRMotor.getCurrentPosition() - fRInit) < 100 && Math.abs(robot.bRMotor.getCurrentPosition() - bRInit) < 100)
                    {
                        rightSpeed += gyroDriveInitBoost;
                    }
                }
                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftSpeed *= speedMult;
                rightSpeed *= speedMult;

                robot.fLMotor.setPower(rightSpeed);
                robot.fRMotor.setPower(-rightSpeed);
                robot.bLMotor.setPower(-leftSpeed);
                robot.bRMotor.setPower(leftSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      fLTarget,  fRTarget, bLTarget, bRTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.fLMotor.getCurrentPosition(),
                        robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.addData("TargetAngle", target);
                telemetry.addData("CurrentAngle", readAngle(xyz));
                telemetry.update();
            }

            // Stop all motion;
            normalDrive(0, 0);

            //correct for drift during drive
            //turnToPosition(-angle, "z", turnSpeed, 2);
            turnToPosition(0, xyz, turnSpeed, 10, false);

            // Turn off RUN_TO_POSITION
            stopAndReset();
        }
    }

    public void GyroDriveTest()
    {
        //gyroDrive(24, .25, false, gyroDriveSpeed, 1, 5);
        stopMotor();

//        sleep(1000);
//
//        gyroDrive(30, .5, false, gyroDriveSpeed, 1, 5);
//        stopMotor();
//
//        sleep(1000);
//
//        gyroDrive(10, 90, false, gyroDriveSpeed, 1, 5);
//        stopMotor();
    }

    public void drive(char direction, double time) {
        if (direction == 'f') {
            robot.fLMotor.setPower(FORWARD_SPEED);
            robot.fRMotor.setPower(FORWARD_SPEED);
            robot.bLMotor.setPower(FORWARD_SPEED);
            robot.bRMotor.setPower(FORWARD_SPEED);
        }
        else if (direction == 'l') {
            robot.fLMotor.setPower(FORWARD_SPEED);
            robot.fRMotor.setPower(-FORWARD_SPEED);
            robot.bLMotor.setPower(-FORWARD_SPEED);
            robot.bRMotor.setPower(FORWARD_SPEED);
        }
        else if (direction == 'r') {
            robot.fLMotor.setPower(-FORWARD_SPEED);
            robot.fRMotor.setPower(FORWARD_SPEED);
            robot.bLMotor.setPower(FORWARD_SPEED);
            robot.bRMotor.setPower(-FORWARD_SPEED);
        }
        else if (direction == 'b') {
            robot.fLMotor.setPower(-FORWARD_SPEED);
            robot.fRMotor.setPower(-FORWARD_SPEED);
            robot.bLMotor.setPower(-FORWARD_SPEED);
            robot.bRMotor.setPower(-FORWARD_SPEED);
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    public void turn(char direction, double time) {
        if (direction == 'r') {
            robot.fLMotor.setPower(FORWARD_SPEED);
            robot.fRMotor.setPower(-FORWARD_SPEED);
            robot.bLMotor.setPower(FORWARD_SPEED);
            robot.bRMotor.setPower(-FORWARD_SPEED);
        }
        else if (direction == 'l') {
            robot.fLMotor.setPower(-FORWARD_SPEED);
            robot.fRMotor.setPower(FORWARD_SPEED);
            robot.bLMotor.setPower(-FORWARD_SPEED);
            robot.bRMotor.setPower(FORWARD_SPEED);
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    public void stopMotor() {
        robot.fLMotor.setPower(0);
        robot.fRMotor.setPower(0);
        robot.bLMotor.setPower(0);
        robot.bRMotor.setPower(0);
    }
}
