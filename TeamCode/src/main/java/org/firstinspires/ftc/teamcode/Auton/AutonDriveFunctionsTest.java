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

package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import org.firstinspires.ftc.teamcode.src.main.java.org.firstinspires.ftc.teamcode.Auton.AutonDrivingDustBowlRefugee;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.AutonDriving;


import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.DEFAULT;

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

@Autonomous(name="Auton Driving Test With Function", group="Pushbot")
//@Disabled
public class AutonDriveFunctionsTest extends AutonDriving {

    /* Declare OpMode members. */
    //Hardware robot = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    private String ringStack = "";
    private float detectionTime = 1 * 1000;


    static final double     FORWARD_SPEED = 0.4;

    @Override
    public void runOpMode() {

        robot.init((hardwareMap));

        BNO055IMU.Parameters p = new BNO055IMU.Parameters();
        p.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        p.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        p.calibrationDataFile = "BNO055IMUCalibration.json";
        p.loggingEnabled = true;
        p.loggingTag = "IMU";
        p.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(p);

        robot.fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lateral motors
        robot.bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initVuforia();
        initTfod();
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        if (tfod != null) {
            tfod.activate();
        }



        if (tfod != null) {
            tfod.shutdown();
        }
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            ringStack = recognition.getLabel();
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.addData("millis", runtime.milliseconds());
                        telemetry.update();
                        if(runtime.milliseconds() > detectionTime)
                        {
                            break;
                        }
                    }
                    else if(runtime.milliseconds() > detectionTime)
                    {
                        break;
                    }
                }
                else if(runtime.milliseconds() > detectionTime)
                {
                    break;
                }
            }
        }
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        runtime.reset();
        switch (ringStack)
        {
            case "Single":
            {
                telemetry.addData("Single Case", null);
                gyroDrive(5, 1, false, gyroDriveSpeed, 1, 5);
                break;
            }
            case "Quad":
            {
                telemetry.addData("Quad Case", null);
                gyroDrive(10, 1, false, gyroDriveSpeed, 1, 5);
                break;
            }
            default:
            {
                telemetry.addData("Default Case", null);
                gyroDrive(15, 1, false, gyroDriveSpeed, 1, 5);
                break;
            }
        }
        telemetry.update();



//        // Step 1:  Drive forward for 3 seconds
//        drive('f', 1.5);
//
//        // Step 2:  Spin right for 1.3 seconds
//        turn('r', 1.0);
//
//        // Step 3:  Drive Backwards for 1 Second
//        drive('b', 1.0);

        // Step 4:  Stop and close the claw.
        stopMotor();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
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
