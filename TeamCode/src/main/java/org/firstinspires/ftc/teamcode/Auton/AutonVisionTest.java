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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware;

import java.util.List;

@Autonomous(name="Auton Vision Test", group="FullAuton")
//@Disabled
public class AutonVisionTest extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.4;

    private boolean objectInVision = false;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private String ringLabel = null;

    public static final String VUFORIA_KEY =
            "AYy6NYn/////AAABmTW3q+TyLUMbg/IXWlIG3BkMMq0okH0hLmwj3CxhPhvUlEZHaOAmESqfePJ57KC2g6UdWLN7OYvc8ihGAZSUJ2JPWAsHQGv6GUAj4BlrMCjHvqhY0w3tV/Azw2wlPmls4FcUCRTzidzVEDy+dtxqQ7U5ZtiQhjBZetAcnLsCYb58dgwZEjTx2+36jiqcFYvS+FlNJBpbwmnPUyEEb32YBBZj4ra5jB0v4IW4wYYRKTNijAQKxco33VYSCbH0at99SqhXECURA55dtmmJxYpFlT/sMmj0iblOqoG/auapQmmyEEXt/T8hv9StyirabxhbVVSe7fPsAueiXOWVm0kCPO+KN/TyWYB9Hg/mSfnNu9i9";

    public AutonVisionTest() {
    }

    @Override
    public void runOpMode() {

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        robot.init(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();
            do {
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

                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());

                            if (recognition.getLabel().equals("Quad") || recognition.getLabel().equals("Single")) {
                                ringLabel = recognition.getLabel();
                                objectInVision = true;
                            }
                        }
                        telemetry.update();
                    }
                }
            } while (runtime.milliseconds() < 5000 && !(objectInVision));
        }

        if (ringLabel.equals("Quad")) {
            telemetry.addData("Pathing", "Quad");
            telemetry.update();
        }

        else if (ringLabel.equals("Single")) {
            telemetry.addData("Pathing", "Single");
            telemetry.update();
        }
        else {
            telemetry.addData("Pathing", "None");
            telemetry.update();
        }
    }
    private void drive(char direction, double time) {
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
    private void turn(char direction, double time) {
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
    private void stopMotor() {
        robot.fLMotor.setPower(0);
        robot.fRMotor.setPower(0);
        robot.bLMotor.setPower(0);
        robot.bRMotor.setPower(0);
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
