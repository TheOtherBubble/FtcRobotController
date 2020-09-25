package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import org.firstinspires.ftc.teamcode.src.main.java.org.firstinspires.ftc.teamcode.DriveOnlyHardware;
import org.firstinspires.ftc.teamcode.AutonDriving;
import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous(name="WheelTest", group="Test")
@Disabled
public class WheelTest extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();
    String xyz = "z";

    static final double     COUNTS_PER_MOTOR_REV = 1120 ;    // Currently: Andymark Neverest 40
    static final double     COUNTS_PER_REV_ARM = 1440;
    static final double     COUNTS_PER_INCH_ARM = COUNTS_PER_REV_ARM/4;
    static final double     DRIVE_GEAR_REDUCTION = .9;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    static final double     WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


//    static final double COUNTS_PER_MOTOR_REV = 537; //216
//    static final double DRIVE_GEAR_REDUCTION = 0.6666;     // This is < 1.0 if geared UP
//    static final double WHEEL_DIAMETER_INCHES = 3.4;     // For figuring circumference
//    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415);
//    static final double DRIVE_SPEED = 1;
//    static final double TURN_SPEED = 0.5;
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //init distance sensors
        /*
        Rev2mDistanceSensor sensorRangeR = hardwareMap.get(Rev2mDistanceSensor.class, "sensorRangeR");
        Rev2mDistanceSensor sensorRangeL = hardwareMap.get(Rev2mDistanceSensor.class, "sensorRangeL");
        sensorRangeL.initialize();
        sensorRangeR.initialize();
        */
        //side motors
        robot.fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lateral motors
        robot.bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        motorTest(robot.fLMotor);

        motorTest(robot.fRMotor);

        motorTest(robot.bLMotor);

        motorTest(robot.bRMotor);

        pathComplete(500);
    }

    public void pathComplete(int millisec)
    {
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(millisec);
    }

    public void motorTest(DcMotor motor)
    {
        motor.setPower(.6);
        sleep(2000);
        motor.setPower(0);
    }

    public static double counts(double inches)
    {
        double newInches = (inches - 3.7959) / 1.1239;
        return newInches;
    }



    public void normalDrive(double lpower, double rpower) {

        if (opModeIsActive()) {
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fLMotor.setPower(lpower);
            robot.fRMotor.setPower(rpower);
            robot.bLMotor.setPower(lpower);
            robot.bRMotor.setPower(rpower);
        }
    }


    public void turnToPosition (double target, String xyz, double topPower, double timeoutS, boolean isCorrection) {
        //Write code to correct to a target position (NOT FINISHED)
        target*= -1;
        double originalAngle = readAngle(xyz);


        runtime.reset();

        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double error = angle - target;
        double powerScaled = topPower;
        do {
            angle = readAngle(xyz);
            error = angle - target;
            if (!isCorrection) {
                powerScaled = topPower * (error / 180) * pidMultiplierTurning(error);
            }

            //double powerScaled = power*pidMultiplier(error);
            telemetry.addData("original angle", originalAngle);
            telemetry.addData("current angle", readAngle(xyz));
            telemetry.addData("error", error);
            telemetry.update();
            if (error > 0) {
                if (xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
                if (xyz.equals("y")) {
                    if (opModeIsActive()) {
                        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fLMotor.setPower(powerScaled);
                        robot.fRMotor.setPower(powerScaled);
                        robot.bLMotor.setPower(powerScaled);
                        robot.bRMotor.setPower(powerScaled);
                    }
                }
            } else if (error < 0) {
                if (xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
                if (xyz.equals("y")) {
                    if (opModeIsActive()) {
                        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fLMotor.setPower(powerScaled);
                        robot.fRMotor.setPower(powerScaled);
                        robot.bLMotor.setPower(powerScaled);
                        robot.bRMotor.setPower(powerScaled);
                    }
                }
            }
        } while (opModeIsActive() && ((error > .3) || (error < -0.3)) && (runtime.seconds() < timeoutS));
        normalDrive(0, 0);

    }

    public double pidMultiplierDriving(double error) {
        //equation for power multiplier is x/sqrt(x^2 + C)
        int C = 100;
        return Math.abs(error / Math.sqrt((error * error) + C));
    }
    public double pidMultiplierTurning(double error) {
        //equation for power multiplier is x/sqrt(x^2 + C)
        double C = .1;
        return Math.abs(error / Math.sqrt((error * error) + C));
    }

    public double readAngle(String xyz) {
        Orientation angles;
        Acceleration gravity;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (xyz.equals("x")) {
            return angles.thirdAngle;
        } else if (xyz.equals("y")) {
            return angles.secondAngle;
        } else if (xyz.equals("z")) {
            return angles.firstAngle;
        } else {
            return 0;
        }
    }

    public void encoderDrive(double inches, String direction, double timeoutS, double topPower)
    {
        int TargetFL = 0;
        int TargetFR = 0;
        int TargetBL = 0;
        int TargetBR = 0;
        double errorFL = 0;
        double errorFR = 0;
        double errorBL = 0;
        double errorBR = 0;
        double powerFL = 0;
        double powerFR = 0;
        double powerBL = 0;
        double powerBR = 0;


        String heading = direction;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            if(heading == "f")
            {
                TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);

            }

            else if(heading == "b")
            {
                TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);


            }

            else if(heading == "r")
            {
                TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH); //weird should be +


            }

            else if(heading == "l")
            {
                TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH); // weird should be +
                TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);

            }

            else
            {
                telemetry.addData("not a valid direction", heading );
            }



            // Determine new target position, and pass to motor controller

            robot.fLMotor.setTargetPosition(TargetFL);
            robot.fRMotor.setTargetPosition(TargetFR);
            robot.bRMotor.setTargetPosition(TargetBR);
            robot.bLMotor.setTargetPosition(TargetBL);


            // Turn On RUN_TO_POSITION
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            /*robot.fLMotor.setPower(Speed);
            robot.fRMotor.setPower(Speed);
            robot.bRMotor.setPower(Speed);
            robot.bLMotor.setPower(Speed);*/


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && ((robot.fLMotor.isBusy() && robot.fRMotor.isBusy()) && robot.bLMotor.isBusy() && robot.bRMotor.isBusy()))
            {
                errorFL = TargetFL - robot.fLMotor.getCurrentPosition();
                errorFR = TargetFR - robot.fRMotor.getCurrentPosition();
                errorBL = TargetBL - robot.bLMotor.getCurrentPosition();
                errorBR = TargetBR - robot.bRMotor.getCurrentPosition();

                powerFL = topPower * pidMultiplierDriving(errorFL);
                powerFR = topPower * pidMultiplierDriving(errorFR);
                powerBL = topPower * pidMultiplierDriving(errorBL);
                powerBR = topPower* pidMultiplierDriving(errorBR);

                robot.fLMotor.setPower(Math.abs(powerFL));
                robot.fRMotor.setPower(Math.abs(powerFR));
                robot.bRMotor.setPower(Math.abs(powerBL));
                robot.bLMotor.setPower(Math.abs(powerBR));
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", TargetFL,  TargetFR, TargetBL, TargetBR);

                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", robot.fLMotor.getCurrentPosition(), robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                //telemetry.addData("speeds",  "Running to %7f :%7f :%7f :%7f", speedfL,  speedfR, speedfL, speedbR);
                telemetry.update();
                //Display it for the driver.
//                telemetry.addData("Remaining Dist",  "Running to %7d :%7d :%7d :%7d", errorFL,  errorFR, errorBL, errorBR);
//                telemetry.addData("Current Pos",  "Running to " + robot.fLMotor.getCurrentPosition() + robot.fRMotor.getCurrentPosition() + robot.bLMotor.getCurrentPosition() + robot.bRMotor.getCurrentPosition());
//                telemetry.addData("Target",  "Running to " + TargetFL + TargetFR + TargetBL +TargetBR);
//                telemetry.addData("Power",  "Running at %7d :%7d :%7d :%7d", powerFL, powerFR, powerBL, powerBR);
                //telemetry.addData("speeds",  "Running to %7f :%7f :%7f :%7f", speedfL,  speedfR, speedfL, speedbR);
                //telemetry.update();
            }

            // Stop all motion;
            robot.fLMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bRMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
}