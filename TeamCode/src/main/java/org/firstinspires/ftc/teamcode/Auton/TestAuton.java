package org.firstinspires.ftc.teamcode.Auton;

import android.database.sqlite.SQLiteException;

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

//@Disabled
@Autonomous(name="TestAuton", group="AutonTest")
public class TestAuton extends AutonDriving {
//    AutonDrivingDriveOnly auton = new AutonDrivingDriveOnly();

    //SkyStoneHardwareDrivingOnly robot = new SkyStoneHardwareDrivingOnly();
    @Override
    public void runOpMode() {

        //init
        robot.init(hardwareMap);
        BNO055IMU.Parameters p = new BNO055IMU.Parameters();
        p.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        p.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        p.calibrationDataFile = "BNO055IMUCalibration.json";
        p.loggingEnabled = true;
        p.loggingTag = "IMU";
        p.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(p);

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

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");


        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        robot.claw.setPosition(clawPos);
        waitForStart();






        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        startAngle = readAngle("z");
        setDir();


        //get in position and read skystone
        gyroDrive(16, NORTH, true, gyroDriveSpeed - .02, moderate, 10);//14 is the only distance that works pls no change
        turnToPosition(NORTH, "z", turnSpeed + .05, 5, false);
        String skystone = vuforia(allTrackables, targetsSkyStone);

        //sleep(1000);

        telemetry.addData("Position", skystone);
        telemetry.update();

        sleep(100);


        //turnDegrees(90, "z", turnSpeed, 5);


        //aim for corner of block to intake
        if(skystone.equals("center"))
        {

            strafe(4, .6, left, leftBal + .01, 362.5, .04);
            sleep(100);
            turnToPosition(NORTH + 31, "z", turnSpeed, 10, true);
            robot.intakeL.setPower(1);
            robot.intakeR.setPower(1);
            gyroDrive(30, NORTH + 31, true, gyroDriveSpeed, moderate + .03, 5);
        }


        //TODO: WORK ON ALT SKYSTONE POSITIONS
        else if(skystone.equals("left"))
        {
            strafe(3, .6, left, leftBal + .01, 250, .04);


            sleep(100);
            turnToPosition(NORTH + 15, "z", turnSpeed, 10, false);


            robot.intakeL.setPower(1);
            robot.intakeR.setPower(1);
            gyroDrive(30, NORTH + 15, true, gyroDriveSpeed, moderate + .03, 10);
        }
        else
        {
            strafe(6, .6, left, leftBal + .01, 362.5, .04);


            sleep(100);
            turnToPosition(NORTH + 40, "z", turnSpeed, 10, false);


            robot.intakeL.setPower(1);
            robot.intakeR.setPower(1);
            gyroDrive(30, NORTH + 40, true, gyroDriveSpeed, moderate + .03, 10);
        }

        sleep(500);

        robot.intakeL.setPower(0);
        robot.intakeR.setPower(0);
        //sleep(100);
        //turnToPosition(EAST, "z", 1, 5, true);


        sleep(100);


        //strafe under skybridge
        strafe(1, .9, right, .05, 2000, .03); //2600 for full auto
        robot.intakeL.setPower(-1);
        robot.intakeR.setPower(-1);

        //turn towards foundation
        /*turnToPosition(178, "z", turnSpeed, 5, false);

        //drive towards the foundation & grab
        sleep(100);
        gyroDrive(-25, SOUTH, true, gyroDriveSpeed, moderate,10);
        robot.latch.setPosition(0);

        sleep(1000);

        //turn drive and strafe along wall
        turnToPosition(EAST, "z", .5, 5, false, true);
        sleep(100);
        gyroDrive(-20, WEST, true, gyroDriveSpeed, moderate, 10);
        strafe(4, .6, right, rightBal,250, .03);


        //let go of foundation
        robot.latch.setPosition(1);

        sleep(1000);

        gyroDrive(10, WEST, true, gyroDriveSpeed, moderate, 10);*/


        pathComplete(500);
    }
}