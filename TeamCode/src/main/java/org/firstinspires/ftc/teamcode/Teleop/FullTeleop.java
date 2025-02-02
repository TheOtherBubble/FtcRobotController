package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

//import org.firstinspires.ftc.teamcode.src.main.java.org.firstinspires.ftc.teamcode.DriveOnlyHardware;


@TeleOp(name="FullTeleop", group="Teleop")

//@Disabled

public class FullTeleop extends OpMode {

    Hardware robot = new Hardware();

    boolean timerv = false;

    ElapsedTime timer = new ElapsedTime();

    private float drive = .7f;
    //private float BRDrive = 1f;

    @Override
    public void init()
    {
        //Initialize the hardware variables.
        //The init() method of the hardware class does all the work here
        robot.init(hardwareMap);


    }

    @Override
    public void loop()

    {
        if (gamepad1.right_bumper) {
            drive = 0.25f;
        }
        else {
            drive = 0.7f;
        }
        if (gamepad1.x || gamepad2.x) {
            robot.launcherServo.setPosition(0.2746);
        }
        else {
            robot.launcherServo.setPosition(.7);
        }
        if (gamepad1.a || gamepad2.x){
            robot.claw.setPosition(0);
        }
        else {
            robot.claw.setPosition(1);
        }
        if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger >.1) {
            robot.launcherMotor.setPower(.6);
        }
        else {
            robot.launcherMotor.setPower(0);
        }
        if (gamepad1.left_bumper || gamepad2.left_bumper)
        {
            robot.intakeMotor.setPower(1);
            robot.conveyorMotor.setPower(-1);
        }
        else
        {
            robot.intakeMotor.setPower(0);
            robot.conveyorMotor.setPower(0);
        }

        if (gamepad1.dpad_up) {
            double liftPower = 0.3;
            if (gamepad1.left_trigger > 0.1) {
                liftPower += 0.3;
            }
            robot.liftMotor.setPower(liftPower);
        }
        else if (gamepad1.dpad_down) {
            double liftPower = 0.3;
            if (gamepad1.left_trigger > 0.1) {
                liftPower += 0.3;
            }
            robot.liftMotor.setPower(-liftPower);
        }
        else {
            robot.liftMotor.setPower(0);
        }

        mecanumMove();

    }

    public void mecanumMove()
    {
        //variables
        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;


        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.fLMotor.setPower(-drive * v1);
        robot.fRMotor.setPower(-drive * v2);
        robot.bLMotor.setPower(-drive * v3);
        robot.bRMotor.setPower(-drive * v4);

        telemetry.addData("fLPower", -drive * v1);
        telemetry.addData("fRPower", -drive * v2);
        telemetry.addData("bLPower", -drive * v3);
        telemetry.addData("bRPower", -drive * v4);
    }
}