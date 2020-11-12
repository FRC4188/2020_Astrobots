package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "lol")
public class ControllerOp extends OpMode {

    private DcMotor frMotor, flMotor, brMotor, blMotor, intakeMotor, shooterMotor;
    private Servo arm;

    @Override
    public void init() {
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        //arm = hardwareMap.get(Servo.class, "arm");

        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //runtime.reset();
    }

    @Override
    public void loop() {
        double forwardController = Math.pow(gamepad1.left_stick_y, 3);
        double strafeController = Math.pow(gamepad1.left_stick_x, 3);
        double rotationController = gamepad1.right_stick_x;

        boolean isIntakePress = gamepad1.a;
        boolean isOuttakePress = gamepad1.b;
        float isShooterPress = gamepad1.right_trigger;

        if (isIntakePress) {
            intakeMotor.setPower(1);
        } else if(isOuttakePress) {
            intakeMotor.setPower(-1);
        }
        else {
            intakeMotor.setPower(0);
        }

       /* if(gamepad1.x) {
            arm.setPosition(0);
        }

        if(gamepad1.y) {
            arm.setPosition(1);
        }
        */

        drivetrain(forwardController, strafeController, rotationController);
    }

    private void drivetrain(double forward, double strafe, double rotation) {
        double frPower = forward - strafe - rotation;
        double flPower = forward + strafe + rotation;
        double brPower = forward + strafe - rotation;
        double blPower = forward - strafe + rotation;

        double maxPower = Math.max(Math.abs(frPower), Math.abs(flPower));
        maxPower = Math.max(maxPower, Math.abs(brPower));
        maxPower = Math.max(maxPower, Math.abs(blPower));

        frMotor.setPower(frPower / maxPower);
        flMotor.setPower(flPower / maxPower);
        brMotor.setPower(brPower / maxPower);
        blMotor.setPower(blPower / maxPower);
    }
}
