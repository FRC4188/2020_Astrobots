package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "lol")
public class ControllerOp extends OpMode {

    private DcMotor frMotor, flMotor, brMotor, blMotor, intakeMotor;

    @Override
    public void init() {
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //runtime.reset();
    }

    @Override
    public void loop() {
        double forwardController = gamepad1.left_stick_y;
        double strafeController = gamepad1.left_stick_x;
        double rotationController = gamepad1.right_stick_x;

        boolean isIntakePress = gamepad1.a;

        if (isIntakePress) {
            intakeMotor.setPower(0.5);
        }
        else {
            intakeMotor.setPower(0);
        }


        forwardController = Range.clip(forwardController, -1, 1);
        strafeController = Range.clip(strafeController, -1, 1);
        rotationController = Range.clip(rotationController, -1, 1);

        drivetrain(forwardController, strafeController, rotationController);
    }

    private void drivetrain(double forward, double strafe, double rotation) {
        frMotor.setPower(forward - strafe - rotation);
        flMotor.setPower(forward + strafe + rotation);
        brMotor.setPower(forward + strafe - rotation);
        blMotor.setPower(forward - strafe + rotation);
    }
}
