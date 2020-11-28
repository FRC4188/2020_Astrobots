package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Parking2")
public class Parking2 extends LinearOpMode {

    private DcMotor frMotor, flMotor, brMotor, blMotor, intakeMotor, magazineMotor, shooterMotor;
    private CRServo arm;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        frMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 2.286));
        flMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 2.286));
        brMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 2.286));
        blMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 2.286));

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllPower(0.5);

        while (opModeIsActive() && (flMotor.isBusy())) {
            idle();
        }

        setAllPower(0);

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5) {
            idle();
        }
        //go back
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frMotor.setTargetPosition((int) Math.round(Constants.TICKS_PER_METER * (2.286 - 1.95)));
        blMotor.setTargetPosition((int) Math.round(Constants.TICKS_PER_METER * (2.286 - 1.95)));
        flMotor.setTargetPosition((int) Math.round(Constants.TICKS_PER_METER * (2.286 - 1.95)));
        brMotor.setTargetPosition((int) Math.round(Constants.TICKS_PER_METER * (2.286 - 1.95)));
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        setAllPower(1.0);
        while (opModeIsActive() && (frMotor.isBusy() || flMotor.isBusy() || brMotor.isBusy() || blMotor.isBusy())) {
            idle();
        }
        setAllPower(0);
    }

    private void initialize() {
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        magazineMotor = hardwareMap.get(DcMotor.class, "magazineMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        arm = hardwareMap.get(CRServo.class, "wobbleArm");

        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        magazineMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void setMotorMode(DcMotor.RunMode runMode) {
        frMotor.setMode(runMode);
        flMotor.setMode(runMode);
        brMotor.setMode(runMode);
        blMotor.setMode(runMode);
    }

    private void setAllPower(double power) {
        frMotor.setPower(power - 0.4);
        flMotor.setPower(power + 0.2);
        brMotor.setPower(power - 0.4);
        blMotor.setPower(power + 0.3);
    }

    private void rotate(double power) {
        frMotor.setPower(-power);
        flMotor.setPower(power);
        brMotor.setPower(-power);
        blMotor.setPower(power);
    }
}
