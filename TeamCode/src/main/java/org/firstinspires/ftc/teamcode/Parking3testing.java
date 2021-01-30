
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Parking3testing", group="Linear Opmode")

public class Parking3testing extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frMotor, flMotor, brMotor, blMotor, intakeMotor, magazineMotor, shooterMotor;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            double time = runtime.seconds();
            if (time > 1 && time < 6){
                drivetrain(0.8,0, 0.057);
            }
            if (time > 6 && time < 9){
                drivetrain(-0.8,0,0);
            }
            if (time > 9 && time < 9.7){
                drivetrain(0,0,-0.2);
            }
            if (time > 9.7 && time < 15.5){
                shooterMotor.setPower(0.65);
                drivetrain(0,0,0);
            }
            if (time > 15.5 && time < 17){
                magazineMotor.setPower(1.0);
            }
            if (time > 17 && time < 19.5){
                magazineMotor.setPower(0.0);
            }
            if (time > 19.5 && time < 21){
                magazineMotor.setPower(1.0);
            }
            if (time > 21 && time < 22.5){
                magazineMotor.setPower(0.0);
                intakeMotor.setPower(-1.0);
            }
            if (time > 22.5 && time < 25.5){
                magazineMotor.setPower(1.0);
            }
            if (time > 25.5 && time < 27.5){
                intakeMotor.setPower(0.0);
                magazineMotor.setPower(0.0);
                shooterMotor.setPower(0.0);
                drivetrain(0.7,0,0);
            }
            if (time > 27.5){
                drivetrain(0,0,0);
            }
        }
    }

    private void initialize() {
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        magazineMotor = hardwareMap.get(DcMotor.class, "magazineMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        magazineMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        runtime.reset();
    }

    private void drivetrain(double forward, double strafe, double rotation) {
        frMotor.setPower(forward - strafe - rotation);
        flMotor.setPower(forward + strafe + rotation);
        brMotor.setPower(forward + strafe - rotation);
        blMotor.setPower(forward - strafe + rotation);
    }

}
