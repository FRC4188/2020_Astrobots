
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Parking1seconds", group="Linear Opmode")

public class Parking1seconds extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx frMotor, flMotor, brMotor, blMotor, intakeMotor, magazineMotor, shooterMotor;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        runtime.reset();


        while(opModeIsActive()) {
            double time = runtime.seconds();
            if (time > 1 && time < 3.8){
                drivetrain(1,0, 0.08);
            }
            if (time > 3.8 && time < 3.95){
                drivetrain(-0.8, 0,0);
            }
            if (time > 3.95){
                drivetrain(0,0,0);
            }

        }
    }

    private void initialize() {
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        magazineMotor = hardwareMap.get(DcMotorEx.class, "magazineMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

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
