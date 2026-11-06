package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.jvm.Pool;

import org.firstinspires.ftc.vision.apriltag.AprilTagCanvasAnnotator;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Turret {
    private DcMotorEx turret;

    private double kP = 0.0001;
    private double kD = 0;

    private double goalX = 0;

    private double lastError = 0;
    private double angleTolerance = 0.2;

    //tune this please
    private final double Max_POWER = 0.6;
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();

    private HardwareMap hwMap;

    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setkP(double newKP) {
        kP = newKP;
    }

    public double getkP(){
        return kP;
    }
    public void setkD(double newKD) {
        kP = newKD;
    }
    public double getkD(){
        return kD;
    }

    public void resetTimer(){
    timer.reset();

    }

    public void update(LLResult curID) {
        double deltaTime = timer.seconds();
        timer.reset();

        if (curID == null ) {
            turret.setPower(0);
            lastError = 0;
            return;
        }

        //------ start PD controller ------

        double error = goalX - curID.ftcPose.bearing;
        double pTerm = error * kP;

        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -Max_POWER, Max_POWER);
        }

        // safety encoder check OR magnetic limit switch check, OR other safety

        turret.setPower(power);
        lastError = error;
    }

}
