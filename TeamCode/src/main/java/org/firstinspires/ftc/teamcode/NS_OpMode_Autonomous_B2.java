package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Nithya on 12/6/2017.
 */

@Autonomous( name = "Autonomous B2", group = "Copmetition")
public class NS_OpMode_Autonomous_B2 extends NS_OpMode_Autonomous {
    @Override
    public void DriveAutonomous() {
        if (opModeIsActive()) KnockJewel(Jewel.RED);

        if (opModeIsActive()) gyroDrive(DRIVE_SPEED, 24, 0.0);
        if (opModeIsActive()) gyroTurn(TURN_SPEED, -26.5);
        if (opModeIsActive()) gyroDrive(DRIVE_SPEED, 7.25, 0.0);
    }
}
