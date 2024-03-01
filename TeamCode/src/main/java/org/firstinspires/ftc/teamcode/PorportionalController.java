package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled

public class PorportionalController {
    double pValue;
    public PorportionalController(double pValue) {
    this.pValue = pValue;
    }

    double porportionalController(double input, double goalInput) {
        double arbitraryValue = (goalInput - input) * pValue;
        return arbitraryValue;
    }
}
