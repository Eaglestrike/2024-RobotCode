#pragma once

namespace ShooterConstants{
    const int FLYWHEEL_ID = 26;

    const double FLYWHEEL_MAX_A = 0.0;
    const double FLYWHEEL_MAX_VOLTS = 0.0;

    struct Feedforward{
        double ks;
        double kv;
        double ka;
    };

    const Feedforward FLYWHEEL_FF = {
        .ks = 0.0,
        .kv = 0.0,
        .ka = 0.0
    };
}