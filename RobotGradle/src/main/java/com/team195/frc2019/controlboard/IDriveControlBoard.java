package com.team195.frc2019.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getTurn();

    boolean getPoopyShoot();

    boolean getQuickTurn();

    boolean getOpenJaw();

    boolean getShoot();
}