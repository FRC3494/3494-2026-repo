## Zeroing swerve

*Make sure swerve gears are on the right when aligning*

####

AdvantageScope Drive > Module[] > rawAbsoluteTurnPosition are the Constants.<position>ZeroPosition

Module[0] = front left
Module[1] = front right
Module[2] = back left
Module[3] = back right




## Turret PID

Visualize in AS
AdvantageScope > RealOutput>  Shooter > Turret > Motor > Setpoint
AdvantageScope > RealOutput>  Shooter > Turret > Motor > Motor

Ran quasistatic characterization auto for turret have reverse and forward on the same logfile

"restart robot code" starts a new log file

ssh into rio and scp log files over

looking at the applied voltage

System identification app

open log file

real outputs shooter > turret > sysidstate

^ drag SysIdState (string) on to testState below

find logs for the motor Turret > motor

- drag velocity/position/ appplied voltage to voltage


