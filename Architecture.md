# Architecture
Living document linking the physical design with software design.

## Intake

### Functionality
- Moves fuel into the hopper
- Moves in and out of frame perims
- Moves to 2 set positions (`STOWED` and `DEPLOYED`)

### Actuators
- Motor that extends and retracts the intake
- Motor the runs the rollers

### Sensors 

### Commands
- `STOW` and `DEPLOY` intake 
- Stop and run intake 

## Hopper

### Functionality
- Moves fuel around, pushing it into the shooter
- Moves at one set speed 
- Rotates in both directions 

### Actuators
- Motor that spins hopper to agitate the fuel

### Sensors

### Commands
- Run and stop the hopper motor (left or right)

## Turret

### Functionality
- Point the shooter towards the goal
- Limited rotation 
- PID controller for turret position 
- The turret is `PRIMED` when its at the right position for firing

### Actuators
- Motor that spins the turret around

### Sensors
- Limit switch 
- Limelight

### Commands
- `PRIME` the turret
  - Use feild localization to get robot position to set the shooter position for scoring
- Rotate the turret

## Shooter

### Functionality
- Send balls into the goal
- Shooter will run at different speed for range control 
- Hood will be at different positions for range control 
- Velocity PID controller for flywheel
- The hood is `PRIMED` when its at the right position for firing 

### Actuators
- 2 motors that spin the flywheel for shooting
- 1 motor that changes the angle of the hood

### Sensors
- Limit switch

### Commands
- `PRIME` the hood
  - Use field localization to get distance from hub to set the shooter hood position for scoring

## Climber

### Functionality
- Climb in auto and endgame
- Climber can go to different setpoints (`STOWED`,`L1`,`L2`,`L3`)

### Actuators
- 2 motors; one for each hook that moves it up and down.

### Sensors
- Limit switches

### Commands
- Raise climber elevator to `L1`,`L2`,`L3`
- Lower climber elevator to `STOWED`