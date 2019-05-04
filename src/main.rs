extern crate nelos_robo;

use std::thread;
use std::time::Duration;
use std::error::Error;
use std::io::stdin;
use std::io::Read;

use nelos_robo::OmniPlatform;
use nelos_robo::devices::StepMotor;


fn main() -> Result<(), Box<dyn Error>> {

    let front_left_motor = StepMotor::new([26, 16, 20, 21]);
    let rear_left_motor = StepMotor::new([19, 13, 5, 6]);
    let rear_right_motor = StepMotor::new([25, 8, 7, 12]);
    let front_right_motor = StepMotor::new([11, 9, 10, 22]);

    let mut robot = OmniPlatform::new(front_left_motor, front_right_motor, rear_left_motor, rear_right_motor);
    robot.start()?;

    robot.drive(1.0, 0.0, 0.0);
    thread::sleep(Duration::from_millis(2000));

    robot.drive(-1.0, 0.0, 0.0);
    thread::sleep(Duration::from_millis(2000));

    robot.drive(0.0, 1.0, 0.0);
    thread::sleep(Duration::from_millis(2000));

    robot.drive(0.0, -1.0, 0.0);
    thread::sleep(Duration::from_millis(2000));

    robot.drive(0.0, 0.0, 1.0);
    thread::sleep(Duration::from_millis(2000));  

    robot.drive(0.0, 0.0, -1.0);
    thread::sleep(Duration::from_millis(2000));

    robot.drive(1.0, 0.0, 1.0);
    thread::sleep(Duration::from_millis(2000));

    robot.drive(0.2, 0.0, 0.1);

    //Wait for key
    let _ = stdin().read(&mut [0u8]).unwrap();

    robot.drive(0.0, 0.0, 0.0);
    let pulses = robot.get_pulse_count();
    println!("{} {} {} {}", pulses[0], pulses[1], pulses[2], pulses[3]);
    robot.stop();

    //Roughly 6150 pulses per round

    Ok(())
}