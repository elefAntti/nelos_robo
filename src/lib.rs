pub mod devices;

use devices::StepMotor;
use std::error::Error;

pub struct OmniPlatform
{
    front_left_motor: StepMotor,
    front_right_motor: StepMotor,
    rear_left_motor: StepMotor,
    rear_right_motor: StepMotor
}

impl OmniPlatform
{
    pub fn new( front_left_motor: StepMotor, front_right_motor: StepMotor, rear_left_motor: StepMotor, rear_right_motor: StepMotor )
        -> OmniPlatform
    {
        OmniPlatform
        {
            front_left_motor,
            front_right_motor,
            rear_left_motor,
            rear_right_motor
        }
    }

    pub fn start(&mut self) -> Result<(), Box<dyn Error>> 
    {
        self.front_left_motor.start()?;
        self.front_right_motor.start()?;
        self.rear_left_motor.start()?;
        self.rear_right_motor.start()
    }

    pub fn stop(&mut self) {
        self.front_left_motor.stop();
        self.front_right_motor.stop();
        self.rear_left_motor.stop();
        self.rear_right_motor.stop();
    }

    pub fn drive(&mut self, forward: f32, left: f32, turn: f32)
    {
        let front_left_vel = forward + left - turn;
        let front_right_vel = forward - left + turn;
        let rear_left_vel = forward - left - turn;
        let rear_right_vel = forward + left + turn;

        let max_vel = [front_left_vel, front_right_vel, rear_left_vel, rear_right_vel].iter().cloned()
            .map(f32::abs)
            .fold(0.0, f32::max);
        let scale = if max_vel > 1.0 { 1.0 / max_vel } else { 1.0 };

        self.front_left_motor.set_velocity(front_left_vel * scale);
        self.front_right_motor.set_velocity(front_right_vel * scale);
        self.rear_left_motor.set_velocity(rear_left_vel * scale);
        self.rear_right_motor.set_velocity(rear_right_vel * scale);
    }

    pub fn get_pulse_count(&self) -> [i32;4]
    {
        [
            self.front_left_motor.get_pulse_count(),
            self.front_right_motor.get_pulse_count(),
            self.rear_left_motor.get_pulse_count(),
            self.rear_right_motor.get_pulse_count()
        ]
    }
}