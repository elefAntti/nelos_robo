use std::error::Error;
use std::fmt;
use std::thread;
use std::thread::JoinHandle;
use std::time::Duration;
use std::sync::{Arc, Mutex};

use rppal::gpio::Gpio;
use rppal::gpio::Level;
use rppal::gpio::OutputPin;

fn set_pins( pins: &mut Vec<OutputPin>, values:[Level;4])
{
    for (pin, value) in pins.iter_mut().zip(values.iter()) {
        pin.write(*value);
    }
}

fn clamp_float( value:f32, min_val:f32, max_val:f32) -> f32{
    if value > max_val {
        max_val
    } else if value < min_val {
        min_val
    } else {
        value
    }
}

#[derive(Debug)]
pub struct MotorError{
}

impl Error for MotorError {
    fn description(&self) -> &str {
        "Motor controller error occured"
    }
}

impl fmt::Display for MotorError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Motor controller error occured")
    }
}

struct MotorState
{
    velocity: f32,
    rotated_steps: i32,
    stop: bool,
    max_velocity_ramp: f32,
}

pub struct StepMotor {
    pins: [u8;4],
    state: Arc<Mutex<MotorState>>,
    thread: Option<JoinHandle<()>>
}

impl StepMotor{
    pub fn new(pins: [u8;4]) -> StepMotor {
        StepMotor {
            state: Arc::new(Mutex::new(MotorState{ 
                velocity: 0.0,
                rotated_steps: 0,
                stop: true,
                max_velocity_ramp : 0.01
                })),
            pins: pins,
            thread: None
        }
    }
    pub fn start(&mut self) -> Result<(), Box<dyn Error>> {
        let gpio = Gpio::new()?;
        let mut pins: Vec<OutputPin> = self.pins.iter()
            .map(|nbr| gpio.get(*nbr).expect("Invalid pin idx").into_output())
            .collect();

        let max_vel_ramp = 0.0;
        let c_mutex = self.state.clone();

        let sequence = [
            [Level::High, Level::Low,  Level::Low,  Level::Low],
            [Level::High, Level::High, Level::Low,  Level::Low],
            [Level::Low,  Level::High, Level::Low,  Level::Low],
            [Level::Low,  Level::High, Level::High, Level::Low],
            [Level::Low,  Level::Low,  Level::High, Level::Low],
            [Level::Low,  Level::Low,  Level::High, Level::High],
            [Level::Low,  Level::Low,  Level::Low,  Level::High],
            [Level::High, Level::Low,  Level::Low,  Level::High]];

        match self.state.lock() {
            Ok(mut state) => {
                if !state.stop {
                    //Motor thread is already running
                    return Ok(());
                }
                state.stop = false;
                state.rotated_steps = 0;
                max_vel_ramp = state.max_velocity_ramp;
            }
            Err(_) => { return Err(Box::new(MotorError{})); }
        }

        self.thread = Some(thread::spawn(move || {
            let mut velocity:f32 = 0.0;
            let mut velocity_setpoint:f32 = 0.0;
            let mut stop = false;
            let mut rotated_steps:i32 = 0;

            while !stop
            {
                if let Ok(ref mut mutex) = c_mutex.try_lock() {
                    velocity_setpoint = mutex.velocity;
                    stop = mutex.stop;
                    max_vel_ramp = mutex.max_velocity_ramp;
                    mutex.rotated_steps = rotated_steps; 
                } 

                let step_count = sequence.len() as i32;
                let step = ((rotated_steps % step_count + step_count) % step_count) as usize;

                if velocity > 0.0 {
                    rotated_steps += 1;
                } else if velocity < 0.0 { 
                    rotated_steps -= 1;
                }            
                
                if velocity == 0.0 {
                    set_pins(&mut pins, [Level::Low, Level::Low, Level::Low, Level::Low]);
                } else {
                    set_pins(&mut pins, sequence[step]);
                }

                let sleeptime_us = if velocity == 0.0 { 1000.0 } else { 1000.0 / velocity.abs().min(1.0) };
                let sleeptime_ms = sleeptime_us / 1000.0;
                thread::sleep(Duration::from_micros(sleeptime_us as u64));

                velocity += clamp_float(velocity_setpoint - velocity, -max_vel_ramp * sleeptime_ms, max_vel_ramp * sleeptime_ms );
            }
            
            set_pins(&mut pins, [Level::Low, Level::Low, Level::Low, Level::Low]);
        }));
        Ok(())
    }

    pub fn set_velocity(&mut self, velocity: f32) {
        let mut lock = self.state.lock().unwrap();
        lock.velocity = velocity;
    }

    pub fn stop(&mut self){
        {
            let mut lock = self.state.lock().unwrap();
            lock.stop = true;
        }
        if let Some(thread) = self.thread.take() {
            thread.join().unwrap();
        }
    }
}

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
            front_left_motor: front_left_motor,
            front_right_motor: front_right_motor,
            rear_left_motor: rear_left_motor,
            rear_right_motor: rear_right_motor
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
        let front_right_vel = forward + left + turn;
        let rear_left_vel = forward - left - turn;
        let rear_right_vel = forward - left + turn;

        let max_vel = [front_left_vel, front_right_vel, rear_left_vel, rear_right_vel].iter().cloned()
            .map(f32::abs)
            .fold(0.0, f32::max);
        let scale = if max_vel > 1.0 { 1.0 / max_vel } else { 1.0 };

        self.front_left_motor.set_velocity(front_left_vel * scale);
        self.front_right_motor.set_velocity(front_right_vel * scale);
        self.rear_left_motor.set_velocity(rear_left_vel * scale);
        self.rear_right_motor.set_velocity(rear_right_vel * scale);
    }
}