extern crate rppal;
use std::error::Error;
use std::fmt;
use std::thread;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use std::sync::{Arc, Mutex};

use self::rppal::gpio::Gpio;
use self::rppal::gpio::Level;
use self::rppal::gpio::Level::{High, Low};
use self::rppal::gpio::Pin;
use self::rppal::gpio::OutputPin;

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
    min_sleep: Duration,
    steps_per_turn: i32,
    state: Arc<Mutex<MotorState>>,
    thread: Option<JoinHandle<()>>
}

impl StepMotor{
    pub fn new(pins: [u8;4]) -> StepMotor {
        StepMotor {
            steps_per_turn: 6150,
            min_sleep: Duration::from_micros(1000),
            state: Arc::new(Mutex::new(MotorState{ 
                velocity: 0.0,
                rotated_steps: 0,
                stop: true,
                max_velocity_ramp : 0.01
                })),
            pins,
            thread: None
        }
    }

    pub fn start(&mut self) -> Result<(), Box<dyn Error>> {
        let gpio = Gpio::new()?;
        let mut pins = {
            let pins: Result<Vec<OutputPin>, _> = self.pins.iter()
                .map(|nbr| gpio.get(*nbr).map(Pin::into_output))
                .collect();
            pins?
        };
    
        let mut max_vel_ramp;
        let min_sleep_us;
        let c_mutex = self.state.clone();

        let sequence = [
            [High, Low,  Low,  Low],
            [High, High, Low,  Low],
            [Low,  High, Low,  Low],
            [Low,  High, High, Low],
            [Low,  Low,  High, Low],
            [Low,  Low,  High, High],
            [Low,  Low,  Low,  High],
            [High, Low,  Low,  High]];

        match self.state.lock() {
            Ok(mut state) => {
                if !state.stop {
                    //Motor thread is already running
                    return Ok(());
                }
                state.stop = false;
                state.rotated_steps = 0;
                max_vel_ramp = state.max_velocity_ramp;
                min_sleep_us = self.min_sleep.as_micros() as f32;
            }
            Err(_) => { return Err(Box::new(MotorError{})); }
        }

        self.thread = Some(thread::spawn(move || {
            let mut velocity:f32 = 0.0;
            let mut velocity_setpoint:f32 = 0.0;
            let mut stop = false;
            let mut rotated_steps:i32 = 0;
            let mut last_rotate = Instant::now();
            let mut last_update = Instant::now();

            while !stop
            {
                if let Ok(ref mut mutex) = c_mutex.try_lock() {
                    velocity_setpoint = mutex.velocity;
                    stop = mutex.stop;
                    max_vel_ramp = mutex.max_velocity_ramp;
                    mutex.rotated_steps = rotated_steps; 
                } 

                let sleeptime_us = if velocity == 0.0 { min_sleep_us } else { min_sleep_us / velocity.abs().min(1.0) };

                let since_update_ms = last_update.elapsed().as_millis() as f32;
                last_update = Instant::now();
                velocity += clamp_float(velocity_setpoint - velocity, -max_vel_ramp * since_update_ms, max_vel_ramp * since_update_ms );

                let time_since_rotate_us = last_rotate.elapsed().as_micros();

                if time_since_rotate_us > sleeptime_us as u128 {
                    last_rotate = Instant::now();
                    if velocity > 0.0 {
                        rotated_steps += 1;
                    } else if velocity < 0.0 { 
                        rotated_steps -= 1;
                    }            
                    
                    let step_count = sequence.len() as i32;
                    let step = ((rotated_steps % step_count + step_count) % step_count) as usize;

                    if velocity == 0.0 {
                        set_pins(&mut pins, [Low, Low, Low, Low]);
                    } else {
                        set_pins(&mut pins, sequence[step]);
                    }
                } else {
                    let sleep_left = sleeptime_us as u128 - time_since_rotate_us;
                    let partial_sleep = Duration::from_micros(u64::min(sleep_left as u64, 30_000));
                    thread::sleep(partial_sleep);
                }
            }
            
            set_pins(&mut pins, [Low, Low, Low, Low]);
        }));
        Ok(())
    }

    pub fn max_rpm(&self) -> f32
    {
        let mpr = self.min_sleep.as_secs() as f32 / 60.0 * self.steps_per_turn as f32;
        1.0 / mpr
    }

    pub fn set_velocity(&mut self, velocity: f32) {
        let mut lock = self.state.lock().unwrap();
        lock.velocity = velocity;
    }

    pub fn set_rpm(&mut self, velocity: f32) {
        let max_rpm = self.max_rpm();
        self.set_velocity(velocity / max_rpm);
    }


    pub fn get_pulse_count(&self) -> i32
    {
        let lock = self.state.lock().unwrap();
        lock.rotated_steps
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
