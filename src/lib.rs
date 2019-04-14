use std::error::Error;
use std::thread;
use std::thread::JoinHandle;
use std::time::Duration;
use std::sync::{Arc, Mutex};

use rppal::gpio::Gpio;
use rppal::gpio::OutputPin;

fn set_pin( pin: &mut OutputPin, value: bool) -> (){
    if value {
        pin.set_high();
    } else {
        pin.set_low();
    }
}

struct MotorState
{
    velocity: f32,
    rotations: i32,
    stop: bool,
}

pub struct StepMotor {
    pins: [u8;4],
    state: Arc<Mutex<MotorState>>,
    thread: Option<JoinHandle<()>>
}

impl StepMotor{
    pub fn new(pins: [u8;4]) -> StepMotor {
        StepMotor {
            state: Arc::new(Mutex::new(MotorState{ velocity: 0.0, rotations: 0, stop: false })),
            pins: pins,
            thread: None
        }
    }
    pub fn start(&mut self) -> Result<(), Box<dyn Error>> {
        let gpio = Gpio::new()?;
        let mut pins: Vec<OutputPin> = self.pins.iter()
            .map(|nbr| gpio.get(*nbr).expect("Invalid pin idx").into_output())
            .collect();
        let c_mutex = self.state.clone();

        let sequence = [
            [true,  false, false, false],
            [true,  true,  false, false],
            [false, true,  false, false],
            [false, true,  true,  false],
            [false, false, true,  false],
            [false, false, true,  true],
            [false, false, false, true],
            [true,  false, false, true]];

        {
            let mut state = self.state.lock().unwrap();
            state.stop = false;
        }

        self.thread = Some(thread::spawn(move || {
            let mut velocity:f32 = 0.0;
            let mut stop = false;
            let mut rotations:i32 = 0;
            while !stop
            {
                {
                    let mut lock = c_mutex.try_lock();
                    if let Ok(ref mut mutex) = lock {
                        velocity = mutex.velocity;
                        stop = mutex.stop;
                        mutex.rotations = rotations; 
                    } 
                }

                let step_count = sequence.len() as i32;
                let step = ((rotations % step_count + step_count) % step_count) as usize;
                for (pin, value) in pins.iter_mut().zip(sequence[step].iter()) {
                    set_pin(pin, *value);
                }

                if velocity > 0.0 {
                    rotations += 1;
                } else if velocity < 0.0 {
                    rotations -= 1;
                } else {
                    thread::sleep(Duration::from_micros(1000));
                    continue;
                }

                let sleeptime = 1000.0 / velocity.abs().min(1.0);
                thread::sleep(Duration::from_micros(sleeptime.abs() as u64));
            }
            
            for pin in pins.iter_mut() {
                set_pin(pin, false);
            }
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
    frontLeftMotor: StepMotor,
    frontRightMotor: StepMotor,
    rearLeftMotor: StepMotor,
    rearRightMotor: StepMotor
}

impl OmniPlatform
{
    pub fn new( frontLeftMotor: StepMotor, frontRightMotor: StepMotor, rearLeftMotor: StepMotor, rearRightMotor: StepMotor )
        -> OmniPlatform
    {
        OmniPlatform
        {
            frontLeftMotor: frontLeftMotor,
            frontRightMotor: frontRightMotor,
            rearLeftMotor: rearLeftMotor,
            rearRightMotor: rearRightMotor
        }
    }

    pub fn start(&mut self) -> Result<(), Box<dyn Error>> 
    {
        self.frontLeftMotor.start()?;
        self.frontRightMotor.start()?;
        self.rearLeftMotor.start()?;
        self.rearRightMotor.start()
    }

    pub fn stop(&mut self) {
        self.frontLeftMotor.stop();
        self.frontRightMotor.stop();
        self.rearLeftMotor.stop();
        self.rearRightMotor.stop();
    }

    pub fn drive(&mut self, forward: f32, left: f32, turn: f32)
    {
        let frontLeftVel = forward + left - turn;
        let frontRightVel = forward + left + turn;
        let rearLeftVel = forward - left - turn;
        let rearRightVel = forward - left + turn;

        let maxVel = [frontLeftVel, frontRightVel, rearLeftVel, rearRightVel].iter()
            .map(|x| x.abs())
            .fold(0.0, f32::max);
        let scale = if maxVel > 1.0 { 1.0 / maxVel } else { 1.0 };

        self.frontLeftMotor.set_velocity(frontLeftVel * scale);
        self.frontRightMotor.set_velocity(frontRightVel * scale);
        self.rearLeftMotor.set_velocity(rearLeftVel * scale);
        self.rearRightMotor.set_velocity(rearRightVel * scale);
    }
}