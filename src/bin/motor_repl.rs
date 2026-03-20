use std::io::{self, Write};
use std::time::Duration;

use anyhow::Result;
use robot::config::RobotConfig;
use robot::motor::{create_ch341_protocol, Motor};

#[tokio::main]
async fn main() -> Result<()> {
    let config = RobotConfig::load("config/robot.yaml")?;

    let can_id: u8 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(127);

    println!("Motor REPL — CAN ID {}", can_id);
    println!("Opening {} ...", config.bus.port);

    let protocol = create_ch341_protocol(&config.bus.port).await?;
    let mut motor = Motor::new(protocol, can_id);

    println!("Transport ready. Type 'help' for commands.\n");

    let mut line = String::new();
    loop {
        print!("motor[{}]> ", can_id);
        io::stdout().flush()?;

        line.clear();
        if io::stdin().read_line(&mut line)? == 0 {
            break;
        }

        let parts: Vec<&str> = line.trim().split_whitespace().collect();
        if parts.is_empty() {
            continue;
        }

        let result = handle_command(&mut motor, &parts).await;
        match result {
            Ok(Action::Continue) => {}
            Ok(Action::Quit) => break,
            Err(e) => eprintln!("  Error: {:#}", e),
        }
    }

    println!("Disabling motor...");
    let _ = motor.disable().await;
    println!("Done.");
    Ok(())
}

enum Action {
    Continue,
    Quit,
}

async fn handle_command(motor: &mut Motor, parts: &[&str]) -> Result<Action> {
    match parts[0] {
        "help" | "h" | "?" => {
            println!("  enable          Enable the motor");
            println!("  disable         Disable the motor");
            println!("  status          Read and print motor state");
            println!("  pos <deg>       Move to position (degrees)");
            println!("  spin <rad/s>    Velocity mode");
            println!("  torque <N·m>    Torque mode");
            println!("  zero            Set current position as zero");
            println!("  voltage         Read bus voltage");
            println!("  gains <kp>      Set position Kp");
            println!("  spdgains <kp> <ki>  Set speed Kp/Ki");
            println!("  spdlim <rad/s>  Set speed limit");
            println!("  trqlim <N·m>    Set torque limit");
            println!("  hold            Hold current position (position mode)");
            println!("  sweep <deg> <n> Sweep ±deg for n cycles at 3 rad/s");
            println!("  debug           Toggle hex frame logging");
            println!("  quit / q        Disable motor and exit");
        }

        "debug" | "d" => {
            motor.debug = !motor.debug;
            println!("  Debug logging: {}", if motor.debug { "ON" } else { "OFF" });
        }

        "enable" | "en" => {
            let state = motor.enable().await?;
            print_state(&state);
        }

        "disable" | "dis" => {
            let state = motor.disable().await?;
            println!("  Disabled. Mode: {:?}", state.mode);
        }

        "status" | "s" => {
            let state = motor.read_state().await?;
            print_state(&state);
        }

        "pos" | "p" => {
            let deg = parse_f32(parts, 1, "pos <degrees>")?;
            motor.move_to_deg(deg, Some(5.0)).await?;
            println!("  Moving to {:.1}°  (speed limit 5 rad/s)", deg);
        }

        "spin" | "v" => {
            let vel = parse_f32(parts, 1, "spin <rad/s>")?;
            let vel = vel.clamp(-10.0, 10.0);
            motor.spin(vel).await?;
            println!("  Spinning at {:.2} rad/s", vel);
        }

        "torque" | "t" => {
            let trq = parse_f32(parts, 1, "torque <N·m>")?;
            let trq = trq.clamp(-30.0, 30.0);
            motor.set_torque(trq).await?;
            println!("  Torque set to {:.2} N·m", trq);
        }

        "zero" | "z" => {
            motor.set_zero().await?;
            println!("  Zero position set.");
        }

        "voltage" | "vbus" => {
            let v = motor.read_voltage().await?;
            println!("  Bus voltage: {:.1} V", v);
        }

        "gains" => {
            let kp = parse_f32(parts, 1, "gains <kp>")?;
            motor.set_position_gain(kp).await?;
            println!("  Position Kp = {:.2}", kp);
        }

        "spdgains" => {
            let kp = parse_f32(parts, 1, "spdgains <kp> <ki>")?;
            let ki = parse_f32(parts, 2, "spdgains <kp> <ki>")?;
            motor.set_speed_gain(kp, ki).await?;
            println!("  Speed Kp = {:.2}, Ki = {:.4}", kp, ki);
        }

        "spdlim" => {
            let lim = parse_f32(parts, 1, "spdlim <rad/s>")?;
            let lim = lim.clamp(0.1, 10.0);
            motor.set_speed_limit(lim).await?;
            println!("  Speed limit = {:.2} rad/s", lim);
        }

        "trqlim" => {
            let lim = parse_f32(parts, 1, "trqlim <N·m>")?;
            let lim = lim.clamp(0.1, 30.0);
            motor.set_torque_limit(lim).await?;
            println!("  Torque limit = {:.2} N·m", lim);
        }

        "hold" => {
            let state = motor.read_state().await?;
            let current_deg = state.angle_rad.to_degrees();
            motor.move_to_deg(current_deg, Some(5.0)).await?;
            println!("  Holding at {:.1}° ({:.3} rad)", current_deg, state.angle_rad);
        }

        "sweep" => {
            let amplitude_deg = parse_f32(parts, 1, "sweep <deg> <n>")?;
            let cycles: u32 = parts
                .get(2)
                .and_then(|s| s.parse().ok())
                .unwrap_or(3);
            let speed = Some(3.0_f32);

            println!("  Sweeping ±{:.0}° for {} cycles...", amplitude_deg, cycles);
            for i in 1..=cycles {
                motor.move_to_deg(amplitude_deg, speed).await?;
                tokio::time::sleep(Duration::from_secs(1)).await;
                let state = motor.read_state().await?;
                println!("    cycle {}/{} +  pos={:.1}°", i, cycles, state.angle_rad.to_degrees());

                motor.move_to_deg(-amplitude_deg, speed).await?;
                tokio::time::sleep(Duration::from_secs(1)).await;
                let state = motor.read_state().await?;
                println!("    cycle {}/{} -  pos={:.1}°", i, cycles, state.angle_rad.to_degrees());
            }
            motor.move_to_deg(0.0, speed).await?;
            tokio::time::sleep(Duration::from_millis(800)).await;
            println!("  Sweep done, returned to 0°.");
        }

        "quit" | "q" | "exit" => {
            return Ok(Action::Quit);
        }

        other => {
            eprintln!("  Unknown command: '{}'. Type 'help' for usage.", other);
        }
    }

    Ok(Action::Continue)
}

fn print_state(state: &robot::motor::MotorState) {
    println!("  Position:    {:.3} rad  ({:.1}°)", state.angle_rad, state.angle_rad.to_degrees());
    println!("  Velocity:    {:.3} rad/s", state.velocity_rads);
    println!("  Torque:      {:.3} N·m", state.torque_nm);
    println!("  Temperature: {:.1} °C", state.temperature_c);
    println!("  Mode:        {:?}", state.mode);
    if !state.faults.is_empty() {
        println!("  Faults:      {:?}", state.faults);
    }
}

fn parse_f32(parts: &[&str], idx: usize, usage: &str) -> Result<f32> {
    parts
        .get(idx)
        .ok_or_else(|| anyhow::anyhow!("Usage: {}", usage))?
        .parse::<f32>()
        .map_err(|_| anyhow::anyhow!("Invalid number. Usage: {}", usage))
}
