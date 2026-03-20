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
            println!("  MIT-style control (single-frame, no RunMode needed):");
            println!("  pos <deg> [kp] [kd]  Position hold (default kp=30 kd=1)");
            println!("  spin <rad/s> [kd]    Velocity mode (default kd=1)");
            println!("  torque <N·m>         Direct torque (clamped ±30)");
            println!("  ctrl <pos> <vel> <kp> <kd> <trq>  Raw MIT control");
            println!();
            println!("  Lifecycle:");
            println!("  enable               Enable the motor");
            println!("  disable              Disable the motor");
            println!("  zero                 Set current position as zero");
            println!();
            println!("  Telemetry:");
            println!("  status               Read motor state");
            println!("  voltage              Read bus voltage");
            println!("  params               Read all key parameters");
            println!();
            println!("  Sequences:");
            println!("  hold                 Hold current position");
            println!("  sweep <deg> <n>      Sweep ±deg for n cycles");
            println!();
            println!("  debug                Toggle hex frame logging");
            println!("  quit / q             Disable motor and exit");
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
            let deg = parse_f32(parts, 1, "pos <degrees> [kp] [kd]")?;
            let kp = parts.get(2).and_then(|s| s.parse().ok());
            let kd = parts.get(3).and_then(|s| s.parse().ok());
            let state = motor.move_to_deg(deg, kp, kd).await?;
            println!("  -> pos={:.1}°  vel={:.3} rad/s  trq={:.3} N·m",
                state.angle_rad.to_degrees(), state.velocity_rads, state.torque_nm);
        }

        "spin" | "v" => {
            let vel = parse_f32(parts, 1, "spin <rad/s> [kd]")?;
            let kd = parts.get(2).and_then(|s| s.parse().ok());
            let state = motor.spin(vel, kd).await?;
            println!("  -> vel={:.3} rad/s  trq={:.3} N·m", state.velocity_rads, state.torque_nm);
        }

        "torque" | "t" => {
            let trq = parse_f32(parts, 1, "torque <N·m>")?;
            let state = motor.set_torque(trq).await?;
            println!("  -> trq={:.3} N·m", state.torque_nm);
        }

        "ctrl" => {
            let pos = parse_f32(parts, 1, "ctrl <pos_rad> <vel> <kp> <kd> <torque>")?;
            let vel = parse_f32(parts, 2, "ctrl <pos_rad> <vel> <kp> <kd> <torque>")?;
            let kp  = parse_f32(parts, 3, "ctrl <pos_rad> <vel> <kp> <kd> <torque>")?;
            let kd  = parse_f32(parts, 4, "ctrl <pos_rad> <vel> <kp> <kd> <torque>")?;
            let trq = parse_f32(parts, 5, "ctrl <pos_rad> <vel> <kp> <kd> <torque>")?;
            let state = motor.send_control(pos, vel, kp, kd, trq).await?;
            print_state(&state);
        }

        "zero" | "z" => {
            motor.set_zero().await?;
            println!("  Zero position set.");
        }

        "voltage" | "vbus" => {
            let v = motor.read_voltage().await?;
            println!("  Bus voltage: {:.1} V", v);
        }

        "params" | "readparams" => {
            use robstride::robstride03::RobStride03Parameter;
            let params = [
                ("RunMode",     RobStride03Parameter::RunMode),
                ("Ref",         RobStride03Parameter::Ref),
                ("LimitSpd",    RobStride03Parameter::LimitSpd),
                ("LimitTorque", RobStride03Parameter::LimitTorque),
                ("LimitCur",    RobStride03Parameter::LimitCur),
                ("LocKp",       RobStride03Parameter::LocKp),
                ("SpdKp",       RobStride03Parameter::SpdKp),
                ("SpdKi",       RobStride03Parameter::SpdKi),
                ("MechPos",     RobStride03Parameter::MechPos),
                ("MechVel",     RobStride03Parameter::MechVel),
                ("Iqf",         RobStride03Parameter::Iqf),
                ("VBus",        RobStride03Parameter::VBus),
            ];
            for (name, param) in &params {
                match motor.read_param(*param).await {
                    Ok(val) => println!("  {:<14} = {}", name, val),
                    Err(e) => println!("  {:<14} = ERROR: {}", name, e),
                }
            }
        }

        "hold" => {
            let state = motor.read_state().await?;
            let state = motor.move_to(state.angle_rad, None, None).await?;
            println!("  Holding at {:.1}° ({:.3} rad)", state.angle_rad.to_degrees(), state.angle_rad);
        }

        "sweep" => {
            let amplitude_deg = parse_f32(parts, 1, "sweep <deg> <n>")?;
            let cycles: u32 = parts
                .get(2)
                .and_then(|s| s.parse().ok())
                .unwrap_or(3);

            println!("  Sweeping ±{:.0}° for {} cycles...", amplitude_deg, cycles);
            for i in 1..=cycles {
                for _ in 0..20 {
                    motor.move_to_deg(amplitude_deg, None, None).await?;
                    tokio::time::sleep(Duration::from_millis(50)).await;
                }
                let state = motor.read_state().await?;
                println!("    cycle {}/{} +  pos={:.1}°", i, cycles, state.angle_rad.to_degrees());

                for _ in 0..20 {
                    motor.move_to_deg(-amplitude_deg, None, None).await?;
                    tokio::time::sleep(Duration::from_millis(50)).await;
                }
                let state = motor.read_state().await?;
                println!("    cycle {}/{} -  pos={:.1}°", i, cycles, state.angle_rad.to_degrees());
            }
            for _ in 0..16 {
                motor.move_to_deg(0.0, None, None).await?;
                tokio::time::sleep(Duration::from_millis(50)).await;
            }
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
