use std::{
    env,
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};

use anyhow::anyhow;
use duct::cmd;

const OPENOCD_INTERFACE: &str = "ftdi/jtag-lock-pick_tiny_2";
const OPENOCD_TARGET: &str = "stm32f4x";
const RTT_TCP_PORT: u16 = 8765;

fn main() -> Result<(), anyhow::Error> {
    let args = env::args().skip(1).collect::<Vec<_>>();
    let args = args.iter().map(|s| &**s).collect::<Vec<_>>();

    env::set_current_dir(repo_root()?)?;

    match &args[..] {
        ["gdb", elf_path] => {
            // Use elf_path here
            gdb(&Path::new(elf_path))?;
        },
        _ => println!("Cargo workflows

USAGE:
    cargo xtask [COMMAND] <PARAMETER>

COMMANDS:
    gdb     spawns a GDB server; flashes and runs firmware; prints logs, require elf path as PARAMETER
"),
    }

    Ok(())
}

fn repo_root() -> Result<PathBuf, anyhow::Error> {
    // path to this crate (the directory that contains this crate's Cargo.toml)
    Ok(PathBuf::from(env::var("CARGO_MANIFEST_DIR")?)
        // from there go one level up
        .parent()
        .unwrap()
        .to_owned())
}

fn gdb(elf: &Path) -> Result<(), anyhow::Error> {
    const RTT_BLOCK_IF_FULL: u32 = 2; // bit in `flags` field
    const RTT_FLAGS: u32 = 44; // offset of `flags` field in control block
    const RTT_ID: &str = "SEGGER RTT"; // control block ID
    const RTT_SIZE: u8 = 48; // control block size
    const THUMB_BIT: u32 = 1;

    let openocd_path = String::from(env::var("OPENOCD_PATH")?).to_owned();

    // get symbol addresses from ELF
    let nm = cmd!("nm", "-C", &elf).read()?;
    let mut rtt = None;
    let mut main = None;
    for line in nm.lines() {
        if line.ends_with("_SEGGER_RTT") {
            rtt = line.splitn(2, ' ').next();
        } else if line.ends_with("main") {
            main = line.splitn(2, ' ').next();
        }
    }

    let rtt = u32::from_str_radix(
        rtt.ok_or_else(|| anyhow!("RTT control block not found"))?,
        16,
    )?;
    let main = u32::from_str_radix(
        main.ok_or_else(|| anyhow!("`main` function not found"))?,
        16,
    )? & !THUMB_BIT;

    #[rustfmt::skip]
    let openocd = cmd!(
        "openocd",
        "-d1",
        "-f", format!("{}/share/openocd/scripts/interface/{}.cfg", openocd_path, OPENOCD_INTERFACE),
        "-c", "transport select swd",
        "-f", format!("{}/share/openocd/scripts/target/{}.cfg", openocd_path, OPENOCD_TARGET),
        "-c", "init",
        "-c", format!("rtt server start {} 0", RTT_TCP_PORT),
        "-c", "reset init",
        "-c", format!("flash write_image erase {}", elf.display()),
        "-c", "reset halt",
        "-c", format!("rtt setup {} {} {:?}", rtt, RTT_SIZE, RTT_ID),
        "-c", "resume",
        "-c", format!("mww {} {}", rtt + RTT_FLAGS, RTT_BLOCK_IF_FULL),
        "-c", "rtt start",
    )
    .stderr_to_stdout()
    .reader()?;

    let mut lines = BufReader::new(openocd).lines();

    while let Some(line) = lines.next() {
        let line = line?;
        println!("{}", line);

        if line.contains("wrote") {
            break;
        }
    }

    cmd!("nc", "localhost", RTT_TCP_PORT.to_string())
        .pipe(cmd!("defmt-print", "-e", &elf))
        .run()?;

    // close `openocd` *after* `nc`
    drop(lines);

    Ok(())
}

