#[cfg(target_os = "linux")]
mod raspberry_pi;

#[cfg(target_os = "linux")]
fn main() {
    if let Err(error) = raspberry_pi::run() {
        eprintln!("error: {error}");
        std::process::exit(1);
    }
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!(
        "stepper-stabilizer's hardware executable runs on Raspberry Pi OS; \
         use `cargo test` on this platform to test the controller"
    );
}
