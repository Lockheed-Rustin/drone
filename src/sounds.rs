use rodio::OutputStream;
use std::{
    io::{BufReader, Cursor},
    thread,
};

pub const CRASH_BYTES: &[u8] = include_bytes!("../assets/sounds/crash.mp3");
pub const DROP_BYTES: &[u8] = include_bytes!("../assets/sounds/drop.mp3");
pub const EASTER_EGG_BYTES: &[u8] = include_bytes!("../assets/sounds/easter_egg.mp3");

fn play_bytes(bytes: &'static [u8]) {
    thread::spawn(move || {
        let Ok((_stream, stream_handle)) = OutputStream::try_default() else {
            return;
        };
        let buff = BufReader::new(Cursor::new(bytes));
        let Ok(sync) = stream_handle.play_once(buff) else {
            return;
        };
        sync.sleep_until_end();
    });
}

pub fn play_crash() {
    play_bytes(CRASH_BYTES);
}

pub fn play_drop() {
    play_bytes(DROP_BYTES);
}

pub fn play_easter_egg() {
    play_bytes(EASTER_EGG_BYTES);
}
