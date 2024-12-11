use crate::drone::LockheedRustin;
use rodio::{Decoder, Source};
use std::io::{BufReader, Cursor};

pub(crate) const CRASH_SOUND: &[u8] = include_bytes!("../assets/sounds/crash.mp3");
pub(crate) const DROP_SOUND: &[u8] = include_bytes!("../assets/sounds/drop.mp3");
pub(crate) const EASTER_EGG_SOUND: &[u8] = include_bytes!("../assets/sounds/easter_egg.mp3");

impl LockheedRustin {
    pub(crate) fn play_sound(&self, sound: &'static [u8]) {
        let buff = BufReader::new(Cursor::new(sound));
        let source = Decoder::new(buff).unwrap();
        self.sound_sys.1.play_raw(source.convert_samples()).unwrap();
    }
}
