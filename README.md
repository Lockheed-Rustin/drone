# The best drone ;D

**Shoots missiles**

# Usage

Add this to your `Cargo.toml`:
```toml
[dependencies]
lockheedrustin-drone = { git = "https://github.com/Lockheed-Rustin/drone.git" }
```

To use it in your code do:
```rust
use lockheedrustin_drone::LockheedRustin;
```

If you want to use the meme sounds effects add sounds to the feature tag:
```toml
[dependencies]
lockheedrustin-drone = { git = "https://github.com/Lockheed-Rustin/drone.git", features = [
    "sounds",
] }
```
Note that if you enable sounds the drone cannot be sent between threads.
