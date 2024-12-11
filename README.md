# The best drone ;D

**Shoots missiles**

# Usage

Add this to your `Cargo.toml`:
```toml
[dependencies]
Lockheed-Rustin = { git = "https://github.com/Lockheed-Rustin/drone.git", package = "drone" }
```

To use it in your code do:
```rust
use Lockheed_Rustin::LockheedRustin;
```

If you want to use the meme sounds effects add sounds to the feature tag:
```toml
[dependencies]
Lockheed-Rustin = { git = "https://github.com/Lockheed-Rustin/drone.git", package = "drone", features = [
    "sounds",
] }
```
Note that if you enable sounds the drone cannot be sent between thread.
