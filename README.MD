# rv64emu no_std example

This is a simple example of how to use the `rv64emu` crate in a `no_std` environment.

+ board: esp32-s3 with 8MB PSRAM, 8MB flash
+ environment: esp-rs/esp-hal

For more information about `esp-rs`, please visit [github.com/esp-rs](https://github.com/esp-rs).

# Troubleshooting

**crossbeam-utils**
```
error[E0432]: unresolved imports `core::sync::atomic::AtomicI64`, `core::sync::atomic::AtomicU64`
  --> /home/leesum/.cargo/registry/src/mirrors.ustc.edu.cn-61ef6e0cd06fb9b8/crossbeam-utils-0.8.16/src/lib.rs:79:49
```
Because `esp32-s3` is a `xTensa` architecture, and the toolchain is `"xtensa-esp32s3-none-elf"`, which have't support by rust mainline yet. So we need to add `"xtensa-esp32s3-none-elf"` to `crossbeam-utils-0.8.16/no_atomic.rs`

```rust
#[allow(dead_code)] // Only crossbeam-utils uses this.
const NO_ATOMIC_64: &[&str] = &[
    "arm-linux-androideabi",
    "armv4t-none-eabi",
    "armv4t-unknown-linux-gnueabi",
    "armv5te-none-eabi",
    "armv5te-unknown-linux-gnueabi",
    "armv5te-unknown-linux-musleabi",
    "armv5te-unknown-linux-uclibceabi",
    "armv6k-nintendo-3ds",
    "avr-unknown-gnu-atmega328",
    "hexagon-unknown-linux-musl",
    "m68k-unknown-linux-gnu",
    "mips-unknown-linux-gnu",
    "mips-unknown-linux-musl",
    "mips-unknown-linux-uclibc",
    "mipsel-sony-psp",
    "mipsel-sony-psx",
    "mipsel-unknown-linux-gnu",
    "mipsel-unknown-linux-musl",
    "mipsel-unknown-linux-uclibc",
    "mipsel-unknown-none",
    "mipsisa32r6-unknown-linux-gnu",
    "mipsisa32r6el-unknown-linux-gnu",
    "msp430-none-elf",
    "powerpc-unknown-freebsd",
    "powerpc-unknown-linux-gnu",
    "powerpc-unknown-linux-gnuspe",
    "powerpc-unknown-linux-musl",
    "powerpc-unknown-netbsd",
    "powerpc-unknown-openbsd",
    "powerpc-wrs-vxworks",
    "powerpc-wrs-vxworks-spe",
    "riscv32gc-unknown-linux-gnu",
    "riscv32gc-unknown-linux-musl",
    "riscv32i-unknown-none-elf",
    "riscv32im-unknown-none-elf",
    "riscv32imac-unknown-none-elf",
    "riscv32imac-unknown-xous-elf",
    "riscv32imc-unknown-none-elf",
    "thumbv4t-none-eabi",
    "thumbv5te-none-eabi",
    "thumbv6m-none-eabi",
    "thumbv7em-none-eabi",
    "thumbv7em-none-eabihf",
    "thumbv7m-none-eabi",
    "thumbv8m.base-none-eabi",
    "thumbv8m.main-none-eabi",
    "thumbv8m.main-none-eabihf",
    "xtensa-esp32s3-none-elf" // manually add
];
```

