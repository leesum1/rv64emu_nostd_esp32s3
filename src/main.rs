#![no_std]
#![no_main]

#[macro_use]
extern crate alloc;

use alloc::{boxed::Box, rc::Rc, vec::Vec};

use esp_backtrace as _;
use esp_println::logger::init_logger;
use esp_println::println;

mod bin_file;

use esp32s3_hal::{
    clock::{ClockControl, CpuClock},
    peripherals::Peripherals,
    prelude::*,
    psram,
    timer::TimerGroup,
    Delay, Rtc, Uart,
};
use log::info;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 200 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr(), HEAP_SIZE);
    }
}

fn init_psram_heap() {
    unsafe {
        ALLOCATOR.init(psram::psram_vaddr_start() as *mut u8, psram::PSRAM_BYTES);
    }
}

use rv64emu::{
    device::{
        device_16550a::Device16550aUART, device_memory::DeviceMemory,
        device_sifive_plic::SIFIVE_UART_IRQ, device_sifive_uart::DeviceSifiveUart,
        device_trait::MEM_BASE,
    },
    rv64core::{
        bus::{Bus, DeviceType},
        cpu_core::{CpuCoreBuild, CpuState},
    },
    rvsim::RVsim,
    tools::{fifo_unbounded_new, rc_refcell_new},
};

#[entry]
fn main() -> ! {
    #[cfg(debug_assertions)]
    compile_error!("PSRAM on ESP32-S3 needs to be built in release mode");
    init_logger(log::LevelFilter::Debug);

    let peripherals: Peripherals = Peripherals::take();

    psram::init_psram(peripherals.PSRAM);
    // init_psram_heap();

    init_heap();

    let mut system = peripherals.SYSTEM.split();

    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

    // let mut peripheral_clock_control = system.peripheral_clock_control;

    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut timer0 = timer_group0.timer0;
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let mut serial0 = Uart::new(peripherals.UART0, &mut system.peripheral_clock_control);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();
    let mut delay = Delay::new(&clocks);

    // // create system bus, which functions are as follows
    // // 1. manage all devices,including plic,clint,and sram
    // // 2. shared by all harts

    let bus_u = rc_refcell_new(Bus::new());

    let mut config = rv64emu::config::Config::new();
    config.set_tlb_size(16);
    config.set_icache_size(0);
    config.set_decode_cache_size(1024 + 512);
    config.set_mmu_type("sv39"); // sv39 sv48 sv57
    config.set_isa("rv64imac");
    let config = Rc::new(config);

    // create hart0 with smode support, some additional features are as follows
    // 1. the first instruction is executed at 0x8000_0000
    // 2. hart0 id is 0
    // 3. smode is enabled
    let mut hart0 = CpuCoreBuild::new(bus_u.clone(), config)
        .with_boot_pc(0x8000_0000)
        .with_hart_id(0)
        .with_smode(true)
        .build();

    // create device dram with 8M capacity
    // load binary file to dram start address (0x8000_0000)
    // Mount the dram under the bus
    let slice = unsafe {
        Vec::from_raw_parts(
            psram::psram_vaddr_start() as *mut u8,
            psram::PSRAM_BYTES,
            psram::PSRAM_BYTES,
        )
    };

    let mut mem = DeviceMemory::from_boxed_slice(slice.into_boxed_slice());

    bus_u.borrow_mut().add_device(DeviceType {
        start: MEM_BASE,
        len: mem.size() as u64,
        instance: Box::new(mem),
        name: "RAM",
    });

    // device 16650_uart
    let uart0_tx_fifo = fifo_unbounded_new::<u8>();
    let uart0_rx_fifo = fifo_unbounded_new::<u8>();

    let device_16650_uart = Device16550aUART::new(uart0_tx_fifo.clone(), uart0_rx_fifo.clone());

    bus_u.borrow_mut().add_device(DeviceType {
        start: 0x1000_0000,
        len: 0x1000,
        instance: Box::new(device_16650_uart),
        name: "16550a_uart",
    });
    // sifive_uart
    let device_sifive_uart = DeviceSifiveUart::new(uart0_tx_fifo.clone(), uart0_rx_fifo.clone());

    bus_u
        .borrow_mut()
        .plic
        .instance
        .register_irq_source(SIFIVE_UART_IRQ, device_sifive_uart.irq_pending.clone());

    bus_u.borrow_mut().add_device(DeviceType {
        start: 0xC000_0000,
        len: 0x1000,
        instance: Box::new(device_sifive_uart),
        name: "sifive uart",
    });

    // print bus device map
    info!("{0}", bus_u.borrow());

    hart0.cpu_state = CpuState::Running;

    let harts_vec = vec![hart0];
    let mut sim = RVsim::new(harts_vec);

    sim.load_image_from_slice(bin_file::LINUX_FILE);
    info!("heap_free: {}", ALLOCATOR.free());

    loop {
        sim.run_once();

        if let Ok(c) = serial0.read() {
            uart0_rx_fifo.push(c)
        }
        // uart0_tx_fifo
        while let Some(c) = uart0_tx_fifo.pop() {
            serial0.write(c);
        }
    }
}
