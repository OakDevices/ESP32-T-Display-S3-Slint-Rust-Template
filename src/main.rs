#![no_std]
#![cfg_attr(not(feature = "simulator"), no_main)]

extern crate alloc;

slint::include_modules!();

#[cfg(not(feature = "simulator"))]
use {
    cst816s::CST816S,
    display_interface_parallel_gpio::{Generic8BitBus, PGPIO8BitInterface},
    embedded_hal::digital::v2::OutputPin,
    esp_alloc::EspHeap,
    esp_backtrace as _,
    hal::{
        clock::{ClockControl, CpuClock},
        i2c::I2C,
        mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, PeripheralClockConfig, MCPWM},
        peripherals::Peripherals,
        prelude::*,
        systimer::SystemTimer,
        timer::TimerGroup,
        Delay, Rtc, IO,
    },
    mipidsi::{Builder, ColorInversion, Display},
    slint::platform::WindowEvent,
};

fn create_slint_app() -> AppWindow {
    let ui = AppWindow::new().expect("Failed to load UI");

    let ui_handle = ui.as_weak();
    ui.on_request_increase_value(move || {
        let ui = ui_handle.unwrap();
        ui.set_counter(ui.get_counter() + 1);
    });
    ui
}

#[cfg(feature = "simulator")]
fn main() -> Result<(), slint::PlatformError> {
    create_slint_app().run()
}

#[cfg(not(feature = "simulator"))]
#[hal::entry]
fn main() -> ! {
    // HEAP configuration
    const HEAP_SIZE: usize = 250 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    #[global_allocator]
    static ALLOCATOR: EspHeap = EspHeap::empty();
    unsafe { ALLOCATOR.init(&mut HEAP as *mut u8, core::mem::size_of_val(&HEAP)) }

    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    // Configure LCD display GPIO
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let d0 = io.pins.gpio39.into_push_pull_output();
    let d1 = io.pins.gpio40.into_push_pull_output();
    let d2 = io.pins.gpio41.into_push_pull_output();
    let d3 = io.pins.gpio42.into_push_pull_output();
    let d4 = io.pins.gpio45.into_push_pull_output();
    let d5 = io.pins.gpio46.into_push_pull_output();
    let d6 = io.pins.gpio47.into_push_pull_output();
    let d7 = io.pins.gpio48.into_push_pull_output();
    let data_command_selection = io.pins.gpio7.into_push_pull_output();
    let write_enable = io.pins.gpio8.into_push_pull_output();
    let reset = io.pins.gpio5.into_push_pull_output();
    let backlight = io.pins.gpio38;
    let mut chip_select = io.pins.gpio6.into_push_pull_output();
    let mut read_enable = io.pins.gpio9.into_push_pull_output();

    chip_select.set_low().unwrap(); // set Chip Select pin to low (always selected)
    read_enable.set_high().unwrap();

    // TODO: Exchange generic 8 bit bus implementation to native LCD_CAM interface when drivers become available by esp32s3-hal
    // Create 8-bit wide bus for a display interface
    let bus = Generic8BitBus::new((d0, d1, d2, d3, d4, d5, d6, d7)).unwrap();
    // Create display interface which uses 8-bit wide bus, data_command_selection and write_enable pins.
    let di = PGPIO8BitInterface::new(bus, data_command_selection, write_enable);
    let mut delay = Delay::new(&clocks);
    // Create display controller
    let display = Builder::st7789(di)
        .with_display_size(170, 320)
        .with_invert_colors(ColorInversion::Inverted)
        .init(&mut delay, Some(reset))
        .unwrap();

    // Configure timers for PWM control of the backlight
    let clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 40u32.MHz()).unwrap();
    let mut mcpwm = MCPWM::new(
        peripherals.MCPWM0,
        clock_cfg,
        &mut system.peripheral_clock_control,
    );
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    let mut backlight_pwm_pin = mcpwm
        .operator0
        .with_pin_a(backlight, PwmPinConfig::UP_ACTIVE_HIGH);
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 20u32.kHz())
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    // Configure platform for Slint
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(Default::default());

    slint::platform::set_platform(alloc::boxed::Box::new(SlintPlatform {
        window: window.clone(),
    }))
    .unwrap();

    struct SlintPlatform {
        window: alloc::rc::Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
    }

    impl slint::platform::Platform for SlintPlatform {
        fn create_window_adapter(
            &self,
        ) -> Result<alloc::rc::Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError>
        {
            Ok(self.window.clone())
        }

        fn duration_since_start(&self) -> core::time::Duration {
            core::time::Duration::from_millis(
                SystemTimer::now() / (SystemTimer::TICKS_PER_SECOND / 1000),
            )
        }

        fn debug_log(&self, arguments: core::fmt::Arguments) {
            esp_println::println!("{}", arguments);
        }
    }

    let mut buffer_provider = DrawBuffer {
        display,
        buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); 320],
    };

    // Configure touchscreen driver
    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio18,
        io.pins.gpio17,
        400u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let touch_int = io.pins.gpio16.into_pull_up_input();
    let touch_rst = io.pins.gpio21.into_push_pull_output();

    let mut touch = CST816S::new(i2c, touch_int, touch_rst);
    touch.setup(&mut delay).unwrap();

    let _ui = create_slint_app();

    // Turn on LCD backlight
    backlight_pwm_pin.set_timestamp(10);

    let mut last_touch_action = None;
    let mut last_touch_position = None;

    loop {
        slint::platform::update_timers_and_animations();
        window.draw_if_needed(|renderer| {
            renderer.render_by_line(&mut buffer_provider);
        });

        // TODO: Fix issue with short click. In this case only one interrupt is received and Pointer stays pressed.
        let button = slint::platform::PointerEventButton::Left;
        if let Some(event) = touch.read_one_touch_event(true).map(|record| {
            let position = slint::PhysicalPosition::new(record.x as _, record.y as _)
                .to_logical(window.scale_factor());
            esp_println::println!("{:?}", record);
            last_touch_position.replace(position);
            last_touch_action.replace(record.action);
            match record.action {
                0 => WindowEvent::PointerPressed { position, button },
                1 => WindowEvent::PointerReleased { position, button },
                2 => WindowEvent::PointerMoved { position },
                _ => WindowEvent::PointerExited,
            }
        }) {
            esp_println::println!("{:?}", event);
            let is_pointer_release_event: bool =
                matches!(event, WindowEvent::PointerReleased { .. });
            window.dispatch_event(event);

            // removes hover state on widgets
            if is_pointer_release_event {
                window.dispatch_event(WindowEvent::PointerExited);
            }
        }

        if window.has_active_animations() {
            continue;
        }

        // TODO: we could save battery here by going to sleep up to
        //   slint::platform::duration_until_next_timer_update()
        // or until the next touch interrupt, whatever comes first
    }
}

#[cfg(not(feature = "simulator"))]
struct DrawBuffer<'a, Display> {
    display: Display,
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

#[cfg(not(feature = "simulator"))]
impl<
        DI: display_interface::WriteOnlyDataCommand,
        RST: OutputPin<Error = core::convert::Infallible>,
    > slint::platform::software_renderer::LineBufferProvider
    for &mut DrawBuffer<'_, Display<DI, mipidsi::models::ST7789, RST>>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
    ) {
        let buffer = &mut self.buffer[range.clone()];

        render_fn(buffer);

        // We send empty data just to get the device in the right window
        self.display
            .set_pixels(
                range.start as u16,
                line as _,
                range.end as u16,
                line as u16,
                buffer
                    .iter()
                    .map(|x| embedded_graphics::pixelcolor::raw::RawU16::new(x.0).into()),
            )
            .unwrap();
    }
}
