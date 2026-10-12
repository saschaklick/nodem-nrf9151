//! Zephyr implementation of virtmach's time interrupt (see virtmach's interrupts/time.rs for the
//! function contract) plus the per-frame gate that makes its waits and the tick non-blocking.
//!
//! Waits never block nodem_task: wait_for/wait_until store a wake time and pause the vm, the
//! gate (installed as Env::vm_gate) then skips the vm until that time has passed, while the
//! UART keeps being served. The tick is not a gate but the frame period itself: it replaces the
//! default one, nodem_task asks nodem_runtime_period_ms() for it, so the vm runs again (and the
//! page is redrawn) one tick after the program stopped at a HLT.
//!
//! There is no RTC, so get_time counts from boot and get_date returns 0, 0, 0. All times are
//! u32 milliseconds of uptime and compared wrapping (about 49 days).

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed};

use nodem_rs::virtmach::{self, RuntimeError, VirtMach, VMAtom, Runtime, interrupts::SoftInterrupt};

unsafe extern "C" {
    // k_uptime_get() is a Zephyr syscall wrapper, see nodem_task.c
    fn nodem_uptime_ms() -> u32;
}

fn now() -> u32 {
    unsafe { nodem_uptime_ms() }
}

fn reached(deadline: u32) -> bool {
    now().wrapping_sub(deadline) as i32 >= 0
}

// Only touched from nodem_task's thread, atomics just avoid `static mut`.
/// Tick while the program has not set one (~60 Hz), also used for a tick of 0.
const DEFAULT_TICK_MS: u32 = 17;

static TICK_MS: AtomicU32 = AtomicU32::new(DEFAULT_TICK_MS);
static WAKE_AT: AtomicU32 = AtomicU32::new(0);
static TICK_ACTIVE: AtomicBool = AtomicBool::new(false);
static HOLDING: AtomicBool = AtomicBool::new(false);
static LAST_UNTIL: AtomicU32 = AtomicU32::new(0);

pub struct Time;

pub const INDEX: usize = virtmach::interrupts::time::INDEX as usize;

/// Interval from the (seconds, milliseconds) arguments, None if either is negative.
fn interval(vm: &mut VirtMach) -> Option<u32> {
    let (seconds, milliseconds) = (vm.stack_pop(), vm.stack_pop());
    if seconds < 0 || milliseconds < 0 {
        vm.error = RuntimeError::InterruptError;
        return None;
    }
    Some(seconds as u32 * 1000 + milliseconds as u32)
}

fn sleep_until(vm: &mut VirtMach, deadline: u32) {
    WAKE_AT.store(deadline, Relaxed);
    HOLDING.store(true, Relaxed);
    vm.pause();
}

impl SoftInterrupt for Time {
    fn name(&self) -> &str {
        virtmach::interrupts::time::NAME
    }

    fn call(&mut self, vm: &mut VirtMach) {
        match vm.stack_pop() {
            0 => {
                if let Some(ms) = interval(vm) {
                    sleep_until(vm, now().wrapping_add(ms));
                }
            }
            1 => {
                let Some(ms) = interval(vm) else { return };
                let deadline = LAST_UNTIL.load(Relaxed).wrapping_add(ms);
                if reached(deadline) {
                    // too late: no catching up, the next interval counts from now
                    LAST_UNTIL.store(now(), Relaxed);
                    vm.stack_push(0);
                } else {
                    LAST_UNTIL.store(deadline, Relaxed);
                    vm.stack_push(1);
                    sleep_until(vm, deadline);
                }
            }
            3 => {
                if let Some(ms) = interval(vm) {
                    TICK_MS.store(if ms == 0 { DEFAULT_TICK_MS } else { ms }, Relaxed);
                }
            }
            10 => {
                let t = now();
                vm.stack_push((t / 3_600_000 % 24) as VMAtom);
                vm.stack_push((t / 60_000 % 60) as VMAtom);
                vm.stack_push((t / 1000 % 60) as VMAtom);
                vm.stack_push((t % 1000) as VMAtom);
            }
            11 => {
                for _ in 0..3 {
                    vm.stack_push(0);
                }
            }
            _ => vm.error = RuntimeError::UnimplementedInterruptFunc,
        }
    }
}

/// Env::vm_gate: false while a wait is still running.
pub fn gate(vm: &VirtMach) -> bool {
    if vm.cycle_cnt == 0 {
        // fresh program: no tick, waits count from now
        TICK_MS.store(DEFAULT_TICK_MS, Relaxed);
        HOLDING.store(false, Relaxed);
        LAST_UNTIL.store(now(), Relaxed);
    }
    TICK_ACTIVE.store(vm.state == Runtime::Hlt || vm.state == Runtime::Run, Relaxed);
    if !HOLDING.load(Relaxed) {
        return true;
    }
    if reached(WAKE_AT.load(Relaxed)) {
        HOLDING.store(false, Relaxed);
        true
    } else {
        false
    }
}

/// Period of the next frame in ms: the program's tick, DEFAULT_TICK_MS if it set none or has
/// stopped, shortened to end at the wake time of a running wait.
#[unsafe(no_mangle)]
pub extern "C" fn nodem_runtime_period_ms() -> u32 {
    let tick = TICK_MS.load(Relaxed);
    if !TICK_ACTIVE.load(Relaxed) {
        return DEFAULT_TICK_MS;
    }
    if HOLDING.load(Relaxed) {
        let left = WAKE_AT.load(Relaxed).wrapping_sub(now()) as i32;
        return tick.min(left.max(1) as u32);
    }
    tick
}
