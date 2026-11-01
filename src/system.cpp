// The C runtime calls these from __libc_init_array; crti/crtn are not linked with -nostartfiles.
extern "C" {
void _init() {}
void _fini() {}
}

// The clock is configured explicitly in main() (as in the Rust port), so no system init is needed here.
extern "C" void SystemInit() {}
