cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nodem_zephyr)

target_sources(app PRIVATE
    src/main.c
    src/config_store.c
    src/display_task.c
    src/heartbeat_task.c
    src/log.c
    src/modem_task.c
    src/nodem_listener.c
    src/nodem_task.c
    src/ota_store.c
    src/pkg_store.c
    src/uart_task.c
)

# modem_task.c embeds the TLS root CA as a byte array via `#include
# "gts-root-r4.pem.inc"` - generate that from the PEM checked in under cert/.
set(modem_cert_gen_dir ${CMAKE_CURRENT_BINARY_DIR}/certs)
zephyr_include_directories(${modem_cert_gen_dir})
generate_inc_file_for_target(
    app
    cert/gts-root-r4.pem
    ${modem_cert_gen_dir}/gts-root-r4.pem.inc
    )

# `nodem-ffi` is built separately (`cargo build -p nodem-ffi --release --target
# thumbv8m.main-none-eabihf`, driven by the top-level `cargo run` wrapper
# before `west build` runs) - it isn't a CMake-built target, just a prebuilt
# static library to link in.
# Soft-float target: this app doesn't enable CONFIG_FPU, so Zephyr's own
# build is soft-float too - a hard-float (`...eabihf`) build of nodem-ffi
# fails to link (`uses VFP register arguments` / ARM EABI attribute mismatch).
set(NODEM_FFI_LIB
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodem-ffi/target/thumbv8m.main-none-eabi/release/libnodem_ffi.a)

if(NOT EXISTS ${NODEM_FFI_LIB})
  message(FATAL_ERROR "nodem-ffi not built yet - run `cargo build -p nodem-ffi --release --target thumbv8m.main-none-eabi` first (expected at ${NODEM_FFI_LIB})")
endif()

# `zephyr_library_import`'s registration (a CMake global property consumed
# while `find_package(Zephyr)` processes zephyr/CMakeLists.txt) resolves
# before this file runs, so a library added that way silently never reaches
# the final link. Use a plain CMake imported target + `target_link_libraries`
# instead, per zephyr/samples/application_development/external_lib.
add_library(nodem_ffi STATIC IMPORTED GLOBAL)
set_target_properties(nodem_ffi PROPERTIES IMPORTED_LOCATION ${NODEM_FFI_LIB})
target_link_libraries(app PUBLIC nodem_ffi)
