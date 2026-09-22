# NCS/Zephyr toolchain environment for this machine - the toolchain is
# self-contained under $TC (its own python, ARM GCC/zephyr-sdk, cmake,
# ninja) and isolated from the system, so `west build` needs these set
# explicitly rather than relying on anything installed globally.
#
# NCS v3.4.1 (was v2.8.0 - toolchain b81a7cd864/SDK ~/ncs/v2.8.0, both still
# on disk if ever needed again, e.g. `git log` this file). Note the Zephyr
# SDK layout changed between these two toolchain releases: v3.4.1's bundle
# nests the GCC toolchains under opt/zephyr-sdk/gnu/... (was
# opt/zephyr-sdk/... directly in the v2.8.0 toolchain).
#
# Usage: `. ncs_env.sh` (source it, don't execute it) before running `west`
# directly; the Makefile already does this for its own targets.

TC=/home/cli/ncs/toolchains/8285d8ad56
export PATH="$TC/bin:$TC/usr/bin:$TC/usr/local/bin:$TC/opt/bin:$TC/opt/nanopb/generator-bin:$TC/opt/zephyr-sdk/gnu/arm-zephyr-eabi/bin:$TC/opt/zephyr-sdk/gnu/riscv64-zephyr-elf/bin:$PATH"
export LD_LIBRARY_PATH="$TC/lib:$TC/lib/x86_64-linux-gnu:$TC/usr/local/lib:$LD_LIBRARY_PATH"
export GIT_EXEC_PATH="$TC/usr/local/libexec/git-core"
export GIT_TEMPLATE_DIR="$TC/usr/local/share/git-core/templates"
export PYTHONHOME="$TC/usr/local"
export PYTHONPATH="$TC/usr/local/lib/python3.12:$TC/usr/local/lib/python3.12/site-packages"
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_INSTALL_DIR="$TC/opt/zephyr-sdk"
export ZEPHYR_BASE=/home/cli/ncs/v3.4.1/zephyr
