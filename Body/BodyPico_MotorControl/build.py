Import("env")

# https://docs.platformio.org/en/stable//integration/compile_commands.html
env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)
