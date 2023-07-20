fn main() {
    cc::Build::new()
        .file("src/import_ov2640_registers.c")
        .compile("ov2640_registers");
}