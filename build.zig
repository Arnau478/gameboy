const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const sdl3_dep = b.dependency("sdl3", .{
        .target = target,
        .optimize = optimize,
    });

    const exe_mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });
    exe_mod.addAnonymousImport("default_boot_rom", .{
        .root_source_file = b.path("boot.bin"),
    });
    exe_mod.addImport("sdl", sdl3_dep.module("sdl3"));

    const exe = b.addExecutable(.{
        .name = "gameboy",
        .root_module = exe_mod,
    });

    b.installArtifact(exe);
}
