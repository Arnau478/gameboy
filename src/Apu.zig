const Apu = @This();

const std = @import("std");

on: bool,
wave_ram: [16]u8,
ch1_config: Ch1Config,

const Ch1Config = struct {
    len_duty: LenDuty,
    volume_envelope: VolumeEnvelope,

    const LenDuty = packed struct(u8) {
        len: u6,
        duty: Duty,
    };

    const VolumeEnvelope = packed struct(u8) {
        sweep_pace: u3,
        env_dir: u1,
        initial_volume: u4,
    };

    const Duty = enum(u2) {
        @"12.5%" = 0b00,
        @"25%" = 0b01,
        @"50%" = 0b10,
        @"75%" = 0b11,
    };
};

pub const Register = enum {
    nr11,
    nr12,
    nr50,
    nr51,
    nr52,
};

pub fn read(apu: Apu, register: Register) u8 {
    return switch (register) {
        .nr11 => @bitCast(apu.ch1_config.len_duty),
        .nr12 => @bitCast(apu.ch1_config.volume_envelope),
        .nr50 => @panic("TODO: NR50"),
        .nr51 => @panic("TODO: NR51"),
        .nr52 => if (apu.on) 0b10000000 else 0, // TODO: Channel enable flags
    };
}

pub fn write(apu: *Apu, register: Register, value: u8) void {
    if (!apu.on and register != .nr52) {
        std.log.warn("Trying to write to a register while the APU is off", .{});
    } else {
        switch (register) {
            .nr11 => apu.ch1_config.len_duty = @bitCast(value),
            .nr12 => apu.ch1_config.volume_envelope = @bitCast(value),
            .nr50 => std.log.warn("TODO: NR50", .{}),
            .nr51 => std.log.warn("TODO: NR51", .{}),
            .nr52 => {
                if (value & 0b00001111 != apu.read(.nr52) & 0b00001111) {
                    std.log.warn("Trying to write to NR52 read-only bits", .{});
                }

                if (value >> 7 != 0) {
                    apu.turnOn();
                } else {
                    apu.turnOff();
                }
            },
        }
    }
}

fn turnOn(apu: *Apu) void {
    if (apu.on) {
        std.log.warn("Trying to turn on the APU, but it's already on", .{});
    }

    apu.on = true;
}

fn turnOff(apu: *Apu) void {
    if (!apu.on) {
        std.log.warn("Trying to turn off the APU, but it's already off", .{});
    }

    apu.* = .{
        .on = false,
        .wave_ram = apu.wave_ram,
        // TODO: DIV-APU counter
        .ch1_config = .{
            .len_duty = @bitCast(@as(u8, 0xaa)),
            .volume_envelope = @bitCast(@as(u8, 0xaa)),
        },
    };
}
