const Cpu = @This();

const std = @import("std");
const Ppu = @import("Ppu.zig");
const Apu = @import("Apu.zig");

regs: Regs,
ime: bool,
ei_queued: bool,
ie: u8,
@"if": u8,
halt: bool,
boot_rom: [256]u8,
boot_rom_enable: u8,
cartridge: [0x8000]u8,
wram: [0x2000]u8,
hram: [127]u8,
ppu: Ppu,
apu: Apu,

pub fn init(boot_rom: [256]u8, cartridge: [0x8000]u8) Cpu {
    return .{
        .regs = .{
            .af = 0xaa00,
            .bc = 0xaaaa,
            .de = 0xaaaa,
            .hl = 0xaaaa,
            .pc = 0,
            .sp = 0xaaaa,
        },
        .ime = false,
        .ei_queued = false,
        .ie = 0,
        .@"if" = 0,
        .halt = false,
        .boot_rom = boot_rom,
        .boot_rom_enable = 0,
        .cartridge = cartridge,
        .wram = [1]u8{0xaa} ** 0x2000,
        .hram = [1]u8{0xaa} ** 127,
        .ppu = .{
            .vram = [1]u8{0xaa} ** 0x2000,
            .mode = .oam_scan,
            .mode_ticks = 0,
            .line = 0,
            .lyc = 0,
            .scx = 0,
            .scy = 0,
            .int_select = @bitCast(@as(u4, 0)),
            .bg_palette = 0,
            .lcd_control = @bitCast(@as(u8, 0)),
            .lcd = [1][160]u2{[1]u2{0} ** 160} ** 144,
        },
        .apu = .{
            .on = false,
            .wave_ram = [1]u8{0xaa} ** 16,
            .ch1_config = .{
                .len_duty = @bitCast(@as(u8, 0xaa)),
                .volume_envelope = @bitCast(@as(u8, 0xaa)),
            },
        },
    };
}

pub const Regs = struct {
    af: u16,
    bc: u16,
    de: u16,
    hl: u16,
    pc: u16,
    sp: u16,

    pub fn a(regs: *Regs) *u8 {
        return &std.mem.asBytes(&regs.af)[1];
    }

    pub fn f(regs: *Regs) *u8 {
        return &std.mem.asBytes(&regs.af)[0];
    }

    pub fn b(regs: *Regs) *u8 {
        return &std.mem.asBytes(&regs.bc)[1];
    }

    pub fn c(regs: *Regs) *u8 {
        return &std.mem.asBytes(&regs.bc)[0];
    }

    pub fn d(regs: *Regs) *u8 {
        return &std.mem.asBytes(&regs.de)[1];
    }

    pub fn e(regs: *Regs) *u8 {
        return &std.mem.asBytes(&regs.de)[0];
    }

    pub fn h(regs: *Regs) *u8 {
        return &std.mem.asBytes(&regs.hl)[1];
    }

    pub fn l(regs: *Regs) *u8 {
        return &std.mem.asBytes(&regs.hl)[0];
    }

    pub fn flags(regs: *Regs) *Flags {
        return @ptrCast(regs.f());
    }

    pub const Flags = packed struct(u8) {
        padding: u4 = 0,
        carry: bool,
        half_carry: bool,
        subtract: bool,
        zero: bool,
    };
};

pub fn readMemory(cpu: Cpu, address: u16) u8 {
    return switch (address) {
        0x0000...0x00ff => if (cpu.boot_rom_enable == 0) cpu.boot_rom[address] else cpu.cartridge[address],
        0x0100...0x7fff => cpu.cartridge[address],
        0x8000...0x9fff => cpu.ppu.vram[address - 0x8000],
        0xc000...0xdfff => cpu.wram[address - 0xc000],
        0xfea0...0xfeff => 0,
        0xff0f => cpu.@"if",
        0xff11 => cpu.apu.read(.nr11),
        0xff12 => cpu.apu.read(.nr12),
        0xff24 => cpu.apu.read(.nr50),
        0xff25 => cpu.apu.read(.nr51),
        0xff26 => cpu.apu.read(.nr52),
        0xff40 => cpu.ppu.read(.lcdc),
        0xff42 => cpu.ppu.read(.scy),
        0xff43 => cpu.ppu.read(.scx),
        0xff44 => cpu.ppu.read(.ly),
        0xff45 => cpu.ppu.read(.lyc),
        0xff47 => cpu.ppu.read(.bgp),
        0xff50 => cpu.boot_rom_enable,
        0xff80...0xfffe => cpu.hram[address - 0xff80],
        0xffff => cpu.ie,
        else => std.debug.panic("TODO: Read from address 0x{x:0>4}", .{address}),
    };
}

pub fn writeMemory(cpu: *Cpu, address: u16, value: u8) void {
    return switch (address) {
        0x0000...0x7fff => std.log.warn("Ignoring write to 0x{x:0>4}", .{address}),
        0x8000...0x9fff => cpu.ppu.vram[address - 0x8000] = value,
        0xc000...0xdfff => cpu.wram[address - 0xc000] = value,
        0xfea0...0xfeff => {},
        0xff0f => cpu.@"if" = value,
        0xff11 => cpu.apu.write(.nr11, value),
        0xff12 => cpu.apu.write(.nr12, value),
        0xff24 => cpu.apu.write(.nr50, value),
        0xff25 => cpu.apu.write(.nr51, value),
        0xff26 => cpu.apu.write(.nr52, value),
        0xff40 => cpu.ppu.write(.lcdc, value),
        0xff42 => cpu.ppu.write(.scy, value),
        0xff43 => cpu.ppu.write(.scx, value),
        0xff44 => cpu.ppu.write(.ly, value),
        0xff45 => cpu.ppu.write(.lyc, value),
        0xff47 => cpu.ppu.write(.bgp, value),
        0xff50 => {
            std.log.debug("Disabling boot ROM", .{});
            cpu.boot_rom_enable = value;
        },
        0xff80...0xfffe => cpu.hram[address - 0xff80] = value,
        0xffff => cpu.ie = value,
        else => std.log.warn("TODO: Write 0x{x:0>2} to address 0x{x:0>4}", .{ value, address }),
    };
}

pub const InterruptType = enum(u3) {
    vblank = 0,
    stat = 1,
    timer = 2,
    serial = 3,
    joypad = 4,
};

pub fn requestInterrupt(cpu: *Cpu, interrupt: InterruptType) void {
    cpu.writeMemory(0xff0f, cpu.readMemory(0xff0f) | (@as(u8, 1) << @intFromEnum(interrupt)));
}

pub const Instruction = union(enum) {
    nop,
    ld: LoadType,
    inc: IncDecTarget,
    dec: IncDecTarget,
    rla,
    jr: JumpRelative,
    halt,
    add: Add,
    sub: ArithmeticTarget,
    @"and": ArithmeticTarget,
    xor: ArithmeticTarget,
    @"or": ArithmeticTarget,
    cp: ArithmeticTarget,
    ret: ?JumpCondition,
    pop: StackReg,
    jp: Jump,
    call: Call,
    push: StackReg,
    reti,
    di,
    ei,
    bit: BitArgs,
    rl: PrefixReg,
    set: BitArgs,

    pub const Indirect = union(enum) {
        bci,
        dei,
        last_byte,
        a16: u16,
        hli_dec,
        hli_inc,
    };

    pub const LoadType = union(enum) {
        byte: Byte,
        word: Word,
        indirect_from_a: Indirect,
        a_from_indirect: Indirect,
        byte_address_from_a: u8,
        a_from_byte_address: u8,

        pub const Byte = struct {
            pub const Target = enum {
                a,
                b,
                c,
                d,
                e,
                h,
                l,
                hli,
            };

            pub const Source = union(enum) {
                a,
                b,
                c,
                d,
                e,
                h,
                l,
                d8: u8,
                hli,
            };

            target: Target,
            source: Source,
        };

        pub const Word = struct {
            pub const Target = enum {
                bc,
                de,
                hl,
                sp,
            };

            target: Target,
            source: u16,
        };
    };

    pub const IncDecTarget = enum {
        a,
        b,
        c,
        d,
        e,
        h,
        l,
        bc,
        de,
        hl,
        sp,
        hli,
    };

    pub const JumpCondition = enum {
        z,
        nz,
        c,
        nc,
    };

    pub const JumpRelative = struct {
        condition: ?JumpCondition,
        rel: i8,
    };

    pub const ArithmeticTarget = union(enum) {
        a,
        b,
        c,
        d,
        e,
        h,
        l,
        hli,
        d8: u8,
    };

    pub const Add = union(enum) {
        byte: ArithmeticTarget,
        hl: enum {
            bc,
            de,
            hl,
            sp,
        },
        sp: i8,
    };

    pub const Jump = struct {
        condition: ?JumpCondition,
        source: union(enum) {
            hl,
            a16: u16,
        },
    };

    pub const Call = struct {
        condition: ?JumpCondition,
        a16: u16,
    };

    pub const StackReg = enum {
        af,
        bc,
        de,
        hl,
    };

    pub const PrefixReg = enum {
        a,
        b,
        c,
        d,
        e,
        h,
        l,
        hli,
    };

    pub const BitArgs = struct {
        bit: u3,
        reg: PrefixReg,
    };

    pub fn format(instruction: Instruction, comptime _: []const u8, _: std.fmt.FormatOptions, writer: anytype) !void {
        switch (instruction) {
            .nop => try writer.print("nop", .{}),
            .ld => |load_type| {
                try writer.print("ld ", .{});
                switch (load_type) {
                    .byte => |byte| {
                        switch (byte.target) {
                            .hli => try writer.print("(hl)", .{}),
                            else => try writer.print("{s}", .{@tagName(byte.target)}),
                        }
                        try writer.print(", ", .{});
                        switch (byte.source) {
                            .d8 => |d8| try writer.print("0x{x:0>2}", .{d8}),
                            .hli => try writer.print("(hl)", .{}),
                            else => try writer.print("{s}", .{@tagName(byte.source)}),
                        }
                    },
                    .word => |word| {
                        switch (word.target) {
                            else => try writer.print("{s}", .{@tagName(word.target)}),
                        }
                        try writer.print(", 0x{x:0>4}", .{word.source});
                    },
                    .indirect_from_a => |target| {
                        switch (target) {
                            .bci => try writer.print("(bc)", .{}),
                            .dei => try writer.print("(de)", .{}),
                            .last_byte => try writer.print("(0xff00+c)", .{}),
                            .a16 => |a16| try writer.print("(0x{x:0>4})", .{a16}),
                            .hli_dec => try writer.print("(hl-)", .{}),
                            .hli_inc => try writer.print("(hl+)", .{}),
                        }
                        try writer.print(", a", .{});
                    },
                    .a_from_indirect => |source| {
                        try writer.print("a, ", .{});
                        switch (source) {
                            .bci => try writer.print("(bc)", .{}),
                            .dei => try writer.print("(de)", .{}),
                            .last_byte => try writer.print("(0xff00+c)", .{}),
                            .a16 => |a16| try writer.print("(0x{x:0>4})", .{a16}),
                            .hli_dec => try writer.print("(hl-)", .{}),
                            .hli_inc => try writer.print("(hl+)", .{}),
                        }
                    },
                    .byte_address_from_a => |a8| try writer.print("(0xff00+0x{x:0>2}), a", .{a8}),
                    .a_from_byte_address => |a8| try writer.print("a, (0xff00+0x{x:0>2})", .{a8}),
                }
            },
            .inc, .dec => |target| {
                try writer.print("{s}", .{@tagName(instruction)});
                try writer.print(" ", .{});
                switch (target) {
                    .hli => try writer.print("(hl)", .{}),
                    else => try writer.print("{s}", .{@tagName(target)}),
                }
            },
            .rla => try writer.print("rla", .{}),
            .jr => |jr| {
                try writer.print("jr ", .{});
                if (jr.condition) |condition| {
                    try writer.print("{s}, ", .{@tagName(condition)});
                }
                try writer.print("{s}0x{x:0>2}", .{ if (jr.rel < 0) "-" else "", @abs(jr.rel) });
            },
            .halt => try writer.print("halt", .{}),
            .add => |add| {
                try writer.print("{s} a ", .{@tagName(instruction)});
                switch (add) {
                    .byte => |target| switch (target) {
                        .d8 => |d8| try writer.print("0x{x:0>2}", .{d8}),
                        .hli => try writer.print("(hl)", .{}),
                        else => try writer.print("{s}", .{@tagName(target)}),
                    },
                    else => try writer.print("{s}", .{@tagName(add)}),
                }
            },
            .sub, .@"and", .xor, .@"or", .cp => |target| {
                try writer.print("{s} ", .{@tagName(instruction)});
                switch (target) {
                    .d8 => |d8| try writer.print("0x{x:0>2}", .{d8}),
                    .hli => try writer.print("(hl)", .{}),
                    else => try writer.print("{s}", .{@tagName(target)}),
                }
            },
            .ret => |condition| {
                try writer.print("ret", .{});
                if (condition) |c| {
                    try writer.print(" {s}", .{@tagName(c)});
                }
            },
            .pop => |reg| try writer.print("pop {s}", .{@tagName(reg)}),
            .jp => |jump| {
                try writer.print("jp ", .{});
                if (jump.condition) |condition| {
                    try writer.print("{s}, ", .{@tagName(condition)});
                }
                switch (jump.source) {
                    .hl => try writer.print("hl", .{}),
                    .a16 => |a16| try writer.print("0x{x:0>4}", .{a16}),
                }
            },
            .call => |call| {
                try writer.print("call ", .{});
                if (call.condition) |condition| {
                    try writer.print("{s}, ", .{@tagName(condition)});
                }
                try writer.print("0x{x:0>4}", .{call.a16});
            },
            .push => |reg| try writer.print("push {s}", .{@tagName(reg)}),
            .reti, .di, .ei => {
                try writer.print("{s}", .{@tagName(instruction)});
            },
            .bit, .set => |bit_args| try writer.print("{s} {d}, {s}", .{ @tagName(instruction), bit_args.bit, switch (bit_args.reg) {
                .hli => "(hl)",
                else => |r| @tagName(r),
            } }),
            .rl => |reg| try writer.print("rl {s}", .{switch (reg) {
                .hli => "(hl)",
                else => |r| @tagName(r),
            }}),
        }
    }
};

fn push(cpu: *Cpu, value: u16) void {
    cpu.regs.sp -%= 1;
    cpu.writeMemory(cpu.regs.sp, @intCast(value >> 8));
    cpu.regs.sp -%= 1;
    cpu.writeMemory(cpu.regs.sp, @truncate(value));
}

fn pop(cpu: *Cpu) u16 {
    var value: u16 = cpu.readMemory(cpu.regs.sp);
    cpu.regs.sp +%= 1;
    value |= @as(u16, cpu.readMemory(cpu.regs.sp)) << 8;
    cpu.regs.sp +%= 1;
    return value;
}

pub const ExecuteResult = struct {
    pc: u16,
    cycles: u8,
};

pub fn execute(cpu: *Cpu, instruction: Instruction, size: usize, constant_cycles: ?u8) ExecuteResult {
    switch (instruction) {
        .nop => {},
        .ld => |load_type| {
            switch (load_type) {
                .byte => |byte| {
                    const value = switch (byte.source) {
                        .d8 => |d8| d8,
                        .hli => cpu.readMemory(cpu.regs.hl),
                        inline else => |_, source| @field(Regs, @tagName(source))(&cpu.regs).*,
                    };
                    switch (byte.target) {
                        .hli => cpu.writeMemory(cpu.regs.hl, value),
                        inline else => |target| @field(Regs, @tagName(target))(&cpu.regs).* = value,
                    }
                },
                .word => |word| {
                    switch (word.target) {
                        inline else => |target| @field(cpu.regs, @tagName(target)) = word.source,
                    }
                },
                .indirect_from_a, .a_from_indirect => |target| {
                    const address = switch (target) {
                        .bci => cpu.regs.bc,
                        .dei => cpu.regs.de,
                        .last_byte => 0xff00 + @as(u16, cpu.regs.c().*),
                        .a16 => |a16| a16,
                        .hli_dec, .hli_inc => cpu.regs.hl,
                    };

                    switch (load_type) {
                        .indirect_from_a => cpu.writeMemory(address, cpu.regs.a().*),
                        .a_from_indirect => cpu.regs.a().* = cpu.readMemory(address),
                        else => unreachable,
                    }

                    switch (target) {
                        .hli_dec => cpu.regs.hl -%= 1,
                        .hli_inc => cpu.regs.hl +%= 1,
                        else => {},
                    }
                },
                .byte_address_from_a => |a8| {
                    cpu.writeMemory(0xff00 + @as(u16, a8), cpu.regs.a().*);
                },
                .a_from_byte_address => |a8| {
                    cpu.regs.a().* = cpu.readMemory(0xff00 + @as(u16, a8));
                },
            }
        },
        .inc, .dec => |target| {
            var value: u16 = switch (target) {
                inline .a, .b, .c, .d, .e, .h, .l => |t| @field(Regs, @tagName(t))(&cpu.regs).*,
                inline .bc, .de, .hl, .sp => |t| @field(cpu.regs, @tagName(t)),
                .hli => cpu.readMemory(cpu.regs.hl),
            };

            const old_value = value;

            switch (instruction) {
                .inc => value +%= 1,
                .dec => value -%= 1,
                else => unreachable,
            }

            switch (target) {
                inline .a, .b, .c, .d, .e, .h, .l => |t| @field(Regs, @tagName(t))(&cpu.regs).* = @truncate(value),
                inline .bc, .de, .hl, .sp => |t| @field(cpu.regs, @tagName(t)) = value,
                .hli => cpu.writeMemory(cpu.regs.hl, @truncate(value)),
            }

            if (switch (target) {
                .a, .b, .c, .d, .e, .h, .l, .hli => true,
                .bc, .de, .hl, .sp => false,
            }) {
                cpu.regs.flags().* = .{
                    .carry = cpu.regs.flags().carry,
                    .half_carry = switch (instruction) {
                        .inc => @as(u8, @truncate(old_value)) & 0xF == 0xF,
                        .dec => @as(u8, @truncate(old_value)) & 0xF == 0x0,
                        else => unreachable,
                    },
                    .subtract = switch (instruction) {
                        .inc => false,
                        .dec => true,
                        else => unreachable,
                    },
                    .zero = @as(u8, @truncate(value)) == 0,
                };
            }
        },
        .rla => {
            const c = cpu.regs.flags().carry;
            var value = cpu.regs.a().*;

            const b7 = (value & 0x80) >> 7 != 0;
            value = (value << 1) | @intFromBool(c);
            cpu.regs.flags().carry = b7;

            cpu.regs.a().* = value;
        },
        .jr => |jump| {
            if (jump.condition == null or switch (jump.condition.?) {
                .z => cpu.regs.flags().zero,
                .nz => !cpu.regs.flags().zero,
                .c => !cpu.regs.flags().carry,
                .nc => !cpu.regs.flags().carry,
            }) {
                return .{
                    .pc = if (jump.rel < 0)
                        cpu.regs.pc -% @as(u16, @intCast(-jump.rel)) + @as(u16, @intCast(size))
                    else
                        cpu.regs.pc +% @as(u16, @intCast(jump.rel)) + @as(u16, @intCast(size)),
                    .cycles = 3,
                };
            } else {
                return .{
                    .pc = cpu.regs.pc +% @as(u16, @intCast(size)),
                    .cycles = 2,
                };
            }
        },
        .halt => {
            cpu.halt = true;
        },
        .add => |add| {
            switch (add) {
                .byte => |target| {
                    const value = switch (target) {
                        .d8 => |d8| d8,
                        .hli => cpu.readMemory(cpu.regs.hl),
                        inline else => |_, t| @field(Regs, @tagName(t))(&cpu.regs).*,
                    };
                    const new_value, const overflow = @addWithOverflow(cpu.regs.a().*, value);
                    cpu.regs.flags().* = .{
                        .carry = overflow == 1,
                        .half_carry = (cpu.regs.a().* & 0xF) + (value & 0xF) > 0xF,
                        .subtract = false,
                        .zero = new_value == 0,
                    };
                    cpu.regs.a().* = new_value;
                },
                .hl => |reg| {
                    const value = switch (reg) {
                        inline else => |t| @field(cpu.regs, @tagName(t)),
                    };
                    const new_value, const overflow = @addWithOverflow(cpu.regs.hl, value);
                    cpu.regs.flags().* = .{
                        .carry = overflow == 1,
                        .half_carry = (cpu.regs.hl & 0b11111111111) + (value & 0b11111111111) > 0b11111111111,
                        .subtract = false,
                        .zero = cpu.regs.flags().zero,
                    };
                    cpu.regs.hl = new_value;
                },
                .sp => @panic("TODO"),
            }
        },
        .sub => |target| {
            const value = switch (target) {
                .d8 => |d8| d8,
                .hli => cpu.readMemory(cpu.regs.hl),
                inline else => |_, t| @field(Regs, @tagName(t))(&cpu.regs).*,
            };
            const new_value, const overflow = @subWithOverflow(cpu.regs.a().*, value);
            cpu.regs.flags().* = .{
                .carry = overflow == 1,
                .half_carry = (cpu.regs.a().* & 0xF) < (value & 0xF),
                .subtract = true,
                .zero = new_value == 0,
            };
            cpu.regs.a().* = new_value;
        },
        .@"and" => |target| {
            cpu.regs.a().* &= switch (target) {
                .d8 => |d8| d8,
                .hli => cpu.readMemory(cpu.regs.hl),
                inline else => |_, t| @field(Regs, @tagName(t))(&cpu.regs).*,
            };

            cpu.regs.flags().* = .{
                .carry = false,
                .half_carry = true,
                .subtract = false,
                .zero = cpu.regs.a().* == 0,
            };
        },
        .xor => |target| {
            cpu.regs.a().* ^= switch (target) {
                .d8 => |d8| d8,
                .hli => cpu.readMemory(cpu.regs.hl),
                inline else => |_, t| @field(Regs, @tagName(t))(&cpu.regs).*,
            };

            cpu.regs.flags().* = .{
                .carry = false,
                .half_carry = false,
                .subtract = false,
                .zero = cpu.regs.a().* == 0,
            };
        },
        .@"or" => |target| {
            cpu.regs.a().* |= switch (target) {
                .d8 => |d8| d8,
                .hli => cpu.readMemory(cpu.regs.hl),
                inline else => |_, t| @field(Regs, @tagName(t))(&cpu.regs).*,
            };

            cpu.regs.flags().* = .{
                .carry = false,
                .half_carry = false,
                .subtract = false,
                .zero = cpu.regs.a().* == 0,
            };
        },
        .cp => |cp| {
            const value = switch (cp) {
                .d8 => |d8| d8,
                .hli => cpu.readMemory(cpu.regs.hl),
                inline else => |_, r| @field(Regs, @tagName(r))(&cpu.regs).*,
            };

            cpu.regs.flags().* = .{
                .carry = cpu.regs.a().* < value,
                .half_carry = (cpu.regs.a().* & 0xF) < (value & 0xF),
                .subtract = true,
                .zero = cpu.regs.a().* == value,
            };
        },
        .ret => |condition| {
            if (condition == null or switch (condition.?) {
                .z => cpu.regs.flags().zero,
                .nz => !cpu.regs.flags().zero,
                .c => !cpu.regs.flags().carry,
                .nc => !cpu.regs.flags().carry,
            }) {
                return .{
                    .pc = cpu.pop(),
                    .cycles = if (condition) |_| 5 else 4,
                };
            }
        },
        .pop => |reg| {
            switch (reg) {
                inline else => |r| @field(cpu.regs, @tagName(r)) = cpu.pop(),
            }
        },
        .jp => |jump| {
            if (jump.condition == null or switch (jump.condition.?) {
                .z => cpu.regs.flags().zero,
                .nz => !cpu.regs.flags().zero,
                .c => !cpu.regs.flags().carry,
                .nc => !cpu.regs.flags().carry,
            }) {
                return .{
                    .pc = switch (jump.source) {
                        .hl => cpu.regs.hl,
                        .a16 => |a16| a16,
                    },
                    .cycles = if (jump.source == .hl) 1 else 4,
                };
            } else {
                return .{
                    .pc = cpu.regs.pc +% @as(u16, @intCast(size)),
                    .cycles = 3,
                };
            }
        },
        .call => |call| {
            if (call.condition == null or switch (call.condition.?) {
                .z => cpu.regs.flags().zero,
                .nz => !cpu.regs.flags().zero,
                .c => !cpu.regs.flags().carry,
                .nc => !cpu.regs.flags().carry,
            }) {
                cpu.push(cpu.regs.pc +% @as(u16, @intCast(size)));
                return .{
                    .pc = call.a16,
                    .cycles = 6,
                };
            } else {
                return .{
                    .pc = cpu.regs.pc +% @as(u16, @intCast(size)),
                    .cycles = 3,
                };
            }
        },
        .push => |reg| {
            switch (reg) {
                inline else => |r| cpu.push(@field(cpu.regs, @tagName(r))),
            }
        },
        .reti => {
            return .{
                .pc = cpu.pop(),
                .cycles = constant_cycles.?,
            };
        },
        .di => {
            cpu.ime = false;
        },
        .ei => {
            cpu.ei_queued = true;
        },
        .bit => |bit_args| {
            cpu.regs.flags().* = .{
                .carry = cpu.regs.flags().carry,
                .half_carry = true,
                .subtract = false,
                .zero = (switch (bit_args.reg) {
                    .hli => cpu.readMemory(cpu.regs.hl),
                    inline else => |r| @field(Regs, @tagName(r))(&cpu.regs).*,
                } >> bit_args.bit) & 1 == 0,
            };
        },
        .rl => |reg| {
            const c = cpu.regs.flags().carry;
            var value = switch (reg) {
                .hli => cpu.readMemory(cpu.regs.hl),
                inline else => |r| @field(Regs, @tagName(r))(&cpu.regs).*,
            };

            const b7 = (value & 0x80) >> 7 != 0;
            value = (value << 1) | @intFromBool(c);
            cpu.regs.flags().* = .{
                .carry = b7,
                .half_carry = false,
                .subtract = false,
                .zero = value == 0,
            };

            switch (reg) {
                .hli => cpu.writeMemory(cpu.regs.hl, value),
                inline else => |r| @field(Regs, @tagName(r))(&cpu.regs).* = value,
            }
        },
        .set => |bit_args| {
            var value = switch (bit_args.reg) {
                .hli => cpu.readMemory(cpu.regs.hl),
                inline else => |r| @field(Regs, @tagName(r))(&cpu.regs).*,
            };

            value |= (@as(u8, 1) << bit_args.bit);

            switch (bit_args.reg) {
                .hli => cpu.writeMemory(cpu.regs.hl, value),
                inline else => |r| @field(Regs, @tagName(r))(&cpu.regs).* = value,
            }
        },
    }

    return .{ .pc = cpu.regs.pc +% @as(u16, @intCast(size)), .cycles = constant_cycles.? };
}

pub fn tick(cpu: *Cpu) usize {
    std.log.debug("af={[af]x:0>4} bc={[bc]x:0>4} de={[de]x:0>4} hl={[hl]x:0>4} pc={[pc]x:0>4} sp={[sp]x:0>4}", cpu.regs);

    if ((cpu.@"if" & cpu.ie) != 0) {
        for (0..5) |i| {
            cpu.halt = false;

            if (cpu.ime and (((cpu.@"if" & cpu.ie) >> @intCast(i)) & 1) != 0) {
                cpu.@"if" &= ~(@as(u8, 1) << @intCast(i));
                cpu.ime = false;
                const address: u16 = @intCast(0x40 + 8 * i);
                cpu.push(cpu.regs.pc);
                cpu.regs.pc = address;
                std.log.debug("{s} interrupt", .{@tagName(@as(InterruptType, @enumFromInt(i)))});
            }
        }
    }

    if (cpu.ei_queued) {
        cpu.ei_queued = false;
        cpu.ime = true;
    }

    if (!cpu.halt) {
        const opcode = cpu.readMemory(cpu.regs.pc);

        const size = instructionSize(opcode);

        const instruction = switch (size) {
            0 => @panic("TODO"),
            1 => decode(&.{opcode}),
            2 => decode(&.{ opcode, cpu.readMemory(cpu.regs.pc + 1) }),
            3 => decode(&.{ opcode, cpu.readMemory(cpu.regs.pc + 1), cpu.readMemory(cpu.regs.pc + 2) }),
        };

        const cycles = constantInstructionCycles(opcode, if (size > 1) cpu.readMemory(cpu.regs.pc + 1) else null);

        std.log.debug("{}", .{instruction});

        const execute_res = cpu.execute(instruction, size, cycles);
        cpu.regs.pc = execute_res.pc;

        for (0..execute_res.cycles) |_| {
            if (cpu.ppu.tick()) |int| cpu.requestInterrupt(int);
        }

        return execute_res.cycles;
    } else {
        return 0;
    }
}

pub fn instructionSize(opcode: u8) u2 {
    if (opcode == 0xcb) return 2;

    return ([256]u2{
        1, 3, 1, 1, 1, 1, 2, 1, 3, 1, 1, 1, 1, 1, 2, 1,
        1, 3, 1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1,
        2, 3, 1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1,
        2, 3, 1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 3, 3, 3, 1, 2, 1, 1, 1, 3, 1, 3, 3, 2, 1,
        1, 1, 3, 0, 3, 1, 2, 1, 1, 1, 3, 0, 3, 0, 2, 1,
        2, 1, 1, 0, 0, 1, 2, 1, 2, 1, 3, 0, 0, 0, 2, 1,
        2, 1, 1, 1, 0, 1, 2, 1, 2, 1, 3, 1, 0, 0, 2, 1,
    })[opcode];
}

pub fn constantInstructionCycles(opcode: u8, next: ?u8) ?u8 {
    if (opcode != 0xcb) {
        return ([256]?u8{
            1,    3, 2, 2,    1,    1, 2, 1, 5, 2, 2, 2, 1, 1, 2, 1,
            1,    3, 2, 2,    1,    1, 2, 1, 3, 2, 2, 2, 1, 1, 2, 1,
            null, 3, 2, 2,    1,    1, 2, 1, 0, 2, 2, 2, 1, 1, 2, 1,
            null, 3, 2, 2,    3,    3, 3, 1, 0, 2, 2, 2, 1, 1, 2, 1,
            1,    1, 1, 1,    1,    1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
            1,    1, 1, 1,    1,    1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
            1,    1, 1, 1,    1,    1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
            2,    2, 2, 2,    2,    2, 1, 2, 1, 1, 1, 1, 1, 1, 2, 1,
            1,    1, 1, 1,    1,    1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
            1,    1, 1, 1,    1,    1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
            1,    1, 1, 1,    1,    1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
            1,    1, 1, 1,    1,    1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
            null, 3, 0, 4,    0,    4, 2, 4, 0, 4, 0, 1, 0, 6, 2, 4,
            null, 3, 0, 0,    0,    4, 2, 4, 0, 4, 0, 0, 0, 0, 2, 4,
            3,    3, 2, null, 0,    4, 2, 4, 4, 1, 4, 0, 0, 0, 2, 4,
            3,    3, 2, 1,    null, 4, 2, 4, 3, 2, 4, 1, 0, 0, 2, 4,
        })[opcode];
    } else {
        return ([256]u8{
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
            2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 4, 2,
        })[next.?];
    }
    return 0;
}

pub fn decode(bytes: []const u8) Instruction {
    const opcode = bytes[0];
    std.debug.assert(instructionSize(opcode) == bytes.len);

    return switch (opcode) {
        0x00 => .nop,
        0x01 => .{ .ld = .{ .word = .{ .target = .bc, .source = @as(u16, bytes[1]) + (@as(u16, bytes[2]) << 8) } } },
        0x03 => .{ .inc = .bc },
        0x04 => .{ .inc = .b },
        0x05 => .{ .dec = .b },
        0x06 => .{ .ld = .{ .byte = .{ .target = .b, .source = .{ .d8 = bytes[1] } } } },
        0x0b => .{ .dec = .bc },
        0x0c => .{ .inc = .c },
        0x0d => .{ .dec = .c },
        0x0e => .{ .ld = .{ .byte = .{ .target = .c, .source = .{ .d8 = bytes[1] } } } },
        0x11 => .{ .ld = .{ .word = .{ .target = .de, .source = @as(u16, bytes[1]) + (@as(u16, bytes[2]) << 8) } } },
        0x13 => .{ .inc = .de },
        0x15 => .{ .dec = .d },
        0x16 => .{ .ld = .{ .byte = .{ .target = .d, .source = .{ .d8 = bytes[1] } } } },
        0x17 => .rla,
        0x18 => .{ .jr = .{ .condition = null, .rel = @bitCast(bytes[1]) } },
        0x19 => .{ .add = .{ .hl = .de } },
        0x1a => .{ .ld = .{ .a_from_indirect = .dei } },
        0x1d => .{ .dec = .e },
        0x1e => .{ .ld = .{ .byte = .{ .target = .e, .source = .{ .d8 = bytes[1] } } } },
        0x20 => .{ .jr = .{ .condition = .nz, .rel = @bitCast(bytes[1]) } },
        0x21 => .{ .ld = .{ .word = .{ .target = .hl, .source = @as(u16, bytes[1]) + (@as(u16, bytes[2]) << 8) } } },
        0x22 => .{ .ld = .{ .indirect_from_a = .hli_inc } },
        0x23 => .{ .inc = .hl },
        0x24 => .{ .inc = .h },
        0x28 => .{ .jr = .{ .condition = .z, .rel = @bitCast(bytes[1]) } },
        0x2a => .{ .ld = .{ .a_from_indirect = .hli_inc } },
        0x2e => .{ .ld = .{ .byte = .{ .target = .l, .source = .{ .d8 = bytes[1] } } } },
        0x31 => .{ .ld = .{ .word = .{ .target = .sp, .source = @as(u16, bytes[1]) + (@as(u16, bytes[2]) << 8) } } },
        0x32 => .{ .ld = .{ .indirect_from_a = .hli_dec } },
        0x33 => .{ .inc = .sp },
        0x34 => .{ .inc = .hli },
        0x36 => .{ .ld = .{ .byte = .{ .target = .hli, .source = .{ .d8 = bytes[1] } } } },
        0x3a => .{ .ld = .{ .a_from_indirect = .hli_dec } },
        0x3c => .{ .inc = .a },
        0x3d => .{ .dec = .a },
        0x3e => .{ .ld = .{ .byte = .{ .target = .a, .source = .{ .d8 = bytes[1] } } } },
        0x4f => .{ .ld = .{ .byte = .{ .target = .c, .source = .a } } },
        0x57 => .{ .ld = .{ .byte = .{ .target = .d, .source = .a } } },
        0x5f => .{ .ld = .{ .byte = .{ .target = .e, .source = .a } } },
        0x66 => .{ .ld = .{ .byte = .{ .target = .h, .source = .hli } } },
        0x67 => .{ .ld = .{ .byte = .{ .target = .h, .source = .a } } },
        0x6f => .{ .ld = .{ .byte = .{ .target = .l, .source = .a } } },
        0x72 => .{ .ld = .{ .byte = .{ .target = .hli, .source = .d } } },
        0x73 => .{ .ld = .{ .byte = .{ .target = .hli, .source = .e } } },
        0x76 => .halt,
        0x77 => .{ .ld = .{ .byte = .{ .target = .hli, .source = .a } } },
        0x78 => .{ .ld = .{ .byte = .{ .target = .a, .source = .b } } },
        0x79 => .{ .ld = .{ .byte = .{ .target = .a, .source = .c } } },
        0x7a => .{ .ld = .{ .byte = .{ .target = .a, .source = .d } } },
        0x7b => .{ .ld = .{ .byte = .{ .target = .a, .source = .e } } },
        0x7c => .{ .ld = .{ .byte = .{ .target = .a, .source = .h } } },
        0x7d => .{ .ld = .{ .byte = .{ .target = .a, .source = .l } } },
        0x86 => .{ .add = .{ .byte = .hli } },
        0x87 => .{ .add = .{ .byte = .a } },
        0x90 => .{ .sub = .b },
        0xa7 => .{ .@"and" = .a },
        0xaf => .{ .xor = .a },
        0xb1 => .{ .@"or" = .c },
        0xbe => .{ .cp = .hli },
        0xc0 => .{ .ret = .nz },
        0xc1 => .{ .pop = .bc },
        0xc2 => .{ .jp = .{ .condition = .nz, .source = .{ .a16 = @as(u16, bytes[1]) + (@as(u16, bytes[2]) << 8) } } },
        0xc3 => .{ .jp = .{ .condition = null, .source = .{ .a16 = @as(u16, bytes[1]) + (@as(u16, bytes[2]) << 8) } } },
        0xc5 => .{ .push = .bc },
        0xc8 => .{ .ret = .z },
        0xc9 => .{ .ret = null },
        0xcb => instruction: {
            const reg: Instruction.PrefixReg = switch (@as(u3, @truncate(bytes[1]))) {
                0 => .b,
                1 => .c,
                2 => .d,
                3 => .e,
                4 => .h,
                5 => .l,
                6 => .hli,
                7 => .a,
            };
            break :instruction switch (@as(u5, @intCast(bytes[1] >> 3))) {
                0x00 >> 3 => @panic("TODO: rlc"),
                0x08 >> 3 => @panic("TODO: rrc"),
                0x10 >> 3 => .{ .rl = reg },
                0x18 >> 3 => @panic("TODO: rr"),
                0x20 >> 3 => @panic("TODO: sla"),
                0x28 >> 3 => @panic("TODO: sra"),
                0x30 >> 3 => @panic("TODO: swap"),
                0x38 >> 3 => @panic("TODO: srl"),
                0x40 >> 3 => .{ .bit = .{ .bit = 0, .reg = reg } },
                0x48 >> 3 => .{ .bit = .{ .bit = 1, .reg = reg } },
                0x50 >> 3 => .{ .bit = .{ .bit = 2, .reg = reg } },
                0x58 >> 3 => .{ .bit = .{ .bit = 3, .reg = reg } },
                0x60 >> 3 => .{ .bit = .{ .bit = 4, .reg = reg } },
                0x68 >> 3 => .{ .bit = .{ .bit = 5, .reg = reg } },
                0x70 >> 3 => .{ .bit = .{ .bit = 6, .reg = reg } },
                0x78 >> 3 => .{ .bit = .{ .bit = 7, .reg = reg } },
                0x80 >> 3...0xb8 >> 3 => @panic("TODO: res"),
                0xc0 >> 3 => .{ .set = .{ .bit = 0, .reg = reg } },
                0xc8 >> 3 => .{ .set = .{ .bit = 1, .reg = reg } },
                0xd0 >> 3 => .{ .set = .{ .bit = 2, .reg = reg } },
                0xd8 >> 3 => .{ .set = .{ .bit = 3, .reg = reg } },
                0xe0 >> 3 => .{ .set = .{ .bit = 4, .reg = reg } },
                0xe8 >> 3 => .{ .set = .{ .bit = 5, .reg = reg } },
                0xf0 >> 3 => .{ .set = .{ .bit = 6, .reg = reg } },
                0xf8 >> 3 => .{ .set = .{ .bit = 7, .reg = reg } },
            };
        },
        0xcd => .{ .call = .{ .condition = null, .a16 = @as(u16, bytes[1]) + (@as(u16, bytes[2]) << 8) } },
        0xd1 => .{ .pop = .de },
        0xd5 => .{ .push = .de },
        0xd9 => .reti,
        0xda => .{ .jp = .{ .condition = .c, .source = .{ .a16 = @as(u16, bytes[1]) + (@as(u16, bytes[2]) << 8) } } },
        0xe0 => .{ .ld = .{ .byte_address_from_a = bytes[1] } },
        0xe1 => .{ .pop = .hl },
        0xe2 => .{ .ld = .{ .indirect_from_a = .last_byte } },
        0xe5 => .{ .push = .hl },
        0xea => .{ .ld = .{ .indirect_from_a = .{ .a16 = @as(u16, bytes[1]) + (@as(u16, bytes[2]) << 8) } } },
        0xf0 => .{ .ld = .{ .a_from_byte_address = bytes[1] } },
        0xf3 => .di,
        0xf1 => .{ .pop = .af },
        0xf5 => .{ .push = .af },
        0xfa => .{ .ld = .{ .a_from_indirect = .{ .a16 = @as(u16, bytes[1]) + (@as(u16, bytes[2]) << 8) } } },
        0xfb => .ei,
        0xfe => .{ .cp = .{ .d8 = bytes[1] } },
        else => std.debug.panic("TODO: opcode 0x{x:0>2}", .{opcode}),
    };
}
