const Ppu = @This();

const std = @import("std");
const Cpu = @import("Cpu.zig");

vram: [0x2000]u8,
mode: Mode,
mode_ticks: usize,
line: u8,
lyc: u8,
scx: u8,
scy: u8,
int_select: IntSelect,
bg_palette: u8,
lcd_control: LcdControl,
lcd: [144][160]u2,

const Mode = enum(u2) {
    oam_scan = 2,
    draw = 3,
    hblank = 0,
    vblank = 1,
};

const IntSelect = packed struct(u4) {
    mode_0: bool,
    mode_1: bool,
    mode_2: bool,
    lyc: bool,
};

const LcdControl = packed struct(u8) {
    bg_win_enable: bool,
    obj_enable: bool,
    obj_size: bool,
    bg_tile_map_area: TileMapArea,
    bg_win_tile_data_area: AddressingMode,
    win_enable: bool,
    win_tile_map_area: TileMapArea,
    lcd_enable: bool,

    const TileMapArea = enum(u1) {
        lower = 0,
        higher = 1,
    };

    const AddressingMode = enum(u1) {
        higher = 0,
        lower = 1,
    };
};

pub const Register = enum {
    scy,
    scx,
    ly,
    lyc,
    bgp,
    lcdc,
    stat,
};

pub fn read(ppu: Ppu, register: Register) u8 {
    return switch (register) {
        .scy => ppu.scy,
        .scx => ppu.scx,
        .ly => ppu.line,
        .lyc => ppu.lyc,
        .bgp => ppu.bg_palette,
        .lcdc => @bitCast(ppu.lcd_control),
        .stat => @as(u8, (@as(u4, @bitCast(ppu.int_select)) << 3)) | (@as(u8, @intFromBool(ppu.lyc == ppu.line)) << 2) | @intFromEnum(ppu.mode),
    };
}

pub fn write(ppu: *Ppu, register: Register, value: u8) void {
    switch (register) {
        .scy => ppu.scy = value,
        .scx => ppu.scx = value,
        .ly => @panic("TODO"),
        .lyc => ppu.lyc = value,
        .bgp => ppu.bg_palette = value,
        .lcdc => ppu.lcd_control = @bitCast(value),
        .stat => ppu.int_select = @bitCast(@as(u4, @truncate(value >> 3))),
    }
}

pub fn tick(ppu: *Ppu) ?Cpu.InterruptType {
    var int: ?Cpu.InterruptType = null;

    if (ppu.lcd_control.lcd_enable) {
        ppu.mode_ticks += 1;
        switch (ppu.mode) {
            .oam_scan => {
                if (ppu.mode_ticks == 80) {
                    ppu.mode = .draw;
                    ppu.mode_ticks = 0;
                }
            },
            .draw => {
                // TODO: Variable time
                if (ppu.mode_ticks == 172) {
                    ppu.renderLine(ppu.line);
                    ppu.mode = .hblank;
                    ppu.mode_ticks = 0;
                }
            },
            .hblank => {
                if (ppu.mode_ticks == 204) {
                    ppu.line += 1;

                    if (ppu.line == 144) {
                        ppu.mode = .vblank;
                        int = .vblank;
                    } else {
                        ppu.mode = .oam_scan;
                    }

                    ppu.mode_ticks = 0;
                }
            },
            .vblank => {
                if (ppu.mode_ticks == 456) {
                    ppu.line += 1;

                    if (ppu.line > 153) {
                        ppu.line = 0;
                        ppu.mode = .oam_scan;
                        ppu.mode_ticks = 0;
                    } else {
                        ppu.mode_ticks = 0;
                    }
                }
            },
        }
    }

    return int;
}

fn renderLine(ppu: *Ppu, line: u8) void {
    const tile_y = (line +% ppu.scy) / 8;
    const tile_y_offset = (line +% ppu.scy) % 8;
    for (0..160) |x| {
        const tile_x = (x +% ppu.scx) / 8;
        const tile_x_offset = (x +% ppu.scx) % 8;

        const tile_offset = @as(usize, tile_y) * 32 + @as(usize, tile_x);

        const tile_idx = ppu.vram[
            switch (ppu.lcd_control.bg_tile_map_area) {
                .lower => @as(u16, 0x9800),
                .higher => @as(u16, 0x9c00),
            } - 0x8000 + tile_offset
        ];

        const tile = ppu.vram[switch (ppu.lcd_control.bg_win_tile_data_area) {
            .lower => @as(usize, tile_idx) * 16,
            .higher => @as(usize, tile_idx) * 16 + if (tile_idx < 128) @as(usize, 0x1000) else 0,
        }..][0..16];

        const color_idx: u2 = @intCast(((tile[2 * tile_y_offset] >> @intCast(7 - tile_x_offset)) & 1) |
            ((tile[2 * tile_y_offset + 1] >> @intCast(7 - tile_x_offset)) & 1) << 1);

        ppu.lcd[line][x] = @intCast((ppu.bg_palette >> 2 * @as(u3, color_idx)) & 3);
    }
}
