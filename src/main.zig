const std = @import("std");
const sdl = @import("sdl");
const Cpu = @import("Cpu.zig");

const default_boot_rom = @embedFile("default_boot_rom");

const scale = 4;

const palette = [4]sdl.pixels.Color{
    .{ .r = 155, .g = 188, .b = 15 },
    .{ .r = 139, .g = 172, .b = 15 },
    .{ .r = 48, .g = 98, .b = 48 },
    .{ .r = 15, .g = 56, .b = 15 },
};

pub fn main() !void {
    var cpu = Cpu.init(default_boot_rom.*, @embedFile("snake.gb").*);

    defer sdl.init.shutdown();

    const init_flags: sdl.init.Flags = .{ .video = true };
    try sdl.init.init(init_flags);
    defer sdl.init.quit(init_flags);

    const main_window = try sdl.video.Window.init("Gameboy emulator", 160 * scale, 144 * scale, .{});
    defer main_window.deinit();

    const tileset_window = try sdl.video.Window.init("Gameboy emulator tileset", 16 * 8 * scale, 24 * 8 * scale, .{});
    defer tileset_window.deinit();

    const main_renderer = try sdl.render.Renderer.init(main_window, null);
    defer main_renderer.deinit();

    const tileset_renderer = try sdl.render.Renderer.init(tileset_window, null);
    defer tileset_renderer.deinit();

    var start = std.time.microTimestamp();
    loop: while (true) {
        while (sdl.events.poll()) |event| {
            switch (event) {
                .quit, .terminating => break :loop,
                .unknown => |e| {
                    switch (e.event_type) {
                        sdl.c.SDL_EVENT_WINDOW_CLOSE_REQUESTED => break :loop,
                        else => {},
                    }
                },
                else => {},
            }
        }

        var cycles: usize = 0;
        while (cycles < 70224) {
            cycles += cpu.tick();
        }

        const elapsed: u64 = @intCast(std.time.microTimestamp() - start);
        if (elapsed < @divFloor(std.time.us_per_s, 59.73)) {
            std.time.sleep((@as(u64, @intFromFloat(@divFloor(std.time.us_per_s, 59.73))) - elapsed) * std.time.ns_per_us);
        } else {
            std.log.warn("Frame lagged behind", .{});
        }
        start = std.time.microTimestamp();

        {
            for (0..144) |y| {
                for (0..160) |x| {
                    try main_renderer.setDrawColor(palette[cpu.ppu.lcd[y][x]]);
                    try main_renderer.renderFillRect(.{ .x = @floatFromInt(x * scale), .y = @floatFromInt(y * scale), .w = scale, .h = scale });
                }
            }

            try main_renderer.present();
        }

        {
            try tileset_renderer.setDrawColor(.{ .r = 0, .g = 0, .b = 0 });
            try tileset_renderer.clear();

            for (0..0x180) |tile_idx| {
                const x = tile_idx % 16;
                const y = tile_idx / 16;

                const tile = cpu.ppu.vram[tile_idx * 16 ..][0..16];

                for (0..8) |ty| {
                    for (0..8) |tx| {
                        const pixel: u2 = @intCast(((tile[2 * ty] >> @intCast(7 - tx)) & 1) | ((tile[2 * ty + 1] >> @intCast(7 - tx)) & 1) << 1);
                        try tileset_renderer.setDrawColor(palette[pixel]);
                        try tileset_renderer.renderFillRect(.{ .x = @floatFromInt((8 * x + tx) * scale), .y = @floatFromInt((8 * y + ty) * scale), .w = scale, .h = scale });
                    }
                }

                try tileset_renderer.setDrawColor(.{ .r = 100, .g = 100, .b = 100 });
                try tileset_renderer.renderRect(.{ .x = @floatFromInt(8 * x * scale), .y = @floatFromInt(8 * y * scale), .w = 8 * scale + 1, .h = 8 * scale + 1 });
            }

            try tileset_renderer.present();
        }
    }
}

pub fn disassemble(bin: []const u8) !void {
    const stdout = std.io.getStdOut().writer();
    var offset: u16 = 0;

    while (offset < bin.len) {
        const opcode = bin[offset];

        const size = Cpu.instructionSize(opcode);
        defer offset += size;

        const instruction = switch (size) {
            0 => @panic("TODO"),
            1 => Cpu.decode(&.{opcode}),
            2 => Cpu.decode(&.{ opcode, bin[offset + 1] }),
            3 => Cpu.decode(&.{ opcode, bin[offset + 1], bin[offset + 2] }),
        };
        try stdout.print("0x{x:0>4}   ", .{offset});

        for (0..size) |i| {
            try stdout.print(" 0x{x:0>2}", .{bin[offset + i]});
        }

        for (0..3 - @as(usize, size)) |_| {
            try stdout.print("     ", .{});
        }

        try stdout.print("   {}\n", .{instruction});
    }
}
