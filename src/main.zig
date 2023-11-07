const std = @import("std");
const builtin = @import("builtin");
const os = std.os;
const Vector = std.meta;
const print = std.debug.print;
const stdout_file = std.io.getStdOut().writer();
var bw = std.io.bufferedWriter(stdout_file);
const stdout = bw.writer();

const TARGET_FRAME_TIME = 33;

const TAU: f32 = 6.28318530717958647692;

const C1_RES: u16 = 32;
const C2_RES: u16 = 64;
const R1: f32 = 2;
const R2: f32 = 6;

const SCREEN_SIZE: u16 = 20;
const SCREEN_RES: u16 = 25;
const SCREEN_POS: f32 = -32;

const MAX_DISTANCE = R1 + R2 - SCREEN_POS;
const MIN_DISTANCE = -R1 - R2 - SCREEN_POS;

const ASCII = ".,-~:;=!*%#$";

const AX_STEP: f32 = 0.016;
const AY_STEP: f32 = 0.006;
const AZ_STEP: f32 = 0.015;

const Donut = struct {
    c1_res: u16 = C1_RES,
    c2_res: u16 = C2_RES,
    points: [C1_RES * C2_RES]@Vector(3, f32) = undefined,
    POINTS: [C1_RES * C2_RES]@Vector(3, f32) = undefined,

    pub fn init(self: *Donut) void {
        const c1_step: f32 = TAU / @as(f32, C1_RES);
        const c2_step: f32 = TAU / @as(f32, C2_RES);

        var c1_index: u16 = 0;
        var c2_index: u16 = 0;
        while (@as(f32, @floatFromInt(c2_index)) * c2_step < TAU) : (c2_index += 1) {
            while (@as(f32, @floatFromInt(c1_index)) * c1_step < TAU) : (c1_index += 1) {
                var x: f32 = (R1 * @cos(@as(f32, @floatFromInt(c1_index)) * c1_step) + R2) * @cos(@as(f32, @floatFromInt(c2_index)) * c2_step);
                var y: f32 = R1 * @sin(@as(f32, @floatFromInt(c1_index)) * c1_step);
                var z: f32 = (R1 * @cos(@as(f32, @floatFromInt(c1_index)) * c1_step) + R2) * @sin(@as(f32, @floatFromInt(c2_index)) * c2_step);
                self.points[c1_index + c2_index * C1_RES] = @Vector(3, f32){ x, y, z };
            }
            c1_index = 0;
        }
        self.POINTS = self.points;
    }

    pub fn rotate(self: *Donut, aX: f32, aY: f32, aZ: f32) void {
        var cosa: f32 = @cos(aZ);
        var sina: f32 = @sin(aZ);

        var cosb: f32 = @cos(aY);
        var sinb: f32 = @sin(aY);

        var cosc: f32 = @cos(aX);
        var sinc: f32 = @sin(aX);

        var aX_x: f32 = cosa * cosb;
        var aX_y: f32 = cosa * sinb * sinc - sina * cosc;
        var aX_z: f32 = cosa * sinb * cosc + sina * sinc;

        var aY_x: f32 = sina * cosb;
        var aY_y: f32 = sina * sinb * sinc + cosa * cosc;
        var aY_z: f32 = sina * sinb * cosc - cosa * sinc;

        var aZ_x: f32 = -sinb;
        var aZ_y: f32 = cosb * sinc;
        var aZ_z: f32 = cosb * cosc;
        var index: u32 = 0;
        for (self.POINTS) |point| {
            var x: f32 = point[0];
            var y: f32 = point[1];
            var z: f32 = point[2];

            var new_x: f32 = aX_x * x + aX_y * y + aX_z * z;
            var new_y: f32 = aY_x * x + aY_y * y + aY_z * z;
            var new_z: f32 = aZ_x * x + aZ_y * y + aZ_z * z;

            self.points[index] = @Vector(3, f32){ new_x, new_y, new_z };
            index += 1;
        }
    }
};

const Camera = struct {
    screen: [SCREEN_RES][SCREEN_RES]f32 = std.mem.zeroes([SCREEN_RES][SCREEN_RES]f32),

    pub fn capture(self: *Camera, donut: *Donut) void {
        var screen_distance_buffer: [SCREEN_RES][SCREEN_RES]f32 = std.mem.zeroes([SCREEN_RES][SCREEN_RES]f32);
        self.screen = std.mem.zeroes([SCREEN_RES][SCREEN_RES]f32);

        for (donut.*.points) |point| {
            var dX: f32 = point[0];
            var dY: f32 = point[1];
            var dZ: f32 = abs(point[2] - SCREEN_POS);
            var distance: f32 = @sqrt(dX * dX + dY * dY + dZ * dZ);

            var sfX: f32 = dX + @as(f32, SCREEN_SIZE) / 2;
            var sfY: f32 = dY + @as(f32, SCREEN_SIZE) / 2;

            var sX: u16 = @intFromFloat(sfX * @as(f32, SCREEN_RES) / @as(f32, SCREEN_SIZE));
            var sY: u16 = @intFromFloat(sfY * @as(f32, SCREEN_RES) / @as(f32, SCREEN_SIZE));
            if (sX >= 0 and sX <= SCREEN_RES - 1 and sY >= 0 and sY <= SCREEN_RES - 1) {
                if (screen_distance_buffer[sX][sY] == 0 or screen_distance_buffer[sX][sY] > distance) {
                    screen_distance_buffer[sX][sY] = distance;
                    // var ray: @Vector(3, f32) = .{ 0, 0, -1 };
                    // var nie_normal: @Vector(3, f32) = .{ point[0], point[1], point[2] };
                    // self.screen[sY][sX] = get_brightness(&ray, &nie_normal);
                    self.screen[sY][sX] = (MIN_DISTANCE - distance) / (MAX_DISTANCE - MIN_DISTANCE) + 1;
                }
            }
        }
    }

    fn get_brightness(vec1: *@Vector(3, f32), vec2: *@Vector(3, f32)) f32 {
        var dot: f32 = vec1.*[0] * vec2.*[0] + vec1.*[1] * vec2.*[1] + vec1.*[2] * vec2.*[2];
        var vec1_length: f32 = @sqrt(vec1.*[0] * vec1.*[0] + vec1.*[1] * vec1.*[1] + vec1.*[2] * vec1.*[2]);
        var vec2_length: f32 = @sqrt(vec2.*[0] * vec2.*[0] + vec2.*[1] * vec2.*[1] + vec2.*[2] * vec2.*[2]);
        var brightness: f32 = (dot / (vec1_length * vec2_length) + 1) / 2.1;
        return brightness;

        // if (brightness > 0) {
        //     return brightness;
        // }
        // else {
        //     return 0.09;
        // }
    }

    pub fn color_map(p: f32) u8 {
        const r = p;
        const g = 0;
        const b = 1 - p;
        const index: u8 = 16 + (36 * @as(u8, @intFromFloat(r * 5))) + (6 * @as(u8, @intFromFloat(g * 5))) + @as(u8, @intFromFloat(b * 5));
        return index;
    }

    pub fn display(self: *Camera, ascii: *const [12:0]u8) !void {
        try stdout.print("\x1b[2J", .{});
        for (self.screen) |row| {
            for (row) |pixel_value| {
                var char_index: u8 = @intFromFloat(@round(pixel_value * @as(f32, 12)));
                if (char_index == 0) {
                    try stdout.print("  ", .{});
                } else {
                    var char = ascii.*[char_index - 1];
                    try stdout.print("\x1b[38;5;{}m{c}{c}\x1b[0m", .{ color_map(pixel_value), char, char });
                }
            }
            try stdout.print("\n", .{});
        }
        try bw.flush();
    }
};

pub fn abs(value: f32) f32 {
    if (value < 0) {
        return value * -1;
    } else {
        return value;
    }
}

pub fn timestamp() u32 {
    var ts: os.timespec = undefined;
    os.clock_gettime(os.CLOCK.REALTIME, &ts) catch |err| switch (err) {
        error.UnsupportedClock, error.Unexpected => return 0,
    };
    const nanos = ts.tv_nsec;
    const milis = @divTrunc(nanos, 1000_000);
    var t: u32 = @intCast(milis);
    return t;
}

pub fn main() !void {
    var ax: f32 = 0;
    var ay: f32 = 0;
    var az: f32 = 0;

    var rotation_factor: f32 = 1;

    var donut = Donut{};
    var camera = Camera{};

    donut.init();

    const TFT_M1 = TARGET_FRAME_TIME - 1;

    while (true) {
        donut.rotate(ax, ay, az);
        camera.capture(&donut);
        try camera.display(ASCII);

        ax += AX_STEP * rotation_factor;
        ay += AY_STEP * rotation_factor;
        az += AZ_STEP * rotation_factor;

        if (ax > TAU) {
            ax -= TAU;
        }
        if (ay > TAU) {
            ay -= TAU;
        }
        if (az > TAU) {
            az -= TAU;
        }

        while (timestamp() == 0) {}
        while (timestamp() % TFT_M1 != 0) {}
        while (timestamp() % TARGET_FRAME_TIME != 0) {}
    }
}
