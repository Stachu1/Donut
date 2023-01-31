const std = @import("std");
const builtin = @import("builtin");
const os = std.os;
const Vector = std.meta.Vector;
const print = std.debug.print;
const stdout_file = std.io.getStdOut().writer();
var bw = std.io.bufferedWriter(stdout_file);
const stdout = bw.writer();

const PI: f32 = 3.14159265358979323846;

const C1_RES: u16 = 32;
const C2_RES: u16 = 64;
const R1: f32 = 2;
const R2: f32 = 6;

const SCREEN_SIZE: u16 = 20;
const SCREEN_RES: u16 = 25;
const SCREEN_POS: f32 = -32;

const ASCII = ".,-~:;=!*%#$";

const AX_STEP: f32 = 0.002;
const AY_STEP: f32 = 0.001;
const AZ_STEP: f32 = 0.0025;



const Donut = struct {
    c1_res: u16 = C1_RES,
    c2_res: u16 = C2_RES,
    points: [C1_RES * C2_RES] @Vector(3, f32) = undefined,
    POINTS: [C1_RES * C2_RES] @Vector(3, f32) = undefined,

    pub fn init(self: *Donut) void {
        const c1_step: f32 = 2 * PI / @intToFloat(f32, self.c1_res);
        const c2_step: f32 = 2 * PI / @intToFloat(f32, self.c2_res);
        
        var c1_index: u16 = 0;
        var c2_index: u16 = 0;
        while (@intToFloat(f32, c2_index) * c2_step < 2 * PI): (c2_index += 1) {
            while (@intToFloat(f32, c1_index) * c1_step < 2 * PI): (c1_index += 1) {
                var x: f32 = (R1 * @cos(@intToFloat(f32, c1_index) * c1_step) + R2) * @cos(@intToFloat(f32, c2_index) * c2_step);
                var y: f32 = R1 * @sin(@intToFloat(f32, c1_index) * c1_step);
                var z: f32 = (R1 * @cos(@intToFloat(f32, c1_index) * c1_step) + R2) * @sin(@intToFloat(f32, c2_index) * c2_step);
                self.points[c1_index + c2_index * C1_RES] = @Vector(3, f32) {x, y, z};
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

        var aX_x: f32 = cosa*cosb;
        var aX_y: f32 = cosa*sinb*sinc - sina*cosc;
        var aX_z: f32 = cosa*sinb*cosc + sina*sinc;

        var aY_x: f32 = sina*cosb;
        var aY_y: f32 = sina*sinb*sinc + cosa*cosc;
        var aY_z: f32 = sina*sinb*cosc - cosa*sinc;

        var aZ_x: f32 = -sinb;
        var aZ_y: f32 = cosb*sinc;
        var aZ_z: f32 = cosb*cosc;
        for (self.POINTS) |point, index| {
            var x: f32 = point[0];
            var y: f32 = point[1];
            var z: f32 = point[2];

            var new_x: f32 = aX_x * x + aX_y * y + aX_z * z;
            var new_y: f32 = aY_x * x + aY_y * y + aY_z * z;
            var new_z: f32 = aZ_x * x + aZ_y * y + aZ_z * z;
            
            self.points[index] = @Vector(3, f32) {new_x, new_y, new_z};
        }
    }
};


const Camera = struct {
    screen: [SCREEN_RES][SCREEN_RES] f32 = std.mem.zeroes([SCREEN_RES][SCREEN_RES]f32),

    pub fn capture(self: *Camera, donut: *Donut) void {
        var screen_distance_buffer: [SCREEN_RES][SCREEN_RES]f32 = std.mem.zeroes([SCREEN_RES][SCREEN_RES]f32);
        self.screen = std.mem.zeroes([SCREEN_RES][SCREEN_RES]f32);

        for (donut.*.points) |point| {
            var dX: f32 = point[0];
            var dY: f32 = point[1];
            var dZ: f32 = abs(point[2] - SCREEN_POS);
            var distance: f32 = @sqrt(dX * dX + dY * dY + dZ * dZ);

            var sfX: f32 = dX + @intToFloat(f32, SCREEN_SIZE) / 2;
            var sfY: f32 = dY + @intToFloat(f32, SCREEN_SIZE) / 2;

            var sX: u16 = @floatToInt(u16, sfX * @intToFloat(f32, SCREEN_RES) / @intToFloat(f32, SCREEN_SIZE));
            var sY: u16 = @floatToInt(u16, sfY * @intToFloat(f32, SCREEN_RES) / @intToFloat(f32, SCREEN_SIZE));
            if (sX >= 0 and sX <= SCREEN_RES - 1 and sY >= 0 and sY <= SCREEN_RES - 1) {
                if (screen_distance_buffer[sX][sY] == 0 or screen_distance_buffer[sX][sY] > distance) {
                    screen_distance_buffer[sX][sY] = distance;
                    var ray: Vector(3, f32) = @Vector(3, f32) {0, 0, -1};
                    var normal: Vector(3, f32) = @Vector(3, f32) {point[0], point[1], point[2]};
                    self.screen[sY][sX] = get_brightness(&ray, &normal);
                }
            }
        }
    }

    pub fn get_brightness(vec1: *Vector(3, f32), vec2: *Vector(3, f32)) f32 {
        var dot: f32 = vec1.*[0] * vec2.*[0] + vec1.*[1] * vec2.*[1] + vec1.*[2] * vec2.*[2];
        var vec1_length: f32 = @sqrt(vec1.*[0] * vec1.*[0] + vec1.*[1] * vec1.*[1] + vec1.*[2] * vec1.*[2]);
        var vec2_length: f32 = @sqrt(vec2.*[0] * vec2.*[0] + vec2.*[1] * vec2.*[1] + vec2.*[2] * vec2.*[2]);
        var brightness: f32 = (dot / (vec1_length * vec2_length) + 1) / 2;
        return brightness;
        // if (brightness > 0) {
        //     return brightness;
        // }
        // else {
        //     return 0.09;
        // }
    }

    pub fn display(self: *Camera, ascii: *const [12:0]u8) !void {
        
        for (self.screen) |row| {
            for (row) |pixel_value| {
                var char_index: u8 = @floatToInt(u8, @round(pixel_value * @as(f32, 12)));
                if (char_index == 0) {
                    try stdout.print("  ", .{});
                }
                else {
                    var char = ascii.*[char_index - 1];
                    try stdout.print("{c}{c}", .{char, char});
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
    }
    else {
        return value;
    }
}


pub fn timestamp() u32 {
    var ts: os.timespec = undefined;
    os.clock_gettime(os.CLOCK.REALTIME, &ts) catch |err| switch (err) {error.UnsupportedClock, error.Unexpected => return 0};
    return @intCast(u32, ts.tv_nsec) / 1000_000;
}


pub fn main() !void {
    var ax: f32 = 0;
    var ay: f32 = 0;
    var az: f32 = 0;

    var rotation_factor: f32 = 1;
    var last_time: u32 = timestamp();

    var donut = Donut {};
    var camera = Camera {};
    
    donut.init();

    while (ax < 2 * PI) {
        donut.rotate(ax, ay, az);
        camera.capture(&donut);
        try camera.display(ASCII);

        ax += AX_STEP * rotation_factor;
        ay += AY_STEP * rotation_factor;
        az += AZ_STEP * rotation_factor;

        if (ax > 2 * PI) {
            ax -= 2 * PI;
        }
        if (ay > 2 * PI) {
            ay -= 2 * PI;
        }
        if (az > 2 * PI) {
            az -= 2 * PI;
        }

        var time: u32 = timestamp();
        if (time > last_time) {
            rotation_factor = @intToFloat(f32, time - last_time);
            print("FRAME: {}ms\n", .{time - last_time});
        }
        

        last_time = time;
    }
}