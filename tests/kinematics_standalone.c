/* Compile using only the generic module's own include directory and standard C. */
#include "swerve_kinematics.h"
#include <math.h>
#include <stdint.h>

static void report(uint32_t operation, const void *argument)
{
    register uint32_t op __asm__("r0") = operation;
    register const void *arg __asm__("r1") = argument;
    __asm__ volatile("bkpt 0xab" : "+r"(op) : "r"(arg) : "memory");
}
static void finish(uint32_t code)
{
    uint32_t args[2] = {0x20026, code};
    report(0x20, args);
    for (;;) {}
}
#define CHECK(x) do { if (!(x)) { report(4, "FAIL: standalone geometry\n"); finish(__LINE__); } } while (0)
#define NEAR(a,b) CHECK(fabsf((a) - (b)) < 0.001f)

void test_main(void)
{
    const float radians = 3.14159265358979323846f / 180.0f;
    SwerveGeometry_s first = {0.216f, 0.38f, 0.0526f};
    SwerveGeometry_s second = {0.432f, 0.76f, 0.0263f};
    float actual[4] = {720, -720, 360, -360};
    SwerveTarget a[4] = {0}, b[4] = {0};
    SwerveSolve(&first, 1, 0, 0, actual, a);
    SwerveSolve(&second, 1, 0, 0, actual, b);
    /* Equal circumference * ratio: equal translational rotor speeds. */
    for (unsigned i = 0; i < 4; ++i) NEAR(a[i].speed_rpm, b[i].speed_rpm);
    SwerveSolve(&first, 0, 0, 90, actual, a);
    SwerveSolve(&second, 0, 0, 90, actual, b);
    /* Double radius with equal transmission scale: double each wheel vector. */
    for (unsigned i = 0; i < 4; ++i)
    {
        NEAR(2 * a[i].speed_rpm * cosf(a[i].angle_deg * radians), b[i].speed_rpm * cosf(b[i].angle_deg * radians));
        NEAR(2 * a[i].speed_rpm * sinf(a[i].angle_deg * radians), b[i].speed_rpm * sinf(b[i].angle_deg * radians));
    }
    /* Interleaving a different chassis must not alter the first configuration. */
    SwerveSolve(&first, 1, 0, 0, actual, a);
    NEAR(a[0].speed_rpm, 60.0f / (first.wheel_perimeter_m * first.drive_reduction));
    report(4, "PASS: standalone solver, independent chassis geometry and multi-turn vectors\n");
    finish(0);
}
