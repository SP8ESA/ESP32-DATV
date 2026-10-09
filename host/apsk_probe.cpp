// Copies the actual constellation pixels passed to SDRangel's TVScreen.
// Demodulation, samples, gain and drawing are forwarded unchanged.
// Build: g++ -std=c++17 -O2 -shared -fPIC host/apsk_probe.cpp -ldl -o probe.so
// Run with LD_PRELOAD=probe.so and DATV_APSK_CAPTURE=/absolute/path/points.s16.
// Output: little-endian int16 row,column pairs; .refs contains white crosses.
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <string>
#include <unistd.h>

using Row = void (*)(void *, int);
using Color = void (*)(void *, int, int, int, int);

struct Output {
    int data = -1, refs = -1;
    Output() {
        const char *path = std::getenv("DATV_APSK_CAPTURE");
        if (!path || !*path) return;
        data = open(path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0600);
        const std::string refpath = std::string(path) + ".refs";
        refs = open(refpath.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0600);
        std::fprintf(stderr, "[APSK probe] constellation capture: %s (data=%d refs=%d)\n", path, data, refs);
    }
};

static Output &output() { static Output o; return o; }
static thread_local void *row_screen = nullptr;
static thread_local int selected_row = 0;

struct Buffer {
    int16_t pairs[1024][2];
    size_t used = 0;
    void append(int fd, int row, int col) {
        if (fd < 0 || row < 0 || row > 255 || col < 0 || col > 255) return;
        pairs[used][0] = int16_t(row); pairs[used][1] = int16_t(col);
        if (++used != 1024) return;
        const char *p = reinterpret_cast<const char *>(pairs);
        size_t left = sizeof pairs;
        while (left) {
            const ssize_t n = write(fd, p, left);
            if (n < 0 && errno == EINTR) continue;
            if (n <= 0) break;
            p += n; left -= size_t(n);
        }
        used = 0;
    }
};

extern "C" void probe_row(void *, int) asm("_ZN8TVScreen9selectRowEi");
extern "C" void probe_row(void *screen, int row) {
    static const auto original = reinterpret_cast<Row>(dlsym(RTLD_NEXT, "_ZN8TVScreen9selectRowEi"));
    if (!original) { std::fprintf(stderr, "[APSK probe] missing selectRow\n"); std::abort(); }
    original(screen, row);
    row_screen = screen; selected_row = row;
}

extern "C" void probe_color(void *, int, int, int, int) asm("_ZN8TVScreen12setDataColorEiiii");
extern "C" void probe_color(void *screen, int col, int r, int g, int b) {
    static const auto original = reinterpret_cast<Color>(dlsym(RTLD_NEXT, "_ZN8TVScreen12setDataColorEiiii"));
    if (!original) { std::fprintf(stderr, "[APSK probe] missing setDataColor\n"); std::abort(); }
    original(screen, col, r, g, b);
    if (row_screen != screen) return;
    // Only the DATV constellation uses these colors and this drawing path.
    static thread_local Buffer samples, refs;
    if (r == 255 && g == 0 && b == 255) samples.append(output().data, selected_row, col);
    else if (r == 250 && g == 250 && b == 250) refs.append(output().refs, selected_row, col);
}
