#include "vis.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

static void print_usage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "  --poses N                Number of poses                (default: 30)\n"
        "  --landmarks N            Number of landmarks            (default: 70)\n"
        "  --noise-translation F    Odometry noise (translation)   (default: 0.15)\n"
        "  --noise-rotation F       Odometry noise (rotation)      (default: 0.07)\n"
        "  --noise-observation F    Observation noise              (default: 0.12)\n"
        "  --observation-radius F   Observation radius             (default: 4.5)\n"
        "  --help                   Show this message\n",
        prog);
}

int main(int argc, char* argv[]) {
    App app;

    for (int i = 1; i < argc; ++i) {
        auto arg = [&](const char* name) { return std::strcmp(argv[i], name) == 0; };
        auto next_int   = [&]() { return std::atoi(argv[++i]); };
        auto next_float = [&]() { return (float)std::atof(argv[++i]); };

        if (arg("--help") || arg("-h")) { print_usage(argv[0]); return 0; }

        if (arg("--poses")              && i + 1 < argc) { app.num_poses          = next_int();   continue; }
        if (arg("--landmarks")          && i + 1 < argc) { app.num_landmarks      = next_int();   continue; }
        if (arg("--noise-translation")  && i + 1 < argc) { app.noise_translation  = next_float(); continue; }
        if (arg("--noise-rotation")     && i + 1 < argc) { app.noise_rotation     = next_float(); continue; }
        if (arg("--noise-observation")  && i + 1 < argc) { app.noise_observation  = next_float(); continue; }
        if (arg("--observation-radius") && i + 1 < argc) { app.observation_radius = next_float(); continue; }

        std::fprintf(stderr, "Unknown option: %s\n", argv[i]);
        print_usage(argv[0]);
        return 1;
    }

    app.init();
    app.run();
    return 0;
}
