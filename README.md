# Minimal 2D pose graph optimizer using Levenberg-Marquardt algorithm

Jointly optimizes robot poses and landmark positions from noisy odometry and observations. Includes a step-by-step visualizer to inspect each solver iteration.

## Dependencies

- [Eigen](https://eigen.tuxfamily.org/)
- [raylib](https://www.raylib.com/)

## Build

```bash
cmake -B build
cmake --build build --config Release
```

## Usage

```bash
./pgo2d [options]
```

| Option                   | Description                  | Default |
|--------------------------|------------------------------|---------|
| `--poses N`              | Number of poses              | 30      |
| `--landmarks N`          | Number of landmarks          | 70      |
| `--noise-translation F`  | Odometry noise (translation) | 0.15    |
| `--noise-rotation F`     | Odometry noise (rotation)    | 0.07    |
| `--noise-observation F`  | Observation noise            | 0.12    |
| `--observation-radius F` | Observation radius           | 4.5     |

### Controls

| Key           | Action                    |
|---------------|---------------------------|
| Left / Right  | Step through iterations   |
| Home / End    | Jump to first / last      |

## License

[MIT](LICENSE)
