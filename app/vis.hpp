#pragma once

#include <vector>

struct Pose2D {
    double x = 0, y = 0, theta = 0;
};

struct Point2D {
    double x = 0, y = 0;
};

struct IterInfo {
    int    iter;
    double chi2;
    double lambda;
};

struct EdgeInfo {
    int  from, to;
    bool is_loop_closure;
};

struct App {
    static constexpr int WIDTH  = 1280;
    static constexpr int HEIGHT = 900;

    // pose-graph data
    std::vector<Pose2D>   ground_truth;
    std::vector<Pose2D>   initial_poses;
    std::vector<EdgeInfo> edge_info;

    // landmark data
    std::vector<Point2D>  gt_landmarks;
    std::vector<Point2D>  initial_landmarks;
    std::vector<bool>     landmark_visible;

    // optimization snapshots per iteration
    std::vector<std::vector<Pose2D>>  opt_history;
    std::vector<std::vector<Point2D>> landmark_history;
    std::vector<IterInfo>             convergence;

    // current frame (controlled by arrow keys)
    int frame     = 0;
    int max_frame = 0;

    // settings (set from CLI before init)
    int   num_poses     = 30;
    int   num_landmarks = 70;
    float noise_translation  = 0.15f;
    float noise_rotation     = 0.07f;
    float noise_observation  = 0.12f;
    float observation_radius = 4.5f;

    void generate_and_solve();
    void init();
    void run();

private:
    void draw_scene();
    void draw_hud();
};
