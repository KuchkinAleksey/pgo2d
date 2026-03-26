#include "vis.hpp"

#include <opt/problem.hpp>
#include <opt/pose_vertex.hpp>
#include <opt/pose_edge.hpp>
#include <opt/point_vertex.hpp>
#include <opt/point_edge.hpp>

#include "raylib.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

static const Color COL_BG       = {22,  22,  28,  255};
static const Color COL_LABEL    = {130, 130, 145, 255};

static const Color COL_GT       = {100, 100, 115, 180};   // ground truth
static const Color COL_INIT     = {180, 70,  75,  160};   // initial
static const Color COL_OPT      = {60,  190, 170, 255};   // optimized
static const Color COL_EDGE     = {55,  55,  65,  100};   // odometry edges
static const Color COL_LOOP     = {220, 140, 40,  180};   // loop closure
static const Color COL_LM_GT    = {100, 100, 115, 100};   // landmark GT
static const Color COL_LM_INIT  = {180, 70,  75,  80};    // landmark initial
static const Color COL_LM_OPT   = {60,  190, 170, 200};   // landmark optimized

static Font g_font = {};

static constexpr float kSpacing = 3.0f;

static void draw_text(const char* text, float x, float y, float size, Color col) {
    DrawTextEx(g_font, text, {x, y}, size, kSpacing, col);
}

static float measure_text(const char* text, float size) {
    return MeasureTextEx(g_font, text, size, kSpacing).x;
}

/// World-to-screen transform with y-flip
struct ViewXform {
    float cx, cy, scale, scr_cx, scr_cy;

    Vector2 to_screen(double wx, double wy) const {
        return {scr_cx + (float)(wx - cx) * scale,
                scr_cy - (float)(wy - cy) * scale};
    }
};

/// Compute a view transform that fits all poses and landmarks into the given rectangle
static ViewXform fit_view(Rectangle area,
                          const std::vector<Pose2D>& a,
                          const std::vector<Pose2D>& b,
                          const std::vector<Point2D>& pts)
{
    double xlo = 1e18, xhi = -1e18, ylo = 1e18, yhi = -1e18;
    auto upd = [&](const std::vector<Pose2D>& v) {
        for (auto& p : v) {
            xlo = std::min(xlo, p.x); xhi = std::max(xhi, p.x);
            ylo = std::min(ylo, p.y); yhi = std::max(yhi, p.y);
        }
    };
    upd(a); upd(b);
    for (auto& p : pts) {
        xlo = std::min(xlo, p.x); xhi = std::max(xhi, p.x);
        ylo = std::min(ylo, p.y); yhi = std::max(yhi, p.y);
    }

    double margin = 1.5;
    xlo -= margin; xhi += margin; ylo -= margin; yhi += margin;

    float sx = area.width  / (float)(xhi - xlo);
    float sy = area.height / (float)(yhi - ylo);
    float sc = std::min(sx, sy);

    return {(float)((xlo + xhi) * 0.5), (float)((ylo + yhi) * 0.5), sc,
            area.x + area.width * 0.5f, area.y + area.height * 0.5f};
}

/// Draw a camera frustum: rear point on the trajectory, two front corners spread out
static void draw_pose_tri(Vector2 pos, float angle, float sz, Color col) {
    // rear = single point behind the pose (on the trajectory line)
    float rear_len = sz * 0.4f;
    Vector2 rear = {pos.x - rear_len * cosf(angle),
                    pos.y - rear_len * sinf(angle)};

    // two front corners spread at ~50 degrees from the heading (FOV ~100)
    float fov_half = 0.87f;  // ~50 degrees
    float front_len = sz * 0.7f;
    Vector2 left  = {pos.x + front_len * cosf(angle + fov_half),
                     pos.y + front_len * sinf(angle + fov_half)};
    Vector2 right = {pos.x + front_len * cosf(angle - fov_half),
                     pos.y + front_len * sinf(angle - fov_half)};

    DrawTriangleLines(rear, right, left, col);
}

/// Draw a connected sequence of poses as triangular frustums
static void draw_trajectory(const ViewXform& vw, const std::vector<Pose2D>& poses,
                            Color line_col, Color marker_col, float marker_sz)
{
    for (int i = 0; i + 1 < (int)poses.size(); ++i) {
        auto a = vw.to_screen(poses[i].x, poses[i].y);
        auto b = vw.to_screen(poses[i + 1].x, poses[i + 1].y);
        DrawLineEx(a, b, 1.5f, line_col);
    }
    for (auto& p : poses) {
        auto sp = vw.to_screen(p.x, p.y);
        // negate theta because screen y is flipped
        draw_pose_tri(sp, -(float)p.theta, marker_sz, marker_col);
    }
}

void App::generate_and_solve() {
    constexpr double kPi = 3.14159265358979323846;

    // ground-truth: figure-8 (lemniscate) trajectory
    // x(t) = a * sin(t),  y(t) = a * sin(t) * cos(t)
    double a = 6.0;
    ground_truth.resize(num_poses);
    for (int i = 0; i < num_poses; ++i) {
        double t  = 2.0 * kPi * i / num_poses;
        double x  = a * std::sin(t);
        double y  = a * std::sin(t) * std::cos(t);

        // heading = tangent direction
        double dxdt = a * std::cos(t);
        double dydt = a * (std::cos(t) * std::cos(t) - std::sin(t) * std::sin(t));
        double theta = std::atan2(dydt, dxdt);

        ground_truth[i] = {x, y, theta};
    }

    std::mt19937 rng{std::random_device{}()};
    std::normal_distribution<double> nt(0.0, noise_translation);
    std::normal_distribution<double> nr(0.0, noise_rotation);

    // noisy relative-pose measurements (odometry + one loop closure back to 0)
    struct Meas { int from, to; opt::Vec3 z; bool lc; };
    std::vector<Meas> measurements;

    for (int i = 0; i < num_poses; ++i) {
        int j = (i + 1) % num_poses;
        auto& pi = ground_truth[i];
        auto& pj = ground_truth[j];

        double dx = pj.x - pi.x, dy = pj.y - pi.y;
        double c = std::cos(pi.theta), s = std::sin(pi.theta);

        opt::Vec3 z;
        z <<  c * dx + s * dy + nt(rng),
             -s * dx + c * dy + nt(rng),
              opt::normalize_angle(pj.theta - pi.theta) + nr(rng);

        measurements.push_back({i, j, z, (j == 0)});
    }

    edge_info.clear();
    for (auto& m : measurements)
        edge_info.push_back({m.from, m.to, m.lc});

    // initial guess: accumulate noisy odometry (drifts, doesn't close)
    initial_poses.resize(num_poses);
    initial_poses[0] = ground_truth[0];
    for (int i = 0; i < num_poses - 1; ++i) {
        auto& prev = initial_poses[i];
        auto& z    = measurements[i].z;
        double c = std::cos(prev.theta), s = std::sin(prev.theta);
        initial_poses[i + 1] = {prev.x + c * z(0) - s * z(1),
                                prev.y + s * z(0) + c * z(1),
                                opt::normalize_angle(prev.theta + z(2))};
    }

    // scatter random landmarks around the trajectory bounding box
    double lm_spread = a + 3.0;
    std::uniform_real_distribution<double> ldx(-lm_spread, lm_spread);
    std::uniform_real_distribution<double> ldy(-lm_spread * 0.6, lm_spread * 0.6);
    std::normal_distribution<double> no(0.0, noise_observation);

    gt_landmarks.resize(num_landmarks);
    for (int i = 0; i < num_landmarks; ++i)
        gt_landmarks[i] = {ldx(rng), ldy(rng)};

    // build observation list: each pose sees landmarks within observation_radius
    struct Obs { int pose_id, lm_id; Eigen::Vector2d z; };
    std::vector<Obs> observations;
    landmark_visible.assign(num_landmarks, false);

    for (int pi = 0; pi < num_poses; ++pi) {
        auto& pose = ground_truth[pi];
        double c = std::cos(pose.theta), s = std::sin(pose.theta);
        for (int li = 0; li < num_landmarks; ++li) {
            double dx = gt_landmarks[li].x - pose.x;
            double dy = gt_landmarks[li].y - pose.y;
            if (dx * dx + dy * dy > observation_radius * observation_radius) continue;

            // observation in the local frame of the pose, with noise
            Eigen::Vector2d z;
            z <<  c * dx + s * dy + no(rng),
                 -s * dx + c * dy + no(rng);
            observations.push_back({pi, li, z});
            landmark_visible[li] = true;
        }
    }

    // initialize each landmark from its first observation via the (noisy) pose
    initial_landmarks.resize(num_landmarks);
    std::vector<bool> lm_init(num_landmarks, false);
    for (auto& ob : observations) {
        if (lm_init[ob.lm_id]) continue;
        auto& pose = initial_poses[ob.pose_id];
        double c = std::cos(pose.theta), s = std::sin(pose.theta);
        initial_landmarks[ob.lm_id] = {
            pose.x + c * ob.z(0) - s * ob.z(1),
            pose.y + s * ob.z(0) + c * ob.z(1)
        };
        lm_init[ob.lm_id] = true;
    }

    // --- build the optimization graph ---
    opt::Problem problem;

    // pose vertices (first pose is fixed as the anchor)
    std::vector<std::shared_ptr<opt::PoseVertex>> verts;
    verts.reserve(num_poses);
    for (int i = 0; i < num_poses; ++i) {
        auto v = std::make_shared<opt::PoseVertex>();
        opt::VecX p(3);
        p << initial_poses[i].x, initial_poses[i].y, initial_poses[i].theta;
        v->set_parameters(p);
        if (i == 0) v->set_fixed(true);
        verts.push_back(v);
        problem.add_vertex(v);
    }

    // odometry + loop-closure edges
    for (auto& m : measurements) {
        auto e = std::make_shared<opt::PoseEdge>();
        e->set_measurement(m.z);
        e->set_vertices({verts[m.from], verts[m.to]});

        opt::MatXX info = opt::MatXX::Identity(3, 3);
        double inv_t2 = 1.0 / (noise_translation * noise_translation);
        double inv_r2 = 1.0 / (noise_rotation   * noise_rotation);
        info(0, 0) = inv_t2; info(1, 1) = inv_t2; info(2, 2) = inv_r2;
        e->set_information(info);
        problem.add_edge(e);
    }

    // landmark vertices
    std::vector<std::shared_ptr<opt::PointVertex>> lm_verts(num_landmarks);
    for (int i = 0; i < num_landmarks; ++i) {
        if (!landmark_visible[i]) continue;
        auto v = std::make_shared<opt::PointVertex>();
        opt::VecX p(2);
        p << initial_landmarks[i].x, initial_landmarks[i].y;
        v->set_parameters(p);
        lm_verts[i] = v;
        problem.add_vertex(v);
    }

    // observation edges (pose -> landmark)
    opt::MatXX obs_info = opt::MatXX::Identity(2, 2) / (noise_observation * noise_observation);
    for (auto& ob : observations) {
        if (!lm_verts[ob.lm_id]) continue;
        auto e = std::make_shared<opt::PointEdge>();
        e->set_measurement(ob.z);
        e->set_vertices({verts[ob.pose_id], lm_verts[ob.lm_id]});
        e->set_information(obs_info);
        problem.add_edge(e);
    }

    // --- solve and log to console ---
    int visible_lm = 0;
    for (bool v : landmark_visible) if (v) ++visible_lm;

    std::printf("=== Pose Graph Optimization (LM) ===\n");
    std::printf("Poses: %d  Landmarks: %d (visible: %d)  Observations: %d\n",
                num_poses, num_landmarks, visible_lm, (int)observations.size());
    std::printf("Odometry edges: %d (incl. 1 loop closure)\n", num_poses);
    std::printf("------------------------------------\n");
    std::printf("%4s  %14s  %14s\n", "iter", "chi2", "lambda");

    convergence.clear();
    opt_history.clear();
    landmark_history.clear();

    problem.solve(50, [&](int iter, double chi2, double lambda) {
        convergence.push_back({iter, chi2, lambda});
        std::printf("%4d  %14.6f  %14.8f\n", iter, chi2, lambda);

        // snapshot poses
        std::vector<Pose2D> snap;
        snap.reserve(num_poses);
        for (auto& v : verts) {
            auto& p = v->parameters();
            snap.push_back({p(0), p(1), p(2)});
        }
        opt_history.push_back(std::move(snap));

        // snapshot landmarks
        std::vector<Point2D> lm_snap(num_landmarks);
        for (int i = 0; i < num_landmarks; ++i) {
            if (lm_verts[i]) {
                auto& p = lm_verts[i]->parameters();
                lm_snap[i] = {p(0), p(1)};
            }
        }
        landmark_history.push_back(std::move(lm_snap));
    });

    std::printf("------------------------------------\n");
    if (!convergence.empty())
        std::printf("Final chi2: %.6f  (%.1fx reduction)\n",
                    convergence.back().chi2,
                    convergence.front().chi2 / std::max(convergence.back().chi2, 1e-15));

    max_frame = std::max(0, (int)opt_history.size() - 1);
    frame     = 0;
}

void App::init() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(WIDTH, HEIGHT, "pgo2d");
    SetWindowMinSize(640, 480);
    SetTargetFPS(60);

    g_font = GetFontDefault();

    generate_and_solve();
}

void App::run() {
    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_RIGHT) && frame < max_frame) ++frame;
        if (IsKeyPressed(KEY_LEFT)  && frame > 0)         --frame;
        if (IsKeyPressed(KEY_HOME))                        frame = 0;
        if (IsKeyPressed(KEY_END))                         frame = max_frame;

        BeginDrawing();
        ClearBackground(COL_BG);
        draw_scene();
        draw_hud();
        EndDrawing();
    }
    CloseWindow();
}

void App::draw_scene() {
    if (ground_truth.empty()) return;

    float w = (float)GetScreenWidth(), h = (float)GetScreenHeight();
    ViewXform vw = fit_view({20, 40, w - 40, h - 60},
                            ground_truth, initial_poses, gt_landmarks);

    int f = std::min(frame, max_frame);
    const auto& poses_now = opt_history.empty() ? initial_poses : opt_history[f];

    // odometry / loop-closure edges
    for (auto& ei : edge_info) {
        auto a = vw.to_screen(poses_now[ei.from].x, poses_now[ei.from].y);
        auto b = vw.to_screen(poses_now[ei.to].x,   poses_now[ei.to].y);
        DrawLineEx(a, b, ei.is_loop_closure ? 2.5f : 1.0f,
                   ei.is_loop_closure ? COL_LOOP : COL_EDGE);
    }

    // ground-truth landmarks (small crosses)
    for (int i = 0; i < (int)gt_landmarks.size(); ++i) {
        if (i < (int)landmark_visible.size() && !landmark_visible[i]) continue;
        auto sp = vw.to_screen(gt_landmarks[i].x, gt_landmarks[i].y);
        DrawLineEx({sp.x - 3, sp.y - 3}, {sp.x + 3, sp.y + 3}, 1.0f, COL_LM_GT);
        DrawLineEx({sp.x + 3, sp.y - 3}, {sp.x - 3, sp.y + 3}, 1.0f, COL_LM_GT);
    }

    // initial landmarks
    for (int i = 0; i < (int)initial_landmarks.size(); ++i) {
        if (i < (int)landmark_visible.size() && !landmark_visible[i]) continue;
        auto sp = vw.to_screen(initial_landmarks[i].x, initial_landmarks[i].y);
        DrawCircleV(sp, 2.5f, COL_LM_INIT);
    }

    // optimized landmarks at current iteration
    if (!landmark_history.empty() && f < (int)landmark_history.size()) {
        const auto& lm = landmark_history[f];
        for (int i = 0; i < (int)lm.size(); ++i) {
            if (i < (int)landmark_visible.size() && !landmark_visible[i]) continue;
            auto sp = vw.to_screen(lm[i].x, lm[i].y);
            DrawCircleV(sp, 3.5f, COL_LM_OPT);
        }
    }

    // trajectories
    draw_trajectory(vw, ground_truth,  COL_GT,   COL_GT,   7.0f);
    draw_trajectory(vw, initial_poses, COL_INIT, COL_INIT, 6.0f);
    if (!opt_history.empty())
        draw_trajectory(vw, opt_history[f], COL_OPT, COL_OPT, 9.0f);
}

void App::draw_hud() {
    float w = (float)GetScreenWidth(), h = (float)GetScreenHeight();

    // iteration counter (top-right)
    const char* iter_text = TextFormat("Iteration: %d / %d", frame, max_frame);
    float tw = measure_text(iter_text, 18);
    draw_text(iter_text, w - tw - 12, 8, 18, COL_OPT);

    // chi2 for current frame
    if (!convergence.empty()) {
        int idx = std::min(frame, (int)convergence.size() - 1);
        const char* chi_text = TextFormat("chi2 = %.4f", convergence[idx].chi2);
        tw = measure_text(chi_text, 16);
        draw_text(chi_text, w - tw - 12, 30, 16, COL_LABEL);
    }

    // legend (top-left)
    float ly = 10;
    auto legend = [&](const char* txt, Color c) {
        DrawCircleV({22, ly + 7}, 4, c);
        draw_text(txt, 34, ly, 15, c);
        ly += 20;
    };
    legend("Ground truth",    COL_GT);
    legend("Initial (odometry)", COL_INIT);
    legend("Optimized (LM)",    COL_OPT);
    legend("Loop closure",      COL_LOOP);

    // controls hint (bottom-right)
    const char* hint = "[Left/Right] step   [Home/End] jump";
    tw = measure_text(hint, 14);
    draw_text(hint, w - tw - 12, h - 26, 14, COL_LABEL);
}
