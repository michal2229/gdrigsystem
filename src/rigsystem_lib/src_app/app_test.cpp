#include <ascii_draw_rigsystem.hpp>
#include <rigsystem_common.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>

// ascii-graphics lib
#include <Camera.hpp>
#include <Lights.hpp>
#include <Mesh.hpp>
#include <Screen.hpp>
#include <agm.hpp>

using namespace rigsystem;
using namespace adr;

namespace app_test {

// ascii screen
const size_t screen_width = 200;
const size_t screen_height = 42;

// rig connections params
const float stiffness_factor = 70000.f;
const float damping_factor = 100.f;
const float break_threshold = 0.2f;

// sim params
const size_t num_iter = 100;
inline constexpr float dt = 1.0f / 60.f / static_cast<float>(num_iter);

// perf note added to ascii buffer
const char perf_note[] = "[PERF] %lu nodes, %lu conns, %lu iter time ms = %.3f";

// rig tower creation function
void create_tower(RigSystemCommon& rs, size_t num_levels, bool horizontal);

}

using namespace app_test;

int main()
{
    srand(42);

    // our ascii renderer
    RigRendererAscii rrend(app_test::screen_width, app_test::screen_height, 30);

    // creating the rig tower to simulate
    RigSystemCommon rs;
    RigSystemCommon rs2;
    create_tower(rs, 8, true);
    create_tower(rs2, 24, false);

    // mesh to update according to current structure state and render as ascii
    const size_t mesh_id2 = rrend.add_mesh(RigWireMesh());
    const size_t mesh_id = rrend.add_mesh(RigWireMesh());

    rrend.get_mesh(mesh_id);
    rrend.get_mesh(mesh_id2);

    rrend.add_transform(mesh_id, rotZ(15.0f));
    rrend.add_transform(mesh_id, adr::translateXYZ(-4.5f, -1.5f, 0.0f));
    size_t tansform_id = rrend.add_transform(mesh_id, rotY(0.0f));
    rrend.add_transform(mesh_id, adr::translateXYZ(0.0f, 0.0f, -4.5f));

    rrend.add_transform(mesh_id2, adr::translateXYZ(-20.0f, -10.0f, -20.0f));

    float roty = -60.0f;
    while (1) {
        auto t1 = std::chrono::high_resolution_clock::now(); // perf

        for (size_t i = 0; i < num_iter; ++i) {
            rs.integrate_system_radau2(dt); // computing phys step
            rs2.integrate_system_radau2(dt); // computing phys step
        }

        auto t2 = std::chrono::high_resolution_clock::now(); // perf
        std::chrono::duration<double, std::milli> ms = t2 - t1;

        rrend.get_transform(mesh_id, tansform_id) = rotY(roty);
        roty -= ms.count() / 50.0f;
        if (roty < -360.0f)
            roty += 720.0f;

        rrend.update(mesh_id, rs);
        rrend.update(mesh_id2, rs2);

        // drawing with ascii in the terminal
        // if not visible, decrease font size and maximize terminal window
        rrend.render();

        // perf note (added test if buffer is too small to avoid segfaults)
        if (std::size(perf_note) < app_test::screen_width - 2)
            sprintf(&(rrend.p_screen->buffer[app_test::screen_height - 2][1]),
                perf_note,
                rs.get_nodes_num() + rs2.get_nodes_num(),
                rs.get_conns_num() + rs2.get_conns_num(),
                num_iter,
                ms.count());
    }

    return 0;
}



namespace app_test {

// create tower rig
void create_tower(RigSystemCommon& rs, size_t num_levels, bool horizontal)
{
    // std::vector<std::pair<size_t, size_t> > conns = { {1, 0}, {2,0}, {3,1} };

    for (size_t i = 0; i < num_levels; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            for (size_t k = 0; k < 2; ++k) {
                Node n = { .id = i * 4 + j * 2 + k,
                    .mass = 1,
                    .pos = horizontal ? vec3(1.0f + i, j - 0.5f, (j ^ k) - 0.5f)
                                      : vec3(j - 0.5f, 1.0f + i, (j ^ k) - 0.5f),
                    .vel = vec3(0.0f, 0.0f, 0.0f),
                    .acc = vec3(0.0f, 0.0f, 0.0f),
                    .frc = vec3(0.0f, 0.0f, 0.0f),
                    .pinned = i == 0 };
                rs.add_node(n);
                std::cout << "[NODE] id = " << n.id << ", pos = (" << n.pos.x << ", "
                          << n.pos.y << ", " << n.pos.z << " )\n";
            }
        }
        if (i >= num_levels - 1) {
            continue;
        }

        std::vector<std::pair<size_t, size_t>> conns_i = {
            { 0 + i * 4, 1 + i * 4 }, { 0 + i * 4, 3 + i * 4 },
            { 0 + i * 4, 4 + i * 4 }, { 0 + i * 4, 5 + i * 4 },
            { 0 + i * 4, 6 + i * 4 }, { 0 + i * 4, 7 + i * 4 },

            { 1 + i * 4, 2 + i * 4 }, { 1 + i * 4, 4 + i * 4 },
            { 1 + i * 4, 5 + i * 4 }, { 1 + i * 4, 6 + i * 4 },
            { 1 + i * 4, 7 + i * 4 },

            { 2 + i * 4, 3 + i * 4 }, { 2 + i * 4, 4 + i * 4 },
            { 2 + i * 4, 5 + i * 4 }, { 2 + i * 4, 6 + i * 4 },
            { 2 + i * 4, 7 + i * 4 },

            { 3 + i * 4, 4 + i * 4 }, { 3 + i * 4, 5 + i * 4 },
            { 3 + i * 4, 6 + i * 4 }, { 3 + i * 4, 7 + i * 4 }
        };

        for (size_t ci = 0; ci < conns_i.size(); ++ci) {
            const auto& p = conns_i[ci];
            Conn c = { .id = i * conns_i.size() + ci,
                .i = p.first,
                .j = p.second,
                .len = -1.0f,
                .stiff = stiffness_factor,
                .damp = damping_factor,
                .brk_thr = break_threshold,
                .broken = false };

            std::cout << "[CONN] id = " << c.id << ", node_a = " << c.i
                      << ", node_b = " << c.j << "\n";
            rs.add_conn(c);
        }
    }

    for (size_t i = 0; i < rs.get_conns_num(); ++i) {
        const vec3& pa = rs.m_s.nodes_pos[rs.m_s.conns_node_a[i]];
        const vec3& pb = rs.m_s.nodes_pos[rs.m_s.conns_node_b[i]];
        const auto dist = pa.distance_to(pb);
        rs.m_s.conns_len[i] = dist;
    }

    for (size_t i = 0; i < rs.get_nodes_num(); ++i) {
        rigsystem::vec3& p = rs.m_s.nodes_pos[i];

        p.x += (static_cast<float>(rand() % 10 + 1) / 10.0f - 0.5f) / 4.0f;
        p.y += (static_cast<float>(rand() % 10 + 1) / 10.0f - 0.5f) / 4.0f;
        p.z += (static_cast<float>(rand() % 10 + 1) / 10.0f - 0.5f) / 4.0f;
    }
}

}
