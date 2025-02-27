#include "rigsystem_common.hpp"
// #include <omp.h>

// #include <yaml-cpp/yaml.h> // uses exceptions

#include <assert.h>
#include <fstream>
#include <iostream>

// #include <godot_cpp/variant/utility_functions.hpp>
#define DEBUG_PRINT(...) (void)0 // godot::UtilityFunctions::print(__VA_ARGS__) //

namespace rigsystem {

// rigsystem::vec3 v(const Vector3& v) { return rigsystem::vec3(v.x, v.y, v.z);
// } Vector3 v(const rigsystem::vec3& v) { return Vector3(v.x, v.y, v.z); }

// some constants
namespace c {

    namespace radau2 {

        inline constexpr float sqrt6 = 2.449489743f;
        // const c1 = (4.0 - sqrt6) / 10.0;
        // const c2 = (4.0 + sqrt6) / 10.0;
        inline constexpr float A11 = (88.0f - 7.0f * sqrt6) / 360.0f;
        inline constexpr float A12 = (296.0f - 169.0f * sqrt6) / 1800.0f;
        inline constexpr float A21 = (296.0f + 169.0f * sqrt6) / 1800.0f;
        inline constexpr float A22 = (88.0f + 7.0f * sqrt6) / 360.0f;
        inline constexpr float b1 = (16.0f - sqrt6) / 36.0f;
        inline constexpr float b2 = (16.0f + sqrt6) / 36.0f;

    } // namespace radau2

    namespace world {

        const vec3 gravity = vec3(0.0f, -9.81f, 0.0f);
        // TODO: air friction, etc

    } // namespace world

    namespace system {

        const int max_iter_num = 16;
        const float max_error = 1e-4f;

    } // namespace system

} // namespace c

void zero(std::vector<vec3>& v)
{
    for (vec3& e : v)
        e.zero();
}

void copy(std::span<const vec3> vi, std::vector<vec3>& vo)
{
    vo.resize(vi.size());
    for (size_t i = 0; i < vi.size(); ++i)
        vo[i] = vi[i];
}


void RigSystemCommon::clear() { m_s.clear(); }

void RigSystemCommon::add_node(Node n)
{
    m_s.add_node(n);

    // DEBUG_PRINT("[RigSystemCommon::add_node] n.id = " + godot::String::num(n.id));
    // DEBUG_PRINT("[RigSystemCommon::add_node] n.mass = " + godot::String::num(n.mass));
    // DEBUG_PRINT("[RigSystemCommon::add_node] n.pos = ( " + godot::String::num(n.pos.x) + ", " + godot::String::num(n.pos.y) + ", " + godot::String::num(n.pos.z) + " )");
}

void RigSystemCommon::add_conn(Conn c)
{
    m_s.add_conn(c);

    DEBUG_PRINT("[RigSystemCommon::add_conn] c.id = " + godot::String::num(c.id));
    DEBUG_PRINT("[RigSystemCommon::add_conn] c.node_a = " + godot::String::num(c.i));
    DEBUG_PRINT("[RigSystemCommon::add_conn] c.node_b = " + godot::String::num(c.j));
    DEBUG_PRINT("[RigSystemCommon::add_conn] c.len = " + godot::String::num(c.len));
    DEBUG_PRINT("[RigSystemCommon::add_conn] c.stiff = " + godot::String::num(c.stiff));
    DEBUG_PRINT("[RigSystemCommon::add_conn] c.damp = " + godot::String::num(c.damp));
    DEBUG_PRINT("[RigSystemCommon::add_conn] c.brk_thr = " + godot::String::num(c.brk_thr));
    DEBUG_PRINT("[RigSystemCommon::add_conn] c.broken = " + godot::String::num(c.broken));
}

size_t RigSystemCommon::get_nodes_num() const { return m_s.get_nodes_num(); }

size_t RigSystemCommon::get_conns_num() const { return m_s.get_conns_num(); }

// this is the heaviest method here (~75% time spent here according to callgrind
// and perf, mainly vector ops; glm::vec3 did not help); TODO optimize
void RigSystemCommon::compute_system_forces(std::span<const vec3> pos_in,
    std::span<const vec3> vel_in)
{
    const auto num_nodes = get_nodes_num();
    // const auto num_conns = get_conns_num();

    m_s.forces.resize(num_nodes);
    copy(m_s.nodes_frc, m_s.forces);

    for (size_t ci = 0; ci < m_s.get_conns_num(); ci++) {
        //      the hottest loop

        [[unlikely]] if (m_s.conns_broken[ci])
            continue;

        const size_t ni = m_s.conns_node_a[ci];
        const size_t nj = m_s.conns_node_b[ci];
        const float lenn = m_s.conns_len[ci];
        //assert(lenn > 0);
        const float stiff = m_s.conns_stiff[ci];
        const float damp = m_s.conns_damp[ci];
        const vec3& pin_i = pos_in[ni];
        const vec3& pin_j = pos_in[nj];
        const vec3& vel_i = vel_in[ni];
        const vec3& vel_j = vel_in[nj];

        const vec3 delta = pin_j - pin_i;
        const vec3 relative_vel = vel_j - vel_i;

        const float dist = delta.length();
        const float extension = dist - lenn;
        const vec3 dir = delta / dist;

        const float fspring = stiff * extension;
        const float fdamp = damp * (relative_vel.dot(dir));
        const vec3 force = (fspring + fdamp) * dir;

        m_s.forces[ni] += force;
        m_s.forces[nj] -= force;
    }

    for (size_t i = 0; i < num_nodes; ++i) {
        m_s.forces[i] += m_s.nodes_mass[i] * c::world::gravity;
    }
}

void RigSystemCommon::system_derivative(std::span<const vec3> pos_in,
    std::span<const vec3> vel_in,
    std::vector<vec3>& acc_out)
{
    const auto num_nodes = get_nodes_num();
    // const auto num_conns = get_conns_num();

    compute_system_forces(pos_in, vel_in);

    acc_out.resize(num_nodes);
    for (size_t i = 0; i < num_nodes; ++i) {
        acc_out[i] = m_s.forces[i] / m_s.nodes_mass[i];

        // n.acc = acc_out[i];
    }

    // DEBUG_PRINT("[RigSystemCommon::system_derivative] // } ");
}

// ~15% time spent here according to callgrind and perf - not critical for now
void RigSystemCommon::integrate_system_radau2(float dt)
{
    const size_t num_nodes = get_nodes_num();
    // const auto num_conns = get_conns_num();

    system_derivative(m_s.nodes_pos, m_s.nodes_vel, m_s.accel);

    m_s.K1_vel.resize(num_nodes);
    m_s.K2_vel.resize(num_nodes);
    m_s.K1_acc.resize(num_nodes);
    m_s.K2_acc.resize(num_nodes);

    for (size_t i = 0; i < num_nodes; ++i) {
        m_s.K1_vel[i] = m_s.nodes_vel[i];
        m_s.K2_vel[i] = m_s.nodes_vel[i];
        m_s.K1_acc[i] = m_s.accel[i];
        m_s.K2_acc[i] = m_s.accel[i];
    }

    m_s.accel1.resize(num_nodes);
    m_s.accel2.resize(num_nodes);
    m_s.pos1.resize(num_nodes);
    m_s.vel1.resize(num_nodes);
    m_s.pos2.resize(num_nodes);
    m_s.vel2.resize(num_nodes);

    for (int i = 0; i < c::system::max_iter_num; ++i) {
        for (size_t j = 0; j < num_nodes; ++j) {
            const auto& npos = m_s.nodes_pos[j];
            const auto& nvel = m_s.nodes_vel[j];

            m_s.pos1[j] = (npos + dt * (c::radau2::A11 * m_s.K1_vel[j] + c::radau2::A12 * m_s.K2_vel[j]));
            m_s.vel1[j] = (nvel + dt * (c::radau2::A11 * m_s.K1_acc[j] + c::radau2::A12 * m_s.K2_acc[j]));
            m_s.pos2[j] = (npos + dt * (c::radau2::A21 * m_s.K1_vel[j] + c::radau2::A22 * m_s.K2_vel[j]));
            m_s.vel2[j] = (nvel + dt * (c::radau2::A21 * m_s.K1_acc[j] + c::radau2::A22 * m_s.K2_acc[j]));
        }

        system_derivative(m_s.pos1, m_s.vel1, m_s.accel1);
        system_derivative(m_s.pos2, m_s.vel2, m_s.accel2);

        m_s.new_K1_vel = m_s.vel1;
        m_s.new_K1_acc = m_s.accel1;
        m_s.new_K2_vel = m_s.vel2;
        m_s.new_K2_acc = m_s.accel2;
        float diff = 0;
        for (size_t j = 0; j < num_nodes; ++j) {
            diff += m_s.new_K1_vel[j].distance_to(m_s.K1_vel[j]);
            diff += m_s.new_K1_acc[j].distance_to(m_s.K1_acc[j]);
            diff += m_s.new_K2_vel[j].distance_to(m_s.K2_vel[j]);
            diff += m_s.new_K2_acc[j].distance_to(m_s.K2_acc[j]);
        }

        [[unlikely]] if (diff < c::system::max_error) {
            m_s.K1_vel = m_s.new_K1_vel;
            m_s.K1_acc = m_s.new_K1_acc;
            m_s.K2_vel = m_s.new_K2_vel;
            m_s.K2_acc = m_s.new_K2_acc;
            break;
        }

        m_s.K1_vel = m_s.new_K1_vel;
        m_s.K1_acc = m_s.new_K1_acc;
        m_s.K2_vel = m_s.new_K2_vel;
        m_s.K2_acc = m_s.new_K2_acc;
    }

    for (size_t i = 0; i < num_nodes; ++i) {
        vec3& npos = m_s.nodes_pos[i];
        vec3& nvel = m_s.nodes_vel[i];

        [[likely]] if (!m_s.nodes_pinned[i]) {
            npos = (npos + dt * (c::radau2::b1 * m_s.K1_vel[i] + c::radau2::b2 * m_s.K2_vel[i]));
            nvel = (nvel + dt * (c::radau2::b2 * m_s.K1_acc[i] + c::radau2::b1 * m_s.K2_acc[i]));
        } else {
            // npos = (npos);
            nvel = (vec3(0.0f, 0.0f, 0.0f));
        }
    }

    for (size_t i = 0; i < m_s.get_conns_num(); ++i) {
        // check for broken conns
        const size_t ni = m_s.conns_node_a[i];
        const size_t nj = m_s.conns_node_b[i];

        const vec3 delta = m_s.nodes_pos[nj] - m_s.nodes_pos[ni];
        const float dist = delta.length();
        [[unlikely]] if (dist < 0.00001f) {
            m_s.conns_broken[i] = true;
            // DEBUG_PRINT("[RigSystemCommon::integrate_system_radau2] broken!1");
            continue;
        }

        const float extension = dist - m_s.conns_len[i];
        [[unlikely]] if (std::abs(extension) > m_s.conns_brk_thr[i]) {
            m_s.conns_broken[i] = true;
            // DEBUG_PRINT("[RigSystemCommon::integrate_system_radau2] broken!2");
            continue;
        }
    }
}

} // namespace rigsystem
