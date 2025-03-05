#include "rigsystem_common.hpp"
// #include <omp.h>

// #include <yaml-cpp/yaml.h> // uses exceptions

#include <assert.h>
#include <vector>

// #include <godot_cpp/variant/utility_functions.hpp>
#define DEBUG_PRINT(...) (void)0 // godot::UtilityFunctions::print(__VA_ARGS__) //

namespace rigsystem {

// rigsystem::vec3 v(const Vector3& v) { return rigsystem::vec3(v.x, v.y, v.z);
// } Vector3 v(const rigsystem::vec3& v) { return Vector3(v.x, v.y, v.z); }

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

void RigSystemCommon::clear() { m_rss.clear(); }

void RigSystemCommon::add_node(Node n)
{
    m_rss.add_node(n);
}

void RigSystemCommon::add_conn(Conn c)
{
    m_rss.add_conn(c);
}

size_t RigSystemCommon::get_nodes_num() const { return m_rss.get_nodes_num(); }

size_t RigSystemCommon::get_conns_num() const { return m_rss.get_conns_num(); }

// this is the heaviest method here (~75% time spent here according to callgrind
// and perf, mainly vector ops; glm::vec3 did not help); TODO optimize
void RigSystemCommon::compute_system_forces(std::vector<Node>& nv)
{
    const auto num_nodes = get_nodes_num();
    const auto num_conns = get_conns_num();

    for (size_t i = 0; i < num_nodes; ++i) {
        nv[i].frc = m_rss.node_forces_external[i];
    }

    for (size_t ci = 0; ci < num_conns; ci++) { //      the hottest loop
        Conn& c = m_rss.conns[ci];

        [[unlikely]] if (c.broken)
            continue;

        const size_t ni = c.node_a;
        const size_t nj = c.node_b;

        const vec3& pin_i = nv[ni].pos;
        const vec3& pin_j = nv[nj].pos;
        const vec3& vel_i = nv[ni].vel;
        const vec3& vel_j = nv[nj].vel;

        const vec3 delta = pin_j - pin_i;
        const vec3 relative_vel = vel_j - vel_i;
        const float dist = delta.length();

        [[unlikely]] if (dist <= 0)
            continue;

        const float extension = dist - c.len;
        const vec3 dir = delta / dist;
        const float fspring = c.stiff * extension;
        const float fdamp = c.damp * (relative_vel.dot(dir));
        const vec3 force = (fspring + fdamp) * dir;

        nv[ni].frc += force;
        nv[nj].frc -= force;
    }

    for (size_t i = 0; i < num_nodes; ++i) {
        nv[i].frc += nv[i].mass * c::world::gravity; // TODO: move elsewhere
    }
}

void RigSystemCommon::system_derivative(std::vector<Node>& nv)
{
    const auto num_nodes = get_nodes_num();
    // const auto num_conns = get_conns_num();

    compute_system_forces(nv);

    // acc_out.resize(num_nodes);
    for (size_t i = 0; i < num_nodes; ++i) {
        nv[i].acc = nv[i].frc / nv[i].mass;
    }

    // DEBUG_PRINT("[RigSystemCommon::system_derivative] // } ");
}

// ~15% time spent here according to callgrind and perf - not critical for now
void RigSystemCommon::integrate_system_radau2(float dt)
{
    const size_t num_nodes = get_nodes_num();
    // const auto num_conns = get_conns_num();

    system_derivative(m_rss.nodes);

    m_rss.nodes_1 = m_rss.nodes;
    m_rss.nodes_2 = m_rss.nodes;
    m_rss.nodes_k1 = m_rss.nodes;
    m_rss.nodes_k2 = m_rss.nodes;
    m_rss.nodes_k1_new = m_rss.nodes;
    m_rss.nodes_k2_new = m_rss.nodes;

    for (int i = 0; i < c::system::max_iter_num; ++i) {
        for (size_t j = 0; j < num_nodes; ++j) {
            const auto& npos = m_rss.nodes[j].pos;
            const auto& nvel = m_rss.nodes[j].vel;

            m_rss.nodes_1[j].pos = (npos + dt * (c::radau2::A11 * m_rss.nodes_k1[j].vel + c::radau2::A12 * m_rss.nodes_k2[j].vel));
            m_rss.nodes_1[j].vel = (nvel + dt * (c::radau2::A11 * m_rss.nodes_k1[j].acc + c::radau2::A12 * m_rss.nodes_k2[j].acc));
            m_rss.nodes_2[j].pos = (npos + dt * (c::radau2::A21 * m_rss.nodes_k1[j].vel + c::radau2::A22 * m_rss.nodes_k2[j].vel));
            m_rss.nodes_2[j].vel = (nvel + dt * (c::radau2::A21 * m_rss.nodes_k1[j].acc + c::radau2::A22 * m_rss.nodes_k2[j].acc));
        }

        system_derivative(m_rss.nodes_1);
        system_derivative(m_rss.nodes_2);

        m_rss.nodes_k1_new = m_rss.nodes_1;
        m_rss.nodes_k2_new = m_rss.nodes_2;

        float diff2 = 0;
        for (size_t j = 0; j < num_nodes; ++j) {
            diff2 += m_rss.nodes_k1_new[j].vel.distance_to2(m_rss.nodes_k1[j].vel);
            diff2 += m_rss.nodes_k1_new[j].acc.distance_to2(m_rss.nodes_k1[j].acc);
            diff2 += m_rss.nodes_k2_new[j].vel.distance_to2(m_rss.nodes_k2[j].vel);
            diff2 += m_rss.nodes_k2_new[j].acc.distance_to2(m_rss.nodes_k2[j].acc);
        }

        [[unlikely]] if (diff2 < c::system::max_error2) {
            m_rss.nodes_k1 = m_rss.nodes_k1_new;
            m_rss.nodes_k2 = m_rss.nodes_k2_new;
            break;
        }

        m_rss.nodes_k1 = m_rss.nodes_k1_new;
        m_rss.nodes_k2 = m_rss.nodes_k2_new;
    }

    for (size_t i = 0; i < num_nodes; ++i) {
        vec3& npos = m_rss.nodes[i].pos;
        vec3& nvel = m_rss.nodes[i].vel;

        [[likely]] if (!m_rss.nodes[i].pinned) {
            npos = (npos + dt * (c::radau2::b1 * m_rss.nodes_k1[i].vel + c::radau2::b2 * m_rss.nodes_k2[i].vel));
            nvel = (nvel + dt * (c::radau2::b2 * m_rss.nodes_k1[i].acc + c::radau2::b1 * m_rss.nodes_k2[i].acc));
        } else {
            nvel = (vec3(0.0f, 0.0f, 0.0f));
        }
    }

    for (size_t i = 0; i < m_rss.get_conns_num(); ++i) {
        // check for broken conns
        const size_t ni = m_rss.conns[i].node_a;
        const size_t nj = m_rss.conns[i].node_b;

        const vec3 delta = m_rss.nodes[nj].pos - m_rss.nodes[ni].pos;
        const float dist = delta.length();
        [[unlikely]] if (dist < 0.00001f) {
            m_rss.conns[i].broken = true;
            // DEBUG_PRINT("[RigSystemCommon::integrate_system_radau2] broken!1");
            continue;
        }

        const float extension = dist - m_rss.conns[i].len;
        [[unlikely]] if (std::abs(extension) > m_rss.conns[i].brk_thr) {
            m_rss.conns[i].broken = true;
            // DEBUG_PRINT("[RigSystemCommon::integrate_system_radau2] broken!2");
            continue;
        }
    }
}

} // namespace rigsystem
