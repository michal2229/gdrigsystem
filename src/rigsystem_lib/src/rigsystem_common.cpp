#include "rigsystem_common.hpp"

//#include <godot_cpp/variant/utility_functions.hpp>
#define DEBUG_PRINT(...) (void)0  //UtilityFunctions::print(__VA_ARGS__) // 

namespace rigsystem
{

//rigsystem::vec3 v(const Vector3& v) { return rigsystem::vec3(v.x, v.y, v.z); }
//Vector3 v(const rigsystem::vec3& v) { return Vector3(v.x, v.y, v.z); }

namespace c
{
    inline constexpr float sqrt6 = 2.449489743f;
    //const c1 = (4.0 - sqrt6) / 10.0;
    //const c2 = (4.0 + sqrt6) / 10.0;
    inline constexpr float A11 = (88.0 - 7.0 * sqrt6) / 360.0;
    inline constexpr float A12 = (296.0 - 169.0 * sqrt6) / 1800.0;
    inline constexpr float A21 = (296.0 + 169.0 * sqrt6) / 1800.0;
    inline constexpr float A22 = (88.0 + 7.0 * sqrt6) / 360.0;
    inline constexpr float b1 = (16.0 - sqrt6) / 36.0;
    inline constexpr float b2 = (16.0 + sqrt6) / 36.0;

    const vec3 GRAVITY = vec3(0.0f, -9.81f, 0.0f);
    const int MAX_ITER = 2;
    const float TOL = 1e-4;    
}

void zero(std::vector<vec3>& v)
{
    for (vec3& e : v) e.zero();
}

void copy( std::span<const vec3> vi, std::vector<vec3>& vo)
{   
    vo.resize(vi.size());
    for (int i = 0; i < vi.size(); ++i) vo[i] = vi[i];
}

namespace {
    std::vector<vec3> K1_vel;
    std::vector<vec3> K1_acc;
    std::vector<vec3> K2_vel;
    std::vector<vec3> K2_acc;

    std::vector<vec3> pos1;
    std::vector<vec3> vel1;
    std::vector<vec3> pos2;
    std::vector<vec3> vel2;

    std::vector<vec3> accel1;
    std::vector<vec3> accel2;

    std::vector<vec3> new_K1_vel;
    std::vector<vec3> new_K1_acc;
    std::vector<vec3> new_K2_vel;
    std::vector<vec3> new_K2_acc;

    std::vector<vec3> forces;
    std::vector<vec3> frc_out;
    std::vector<vec3> accel;
}

void RigSystemCommon::clear()
{
    m_s.clear();
}

void RigSystemCommon::add_node(Node n)
{
    m_s.add_node(n);

    DEBUG_PRINT("[RigSystemCommon::add_node] n.id = " + String::num(n.id));
    DEBUG_PRINT("[RigSystemCommon::add_node] n.mass = " + String::num(n.mass));
    DEBUG_PRINT("[RigSystemCommon::add_node] n.pos = ( " 
        + String::num(n.pos.x) 
        + ", " + String::num(n.pos.y) 
        + ", " + String::num(n.pos.z) + " )" 
    );
}

void RigSystemCommon::add_conn(Conn c)
{
    m_s.add_conn(c);

    DEBUG_PRINT("[RigSystemCommon::add_conn] c.id = " + String::num(c.id));
    DEBUG_PRINT("[RigSystemCommon::add_conn] c.node_a = " + String::num(c.i));
    DEBUG_PRINT("[RigSystemCommon::add_conn] c.node_b = " + String::num(c.j));
    DEBUG_PRINT("[RigSystemCommon::add_conn] c.len = " + String::num(c.len));
    DEBUG_PRINT("[RigSystemCommon::add_conn] c.stiff = " + String::num(c.stiff));
    DEBUG_PRINT("[RigSystemCommon::add_conn] c.damp = " + String::num(c.damp));
    DEBUG_PRINT("[RigSystemCommon::add_conn] c.brk_thr = " + String::num(c.brk_thr));
    DEBUG_PRINT("[RigSystemCommon::add_conn] c.broken = " + String::num(c.broken));
}

size_t RigSystemCommon::get_nodes_num()
{
    return m_s.get_nodes_num();
}

size_t RigSystemCommon::get_conns_num()
{
    return m_s.get_conns_num();
}

// this is the heaviest method here (~75% time spent here according to callgrind and perf, mainly vector ops; glm::vec3 did not help); TODO optimize
void RigSystemCommon::compute_system_forces( std::span<const vec3> pos_in, std::span<const vec3> vel_in )
{
    const auto num_nodes = get_nodes_num();
    const auto num_conns = get_conns_num();
    
    forces.resize(num_nodes);
    copy(m_s.nodes_frc, forces);

    for(int ci = 0; ci < m_s.get_conns_num(); ci++)
    {
        //      the hottest loop

        if(m_s.conns_broken[ci]) continue;

        const int ni = m_s.conns_node_a[ci];
        const int nj = m_s.conns_node_b[ci];

        const vec3 delta = pos_in[nj] - pos_in[ni];
        const float dist = delta.length();
        const vec3 dir = delta / dist;
        const float extension = dist - m_s.conns_len[ci];

        const vec3 relative_vel = vel_in[nj] - vel_in[ni];
        const float fspring = m_s.conns_stiff[ci] * extension;
        const float fdamp = m_s.conns_damp[ci] * (relative_vel.dot(dir));
        const vec3 force = (fspring + fdamp) * dir;

        forces[ni] += force;
        forces[nj] -= force; 
    }

    for (int i = 0; i < num_nodes; ++i) 
    {
        forces[i] += m_s.nodes_mass[i] * c::GRAVITY;
    }
}


void RigSystemCommon::system_derivative(std::span<const vec3> pos_in, 
                                        std::span<const vec3> vel_in, 
                                        std::vector<vec3>& acc_out )
{
    const auto num_nodes = get_nodes_num();
    const auto num_conns = get_conns_num();

    compute_system_forces(pos_in, vel_in);

    acc_out.resize(num_nodes);
    for (int i = 0; i < num_nodes; ++i) 
    {
        acc_out[i] = forces[i] / m_s.nodes_mass[i];

        //n.acc = acc_out[i];
    }

    DEBUG_PRINT("[RigSystemCommon::system_derivative] // } ");
}


// ~15% time spent here according to callgrind and perf - not critical for now
void RigSystemCommon::integrate_system_radau2( float dt )
{
    const auto num_nodes  = get_nodes_num();
    const auto num_conns = get_conns_num();
    
    system_derivative(m_s.nodes_pos, m_s.nodes_vel, accel);

    K1_vel.resize(num_nodes);
    K2_vel.resize(num_nodes);
    K1_acc.resize(num_nodes);
    K2_acc.resize(num_nodes);

    for (int i = 0; i < num_nodes; ++i)
    {
        K1_vel[i] = m_s.nodes_vel[i];
        K2_vel[i] = m_s.nodes_vel[i];
        K1_acc[i] = accel[i];
        K2_acc[i] = accel[i];
    }

    accel1.resize(num_nodes);
    accel2.resize(num_nodes);
    pos1.resize(num_nodes);
    vel1.resize(num_nodes);
    pos2.resize(num_nodes);
    vel2.resize(num_nodes);

    for (int i=0; i<c::MAX_ITER; ++i) 
    {
        for (int j=0; j<num_nodes; ++j) 
        {
            const auto& npos = m_s.nodes_pos[j];
            const auto& nvel = m_s.nodes_vel[j];

            pos1[j] = ( npos + dt * (c::A11 * K1_vel[j] + c::A12 * K2_vel[j]) );
            vel1[j] = ( nvel + dt * (c::A11 * K1_acc[j] + c::A12 * K2_acc[j]) );
            pos2[j] = ( npos + dt * (c::A21 * K1_vel[j] + c::A22 * K2_vel[j]) );
            vel2[j] = ( nvel + dt * (c::A21 * K1_acc[j] + c::A22 * K2_acc[j]) );
        }

        system_derivative(pos1, vel1, accel1);
        system_derivative(pos2, vel2, accel2);

        new_K1_vel = vel1;
        new_K1_acc = accel1;
        new_K2_vel = vel2;
        new_K2_acc = accel2;

        float diff = 0;
        for (int i=0; i<num_nodes; ++i)
        {
            diff += new_K1_vel[i].distance_to(K1_vel[i]);
            diff += new_K1_acc[i].distance_to(K1_acc[i]);
            diff += new_K2_vel[i].distance_to(K2_vel[i]);
            diff += new_K2_acc[i].distance_to(K2_acc[i]);
        }

        if (diff < c::TOL)
        {
            K1_vel = new_K1_vel;
            K1_acc = new_K1_acc;
            K2_vel = new_K2_vel;
            K2_acc = new_K2_acc;
            break;
        }

        K1_vel = new_K1_vel;
        K1_acc = new_K1_acc;
        K2_vel = new_K2_vel;
        K2_acc = new_K2_acc;
    }

    for (int i=0; i<num_nodes; ++i)
    {
        vec3& npos = m_s.nodes_pos[i];
        vec3& nvel = m_s.nodes_vel[i];

        if (!m_s.nodes_pinned[i])
        {
            npos = (npos + dt * (c::b1 * K1_vel[i] + c::b2 * K2_vel[i]));
            nvel = (nvel + dt * (c::b2 * K1_acc[i] + c::b1 * K2_acc[i]));
        } 
        else 
        {
            npos = (npos);
            nvel = (vec3(0, 0, 0));
        }
    }

    for (int i = 0; i < m_s.get_conns_num(); ++i )
    {  
        // check for broken conns
        const int ni = m_s.conns_node_a[i];
        const int nj = m_s.conns_node_b[i];

        const vec3 delta = m_s.nodes_pos[nj] - m_s.nodes_pos[ni];
        const float dist = delta.length();
        if (dist < 0.00001f) {
            m_s.conns_broken[i] = true; 
            DEBUG_PRINT("[RigSystemCommon::integrate_system_radau2] broken!1");
            continue; 
        }

        const vec3 dir = delta / dist;
        const float extension = dist - m_s.conns_len[i];
        if (std::abs(extension) > m_s.conns_brk_thr[i]){
            m_s.conns_broken[i] = true; 
            DEBUG_PRINT("[RigSystemCommon::integrate_system_radau2] broken!2");
            continue;
        }
    }
}

}
