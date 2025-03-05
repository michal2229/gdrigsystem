#pragma once

#include "rigsystem_vec3.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <span>
#include <string>
#include <vector>

namespace rigsystem {

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

        inline constexpr int max_iter_num = 16;
        inline constexpr float max_error = 1e-3f;
        inline constexpr float max_error2 = max_error * max_error;

    } // namespace system

} // namespace c

struct Node {
    vec3 pos; // node position          // 3*4 = 12B | 12
    vec3 vel; // node velocity          // 3*4 = 12B | 24
    vec3 acc; // node acceleration      // 3*4 = 12B | 36
    vec3 frc; // forces acting on node  // 3*4 = 12B | 48
    float mass; // node mass            //        4B | 52
    char pinned; // node pinned flag    //        1B | 53
};

struct alignas(16) Conn {
    uint16_t node_a; // node a id                 // 2B | 4
    uint16_t node_b; // node b id                 // 2B | 8
    float len; // connection rest length          // 4B | 12
    float stiff; // connection stiffness param    // 4B | 16
    float damp; // connection damping param       // 4B | 20
    float brk_thr; // connection break threshold  // 4B | 24
    char broken; // connection broken flag        // 1B | 25
};

struct RigStructureState {
    std::vector<Node> nodes; // vector of nodes
    std::vector<Conn> conns; // vector of connections between nodes
    std::vector<vec3> node_forces_external;

    size_t nodes_num = 0;  // TODO: maybe use vec.size() ?
    size_t conns_num = 0;

    void clear()
    {
        nodes.clear();
        conns.clear();
        node_forces_external.clear();

        nodes_num = 0;
        conns_num = 0;
    }

    void nodes_reserve(size_t n)
    {
        nodes.reserve(n);
        node_forces_external.reserve(n);
    }

    void conns_reserve(size_t n)
    {
        conns.reserve(n);
    }

    void add_node(Node n)
    {
        nodes.push_back(n);
        node_forces_external.emplace_back(vec3());

        nodes_num++;
    }

    void add_conn(Conn cc)
    {
        Conn c = cc;

        if (c.len < 0.0f && nodes_num > std::max(c.node_a, c.node_b) + 1) {
            c.len = nodes[c.node_a].pos.distance_to(nodes[c.node_b].pos);
        } else {
            if (c.len < 0.0f) {
                std::cerr << "[ERROR] Connection length is negative for connection: " << c.node_a << "-" << c.node_b << "\n";
                c.len = 1.0f;
            } 
        }

        conns.push_back(c);
        conns_num++;
    }

    inline size_t get_nodes_num() const { return nodes_num; }

    inline size_t get_conns_num() const { return conns_num; }


    // temp stuff
    std::vector<Node> nodes_1;
    std::vector<Node> nodes_2;
    std::vector<Node> nodes_k1;
    std::vector<Node> nodes_k2;
    std::vector<Node> nodes_k1_new;
    std::vector<Node> nodes_k2_new;
};

void zero(std::vector<vec3>& v);
void copy(std::span<const vec3> vi, std::vector<vec3>& vo);
// vec3 v(const YAML::Node& node);

class RigSystemCommon {
public:
    void clear();

    void add_node(Node n);
    void add_conn(Conn c);

    size_t get_nodes_num() const;
    size_t get_conns_num() const;

    inline vec3& get_node_pos(size_t id)
    {
        return m_rss.nodes[id].pos;
    }

    inline vec3& get_node_vel(size_t id)
    {
        return m_rss.nodes[id].vel;
    }

    inline vec3& get_node_frc(size_t id)
    {
        return m_rss.nodes[id].frc;
    }

    inline float& get_conn_len(size_t id)
    {
        return m_rss.conns[id].len;
    }

    inline char& get_conn_broken(size_t id)
    {
        return m_rss.conns[id].broken;
    }

    inline vec3& get_conn_node_a_pos(size_t id)
    {
        return m_rss.nodes[m_rss.conns[id].node_a].pos;
    }

    inline vec3& get_conn_node_b_pos(size_t id)
    {
        return m_rss.nodes[m_rss.conns[id].node_b].pos;
    }

    void simulate(float dt);

    void compute_system_forces(std::vector<Node>& nv);

    void system_derivative(std::vector<Node>& nv);

    void integrate_system_radau2(float dt);

    RigStructureState m_rss;
};

} // namespace rigsystem
