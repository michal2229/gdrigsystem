#include "gdrigsystem.hpp"
#include "rigsystem_lib/src/rigsystem_common.hpp"

#include <godot_cpp/classes/cylinder_mesh.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <chrono>
#include <ratio>
#include <unistd.h>
#include <assert.h>
#include <vector>

using namespace godot;
using namespace rigsystem;

#define DEBUG_PRINT(...) (void)0 // UtilityFunctions::print(__VA_ARGS__) //

vec3 godot::v(const Vector3& v) { return vec3(v.x, v.y, v.z); }
vec3 godot::av(const Array& v) { return vec3(v[0], v[1], v[2]); }
Vector3 godot::v(const vec3& v) { return Vector3(v.x, v.y, v.z); }

GDRigSystem::GDRigSystem()
{
}

GDRigSystem::~GDRigSystem()
{

}

void GDRigSystem::_bind_methods()
{
    ClassDB::bind_method(godot::D_METHOD("add_node", "node_dict", "node_defaults", "node_visual"), &GDRigSystem::add_node);
    ClassDB::bind_method(godot::D_METHOD("add_conn", "conn_dict", "conn_defaults", "conn_visual"), &GDRigSystem::add_conn);
    ClassDB::bind_method(godot::D_METHOD("initialize_nodes", "parent"), &GDRigSystem::initialize_nodes);
    ClassDB::bind_method(godot::D_METHOD("initialize_connections", "parent"), &GDRigSystem::initialize_connections);
    ClassDB::bind_method(godot::D_METHOD("simulate", "active"), &GDRigSystem::simulate);
    ClassDB::bind_method(godot::D_METHOD("visualize", "active"), &GDRigSystem::visualize);
    ClassDB::bind_method(godot::D_METHOD("clear"), &GDRigSystem::clear);
}

void GDRigSystem::_process(double delta)
{
    if (!state_sim_active)
        return;

    if (m_rs.get_nodes_num() == 0)
        return;

    DEBUG_PRINT("[_process] { ");

    if (state_visuals_active)
        align_visuals();

    DEBUG_PRINT("[_process] // } ");
}

void GDRigSystem::_physics_process(double delta)
{
    if (!state_sim_active || m_rs.get_nodes_num() == 0)
        return;

    DEBUG_PRINT("[_physics_process] {");
    last_delta = delta;

    if (m_init_done)
        fut.get();
    {
        for (size_t i = 0; i < m_rs.get_nodes_num(); i++) {
            DEBUG_PRINT("[_physics_process] A2 { ");
            Node* n = m_p_nodes[i];
            Node3D* n3d = Object::cast_to<Node3D>(n);
            m_rs.m_rss.node_forces_external[i] = v(n3d->get("force")) * 8.0f; // forces from godot

            DEBUG_PRINT("[_physics_process] A2 // } ");
        }
    }
    fut = std::async(std::launch::async, [&]() {
        for (int i = 0; i < 16; ++i) {

            DEBUG_PRINT("[_physics_process] B1 { ");
            m_rs.integrate_system_radau2(last_delta / 16.0);

            for (size_t i = 0; i < m_rs.get_nodes_num(); i++) {
                m_rs.m_rss.node_forces_external[i] *= 0.5f;
            }
            DEBUG_PRINT("[_physics_process] B1 // } ");
        }

        for (size_t i = 0; i < m_rs.get_nodes_num(); i++) {
            DEBUG_PRINT("[_physics_process] B2 { ");

            // damp velocity
            m_rs.get_node_vel(i) *= (1 - last_delta * 0.5);

            // Node* n = m_p_nodes[i];
            // Node3D* n3d = Object::cast_to<Node3D>(n);
            // n3d->set_global_position(v(m_rs.get_node_pos(i)));  // position to render in godot
            // n->set("velocity", v(m_rs.get_node_vel(i)));

            DEBUG_PRINT("[_physics_process] B2 // } ");
        }
    });
    // fut.get();
    m_init_done = true;
    DEBUG_PRINT("[_physics_process] // }");
}

void GDRigSystem::simulate(bool active)
{
    state_sim_active = active;
}

void GDRigSystem::visualize(bool active)
{
    state_visuals_active = active;
}

void GDRigSystem::clear()
{
    visualize(false);
    simulate(false);

    m_rs.clear();
    m_p_nodes.clear();
    m_p_conns.clear();
    m_p_conns_nodes.clear();
    m_p_conns_visual.clear();
}

void GDRigSystem::add_node(Dictionary node_dict, Dictionary node_defaults, Node* visual)
{
    rigsystem::Node n;

    Array keys_default = node_defaults.keys();
    for (int i = 0; i < keys_default.size(); i++) {
        String key = keys_default[i];
        Variant value = node_defaults[key];
        const std::string k = key.utf8().get_data();

        if (k.compare(std::string("mass")) == 0)
            n.mass = value;
        if (k.compare(std::string("pos")) == 0) {
            Array vv = static_cast<Array>(value);
            n.pos = av(vv);
        };
        if (k.compare(std::string("vel")) == 0) {
            Array vv = static_cast<Array>(value);
            n.vel = av(vv);
        };
        if (k.compare(std::string("acc")) == 0) {
            Array vv = static_cast<Array>(value);
            n.acc = av(vv);
        };
        if (k.compare(std::string("frc")) == 0) {
            Array vv = static_cast<Array>(value);
            n.frc = av(vv);
        };
        if (k.compare(std::string("pinned")) == 0)
            n.pinned = static_cast<bool>(value);
    }

    Array keys_node = node_dict.keys();
    for (int i = 0; i < keys_node.size(); i++) {
        String key = keys_node[i];
        Variant value = node_dict[key];
        const std::string k = key.utf8().get_data();

        if (k.compare(std::string("mass")) == 0)
            n.mass = value;
        if (k.compare(std::string("pos")) == 0) {
            Array vv = static_cast<Array>(value);
            n.pos = av(vv);
        };
        if (k.compare(std::string("vel")) == 0) {
            Array vv = static_cast<Array>(value);
            n.vel = av(vv);
        };
        if (k.compare(std::string("acc")) == 0) {
            Array vv = static_cast<Array>(value);
            n.acc = av(vv);
        };
        if (k.compare(std::string("frc")) == 0) {
            Array vv = static_cast<Array>(value);
            n.frc = av(vv);
        };
        if (k.compare(std::string("pinned")) == 0)
            n.pinned = static_cast<bool>(value);
    }

    m_rs.add_node(n);
    m_p_nodes.push_back(visual);
    std::cout << "[GDRigSystem::add_node] added node: pos = " << n.pos << "\n";
}

void GDRigSystem::add_conn(Dictionary conn_dict, Dictionary conn_defaults, Node* visual)
{
    rigsystem::Conn c;

    Array keys_default = conn_defaults.keys();
    for (int i = 0; i < keys_default.size(); i++) {
        String key = keys_default[i];
        Variant value = conn_defaults[key];
        const std::string k = key.utf8().get_data();

        if (k.compare(std::string("node_a")) == 0)
            c.node_a = value;
        if (k.compare(std::string("node_b")) == 0)
            c.node_b = value;
        if (k.compare(std::string("len")) == 0)
            c.len = value;
        if (k.compare(std::string("stiff")) == 0)
            c.stiff = value;
        if (k.compare(std::string("damp")) == 0)
            c.damp = value;
        if (k.compare(std::string("brk_thr")) == 0)
            c.brk_thr = value;
        if (k.compare(std::string("broken")) == 0)
            c.broken = static_cast<bool>(value);
    }

    Array keys_conn = conn_dict.keys();
    for (int i = 0; i < keys_conn.size(); i++) {
        String key = keys_conn[i];
        Variant value = conn_dict[key];
        const std::string k = key.utf8().get_data();

        if (k.compare(std::string("node_a")) == 0)
            c.node_a = value;
        if (k.compare(std::string("node_b")) == 0)
            c.node_b = value;
        if (k.compare(std::string("len")) == 0)
            c.len = value;
        if (k.compare(std::string("stiff")) == 0)
            c.stiff = value;
        if (k.compare(std::string("damp")) == 0)
            c.damp = value;
        if (k.compare(std::string("brk_thr")) == 0)
            c.brk_thr = value;
        if (k.compare(std::string("broken")) == 0)
            c.broken = static_cast<bool>(value);
    }

    m_rs.add_conn(c);

    Node* node_a = Object::cast_to<Node>(visual->get("rig_node_a"));
    Node* node_b = Object::cast_to<Node>(visual->get("rig_node_b"));
    Node3D* node_a3d = Object::cast_to<Node3D>(node_a);
    Node3D* node_b3d = Object::cast_to<Node3D>(node_b);

    m_p_conns.push_back(visual);
    m_p_conns_visual.push_back(Object::cast_to<MeshInstance3D>(visual->get_node_or_null(NodePath("RigConnVisualMesh"))));
    m_p_conns_nodes.push_back(std::make_pair(node_a3d, node_b3d));

    std::cout << "[GDRigSystem::add_conn] added conn " << c.node_a << " -- " << c.node_b << "\n";
}

void GDRigSystem::align_visuals()
{
    DEBUG_PRINT("[align_visuals] { ");

    for (size_t i = 0; i < m_p_nodes.size(); i++) {
        Node* n = m_p_nodes[i];
        Node3D* n3d = Object::cast_to<Node3D>(n);
        n3d->set_global_position(v(m_rs.get_node_pos(i)));
        n->set("velocity", v(m_rs.get_node_vel(i)));
    }

    for (size_t i = 0; i < m_p_conns.size(); i++) {
        Node* c = m_p_conns[i];
        if (!c) continue;
        c->set("broken", m_rs.get_conn_broken(i));

        Node3D* c3d = Object::cast_to<Node3D>(c);
        if (!c3d)
            continue;

        MeshInstance3D* cvm3d = m_p_conns_visual[i];
        if (!cvm3d)
            continue;

        if (!cvm3d->is_visible())
            continue;
        if (m_rs.get_conn_broken(i)) {
            cvm3d->set_visible(false);
            continue;
        }

        Node3D* node_a = m_p_conns_nodes[i].first;
        Node3D* node_b = m_p_conns_nodes[i].second;
        if (!node_a || !node_b)
            continue;

        vec3 pa = m_rs.get_conn_node_a_pos(i);
        vec3 pb = m_rs.get_conn_node_b_pos(i);

        float dist = pa.distance_to(pb);

        Ref<CylinderMesh> cm = cvm3d->get_mesh();

        // cm -> set_height( dist );  // heavy perf hit
        cvm3d->set("scale", Vector3(1, dist, 1));

        if (dist < 0.01) {
            c3d->set_global_position(v(pa));
            continue;
        }

        auto upvec = Vector3(0.3446546655447658f, 0.276987544556753f, 0.9854435333356f); // a bit hacky, but will do for now

        c3d->look_at_from_position(v(pa / 2.0f) + v(pb / 2.0f), v(pb), upvec);
    }

    DEBUG_PRINT("[align_visuals] // }");
}

void GDRigSystem::initialize_nodes(Node* parent)
{
    DEBUG_PRINT("[initialize_nodes] { ");

    m_rs.clear();

    TypedArray<Node> rignodes = parent->get_children();

    for (size_t i = 0; i < static_cast<size_t>(rignodes.size()); i++) {
        DEBUG_PRINT("[initialize_nodes] i = " + String::num(i));

        Node* n = parent->get_child(i);
        Node3D* n3d = Object::cast_to<Node3D>(n);

        m_p_nodes.push_back(n);
        n->set("id", i);

        rigsystem::Node nn = {
            .pos = v(n3d->get_global_position()),
            .vel = vec3(0, 0, 0),
            .acc = vec3(0, 0, 0),
            .frc = v(n->get("force")),
            .mass = n->get("mass"),
            .pinned = static_cast<bool>(n->get("pinned"))
        };
        m_rs.add_node(nn);
    }

    DEBUG_PRINT("[initialize_nodes] // } ");
}

void GDRigSystem::initialize_connections(Node* parent)
{
    DEBUG_PRINT("[initialize_connections] { ");

    TypedArray<Node> rigconns = parent->get_children();

    for (size_t i = 0; i < static_cast<size_t>(rigconns.size()); i++) {
        DEBUG_PRINT("[initialize_connections] i = " + String::num(i));
        Node* c = parent->get_child(i);
        m_p_conns.push_back(c);
        c->set("id", i);

        Node* node_a = Object::cast_to<Node>(c->get("rig_node_a"));
        Node* node_b = Object::cast_to<Node>(c->get("rig_node_b"));
        Node3D* node_a3d = Object::cast_to<Node3D>(node_a);
        Node3D* node_b3d = Object::cast_to<Node3D>(node_b);

        rigsystem::Conn con_str = {
            .node_a = node_a->get("id"),
            .node_b = node_b->get("id"),
            .len = c->get("rest_length"),
            .stiff = c->get("stiffness"),
            .damp = c->get("damping"),
            .brk_thr = c->get("break_threshold"),
            .broken = static_cast<bool>(c->get("broken"))
        };

        m_rs.add_conn(con_str);

        m_p_conns_visual.push_back(Object::cast_to<MeshInstance3D>(c->get_node_or_null(NodePath("RigConnVisualMesh"))));
        m_p_conns_nodes.push_back(std::make_pair(node_a3d, node_b3d));
    }
    DEBUG_PRINT("[initialize_connections] // } ");
}
