#include "gdrigsystem.hpp"
#include "rigsystem_lib/src/rigsystem_common.hpp"

#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/cylinder_mesh.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/node3d.hpp>

#include <vector>

using namespace godot;
using namespace rigsystem;

#define DEBUG_PRINT(...) (void)0  //UtilityFunctions::print(__VA_ARGS__) //


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
    ClassDB::bind_method( godot::D_METHOD("add_node", "node_dict", "node_defaults"), &GDRigSystem::add_node );
    ClassDB::bind_method( godot::D_METHOD("add_conn", "conn_dict", "conn_defaults"), &GDRigSystem::add_conn );
}


void GDRigSystem::_process(double delta)
{
	if (!m_init_done)
    {
        DEBUG_PRINT("[_process/init] { ");
		initialize_nodes();
		initialize_connections();

		m_init_done = 1;
        DEBUG_PRINT("[_process/init] // } ");
    }

    if (!m_init_done) return;
    if (m_rs.m_s.get_nodes_num() == 0) return;

    DEBUG_PRINT("[_process] { ");

    align_visuals(); 

    DEBUG_PRINT("[_process] // } ");
}


void GDRigSystem::_physics_process(double delta)
{
    if (!m_init_done) return;
    if (m_rs.m_s.get_nodes_num() == 0) return;
    DEBUG_PRINT("[_physics_process] {");

    for (size_t i = 0; i < m_rs.get_nodes_num(); i++)
    {
        DEBUG_PRINT("[_physics_process] A2 { ");
        Node* n = m_p_nodes[i];
        Node3D* n3d = Object::cast_to<Node3D>(n);
        m_rs.m_s.nodes_frc[i] = v(n3d->get("force"));  // forces from godot
        
        DEBUG_PRINT("[_physics_process] A2 // } ");
    }

    for (int i = 0; i < 8; ++i)
        m_rs.integrate_system_radau2(delta / 8.0);
        

    for (size_t i = 0; i < m_rs.get_nodes_num(); i++)
    {
        DEBUG_PRINT("[_physics_process] B { ");

        // damp velocity
        m_rs.m_s.nodes_vel[i] *= (1 - delta * 0.5);

        Node* n = m_p_nodes[i];
        Node3D* n3d = Object::cast_to<Node3D>(n);
        n3d->set_global_position(v(m_rs.m_s.nodes_pos[i]));  // position to render in godot
        n->set("velocity", v(m_rs.m_s.nodes_vel[i]));

        DEBUG_PRINT("[_physics_process] B // } ");
    }
 
    DEBUG_PRINT("[_physics_process] // }");
}


void GDRigSystem::add_node(Dictionary node_dict, Dictionary node_defaults)
{
    rigsystem::Node n;

    Array keys_default = node_defaults.keys();
    for (int i = 0; i < keys_default.size(); i++)
    {
        String key = keys_default[i]; Variant value = node_defaults[key];
        const std::string k = key.utf8().get_data();

        if (k.compare(std::string("id")) == 0) n.id = value;
        if (k.compare(std::string("mass")) == 0) n.mass = value;
        if (k.compare(std::string("pos")) == 0) { Array vv = static_cast<Array>(value); n.pos = av(vv); };
        if (k.compare(std::string("vel")) == 0) { Array vv = static_cast<Array>(value); n.vel = av(vv); };
        if (k.compare(std::string("acc")) == 0) { Array vv = static_cast<Array>(value); n.acc = av(vv); };
        if (k.compare(std::string("frc")) == 0) { Array vv = static_cast<Array>(value); n.frc = av(vv); };
        if (k.compare(std::string("pinned")) == 0) n.pinned = value;
    }

    Array keys_node = node_dict.keys();
    for (int i = 0; i < keys_node.size(); i++)
    {
        String key = keys_node[i]; Variant value = node_dict[key];
        const std::string k = key.utf8().get_data();

        if (k.compare(std::string("id")) == 0) n.id = value;
        if (k.compare(std::string("mass")) == 0) n.mass = value;
        if (k.compare(std::string("pos")) == 0) { Array vv = static_cast<Array>(value); n.pos = av(vv); };
        if (k.compare(std::string("vel")) == 0) { Array vv = static_cast<Array>(value); n.vel = av(vv); };
        if (k.compare(std::string("acc")) == 0) { Array vv = static_cast<Array>(value); n.acc = av(vv); };
        if (k.compare(std::string("frc")) == 0) { Array vv = static_cast<Array>(value); n.frc = av(vv); };
        if (k.compare(std::string("pinned")) == 0) n.pinned = value;
    }

    m_rs.add_node(n);
}

void GDRigSystem::add_conn(Dictionary conn_dict, Dictionary conn_defaults)
{
    rigsystem::Conn c;

    Array keys_default = conn_defaults.keys();
    for (int i = 0; i < keys_default.size(); i++)
    {
        String key = keys_default[i]; Variant value = conn_defaults[key];
        const std::string k = key.utf8().get_data();

        if (k.compare(std::string("id")) == 0) c.id = value;
        if (k.compare(std::string("node_a")) == 0) c.i = value;
        if (k.compare(std::string("node_b")) == 0) c.j = value;
        if (k.compare(std::string("len")) == 0) c.len = value;
        if (k.compare(std::string("stiff")) == 0) c.stiff = value;
        if (k.compare(std::string("damp")) == 0) c.damp = value;
        if (k.compare(std::string("brk_thr")) == 0) c.brk_thr = value;
        if (k.compare(std::string("broken")) == 0) c.broken = value;
    }

    Array keys_conn = conn_dict.keys();
    for (int i = 0; i < keys_conn.size(); i++)
    {
        String key = keys_conn[i]; Variant value = conn_dict[key];
        const std::string k = key.utf8().get_data();

        if (k.compare(std::string("id")) == 0) c.id = value;
        if (k.compare(std::string("node_a")) == 0) c.i = value;
        if (k.compare(std::string("node_b")) == 0) c.j = value;
        if (k.compare(std::string("len")) == 0) c.len = value;
        if (k.compare(std::string("stiff")) == 0) c.stiff = value;
        if (k.compare(std::string("damp")) == 0) c.damp = value;
        if (k.compare(std::string("brk_thr")) == 0) c.brk_thr = value;
        if (k.compare(std::string("broken")) == 0) c.broken = value;
    }

    m_rs.add_conn(c);
}


void GDRigSystem::align_visuals()
{
    DEBUG_PRINT("[align_visuals] { ");

    for (size_t i = 0; i < m_p_nodes.size(); i++)
    {
        Node* n = m_p_nodes[i];
        Node3D* n3d = Object::cast_to<Node3D>(n);
        n3d->set_global_position(v(m_rs.m_s.nodes_pos[i]));
        n->set("velocity", v(m_rs.m_s.nodes_vel[i]));
    }

    for (size_t i = 0; i < m_p_conns.size(); i++)
    {
        Node* c = m_p_conns[i];
        if (!c) continue;
        c->set("broken", m_rs.m_s.conns_broken[i]);

        Node3D* c3d = Object::cast_to<Node3D>(c);
        if (!c3d) continue;

        MeshInstance3D* cvm3d = m_p_conns_visual[i];
        if (!cvm3d) continue;

        if (!cvm3d->is_visible()) continue;
        if (m_rs.m_s.conns_broken[i]) 
        {
            cvm3d->set_visible(false);
            continue;
        }

        Node3D* node_a = m_p_conns_nodes[i].first;
        Node3D* node_b = m_p_conns_nodes[i].second;
        if (!node_a || !node_b) continue;
        
        vec3 pa = m_rs.m_s.nodes_pos[m_rs.m_s.conns_node_a[i]];
        vec3 pb = m_rs.m_s.nodes_pos[m_rs.m_s.conns_node_b[i]];

        float dist = pa.distance_to(pb);

        Ref<CylinderMesh> cm = cvm3d->get_mesh();

        //cm -> set_height( dist );  // heavy perf hit
        cvm3d -> set("scale", Vector3(1,dist,1));

        if (dist < 0.01)
        {
            c3d->set_global_position( v(pa) );
            continue;
        }

        auto upvec = Vector3(0.0, 1.0, 0.0);
        if 	(pa.x == pb.x && pa.z == pb.z) 
            upvec = Vector3(1.0, 0.0, 0.0);

        c3d->look_at_from_position(v(pa/2.0f) + v(pb/2.0f), v(pb), upvec);
    }

    DEBUG_PRINT("[align_visuals] // }");
}


void GDRigSystem::initialize_nodes()
{
    DEBUG_PRINT("[initialize_nodes] { ");

    m_rs.m_s.clear();

    Node* rignodes_group = get_node_or_null(NodePath("../RigNodes"));

    TypedArray<Node> rignodes = rignodes_group->get_children();

    for (size_t i = 0; i < static_cast<size_t>(rignodes.size()); i++)
    {
        DEBUG_PRINT("[initialize_nodes] i = " + String::num(i));

        Node* n = rignodes_group->get_child(i);
        Node3D* n3d = Object::cast_to<Node3D>(n);

        m_p_nodes.push_back(n);
		n->set("id", i);

        rigsystem::Node nn = {
        	.id = i,
            .mass = n->get("mass"),
            .pos = v(n3d->get_global_position()),
            .vel = vec3(0, 0, 0),
            .acc = vec3(0, 0, 0),
            .frc = v(n->get("force")),
            .pinned = static_cast<bool>(n->get("pinned")) 
        };
        m_rs.add_node(nn);
	}

    DEBUG_PRINT("[initialize_nodes] // } ");
}


void GDRigSystem::initialize_connections()
{
    DEBUG_PRINT("[initialize_connections] { ");

    Node* rigconns_group = get_node_or_null(NodePath("../RigConns"));
    TypedArray<Node> rigconns = rigconns_group->get_children();

    for (size_t i = 0; i < static_cast<size_t>(rigconns.size()); i++)
    {
        DEBUG_PRINT("[initialize_connections] i = " + String::num(i));
        Node* c = rigconns_group->get_child(i);
        m_p_conns.push_back(c);
        c->set("id", i);

        Node* node_a = Object::cast_to<Node>(c->get("rig_node_a"));
        Node* node_b = Object::cast_to<Node>(c->get("rig_node_b"));
        Node3D* node_a3d = Object::cast_to<Node3D>(node_a);
        Node3D* node_b3d = Object::cast_to<Node3D>(node_b);

        rigsystem::Conn con_str = {
            .id = i,
            .i = node_a->get("id"),
            .j = node_b->get("id"), 
            .len = c->get("rest_length"),
            .stiff = c->get("stiffness"),
            .damp = c->get("damping"),
            .brk_thr = c->get("break_threshold"),
            .broken = c->get("broken")
        };

        m_rs.add_conn(con_str);

        m_p_conns_visual.push_back( Object::cast_to<MeshInstance3D>(c->get_node_or_null(NodePath("RigConnVisualMesh"))) );
        m_p_conns_nodes.push_back(std::make_pair(node_a3d, node_b3d));

    }
    DEBUG_PRINT("[initialize_connections] // } ");
}
