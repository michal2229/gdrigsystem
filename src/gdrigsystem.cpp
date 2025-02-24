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


rigsystem::vec3 godot::v(const Vector3& v) { return rigsystem::vec3(v.x, v.y, v.z); }
Vector3 godot::v(const rigsystem::vec3& v) { return Vector3(v.x, v.y, v.z); }


void GDRigSystem::_bind_methods()
{
}


GDRigSystem::GDRigSystem()
{
}


GDRigSystem::~GDRigSystem()
{
}


void GDRigSystem::initialize_nodes()
{
    DEBUG_PRINT("[initialize_nodes] { ");

    
    m_rs.m_s.clear();

    Node* rignodes_group = get_node_or_null(NodePath("../RigNodes"));

    TypedArray<Node> rignodes = rignodes_group->get_children();

    for (int i = 0; i < rignodes.size(); i++)
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

    for (int i = 0; i < rigconns.size(); i++)
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

    for (int i = 0; i < m_rs.get_nodes_num(); i++)
    {
        DEBUG_PRINT("[_physics_process] A2 { ");
        Node* n = m_p_nodes[i];
        Node3D* n3d = Object::cast_to<Node3D>(n);
        m_rs.m_s.nodes_frc[i] = v(n3d->get("force")); 
        
        DEBUG_PRINT("[_physics_process] A2 // } ");
    }

    m_rs.integrate_system_radau2(/*m_pos, m_vel, m_mas, m_pin, */delta /*m_con, m_pos, m_vel, m_frc*/);

    for (int i = 0; i < m_rs.get_nodes_num(); i++)
    {
        DEBUG_PRINT("[_physics_process] B { ");

        Node* n = m_p_nodes[i];
        Node3D* n3d = Object::cast_to<Node3D>(n);
        n3d->set_global_position(v(m_rs.m_s.nodes_pos[i]));
        n->set("velocity", v(m_rs.m_s.nodes_vel[i]));

        DEBUG_PRINT("[_physics_process] B // } ");
    }
 
    DEBUG_PRINT("[_physics_process] // }");
}


void GDRigSystem::align_visuals()
{
    DEBUG_PRINT("[align_visuals] { ");

    for (int i = 0; i < m_p_nodes.size(); i++)
    {
        Node* n = m_p_nodes[i];
        Node3D* n3d = Object::cast_to<Node3D>(n);
        n3d->set_global_position(v(m_rs.m_s.nodes_pos[i]));
        n->set("velocity", v(m_rs.m_s.nodes_vel[i]));
    }

    for (int i = 0; i < m_p_conns.size(); i++)
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
	