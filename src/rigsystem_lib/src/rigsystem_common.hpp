#pragma once

#include "rigsystem_vec3.hpp"

#include <cmath>
#include <vector>
#include <span>


namespace rigsystem 
{
struct Node
{
	size_t id;   // node id
	float mass; // node mass
	vec3 pos;  // node position
	vec3 vel;  // node velocity
	vec3 acc;  // node acceleration
	vec3 frc;  // forces acting on node
	bool pinned; // node pinned flag
	char _padding[3] = {}; 
};


struct Conn
{
	size_t id;  // connection id
	size_t i;   // node a id
	size_t j;   // node b id
	float len;  // connection rest length
	float stiff;   // connection stiffness param
	float damp;    // connection damping param
	float brk_thr; // connection break threshold
	bool broken;   // connection broken flag
	char _padding[7] = {}; 
};


struct RigStructureState 
{
	//std::vector<Node> nodes;  // vector of nodes
	//std::vector<Conn> conns;  // vector of connections between nodes

	void clear()
	{
		//nodes.clear();
		//conns.clear();

		nodes_mass.clear();
		nodes_pos.clear();
		nodes_vel.clear();
		nodes_acc.clear();
		nodes_frc.clear();
		nodes_pinned.clear();

		conns_node_a.clear();
		conns_node_b.clear();
		conns_len.clear();
		conns_stiff.clear();
		conns_damp.clear();
		conns_brk_thr.clear();
		conns_broken.clear();
	}

	void add_node(Node n)
	{
		//nodes.push_back(n);

		nodes_mass.push_back(n.mass);
		nodes_pos.push_back(n.pos);
		nodes_vel.push_back(n.vel);
		nodes_acc.push_back(n.acc);
		nodes_frc.push_back(n.frc);
		nodes_pinned.push_back(n.pinned);
	}

	void add_conn(Conn c)
	{
		//conns.push_back(c);

		conns_node_a.push_back(c.i);
		conns_node_b.push_back(c.j);
		conns_len.push_back(c.len);
		conns_stiff.push_back(c.stiff);
		conns_damp.push_back(c.damp);
		conns_brk_thr.push_back(c.brk_thr);
		conns_broken.push_back(c.broken);
	}
		

	size_t get_nodes_num() const
	{
		return nodes_pos.size();
	}

	size_t get_conns_num() const
	{
		return conns_len.size();
	}

	// main nodes and conns definition and state
	std::vector<float> nodes_mass;
	std::vector<vec3>  nodes_pos;
	std::vector<vec3>  nodes_vel;
	std::vector<vec3>  nodes_acc;
	std::vector<vec3>  nodes_frc;
	std::vector<char>  nodes_pinned;

	std::vector<size_t> conns_node_a;
	std::vector<size_t> conns_node_b;
	std::vector<float> conns_len;
	std::vector<float> conns_stiff;
	std::vector<float> conns_damp;
	std::vector<float> conns_brk_thr;
	std::vector<char>  conns_broken;

	// helper/temp stuff, to avoid reallocating all the time
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
};


void zero(std::vector<vec3>& v);
void copy( std::span<const vec3> vi, std::vector<vec3>& vo);


class RigSystemCommon
{
public:
	void load_definition_from_file();

	void clear();

	void add_node(Node n);
	void add_conn(Conn c);

	size_t get_nodes_num() const;
	size_t get_conns_num() const;

	void simulate(float dt);

	void compute_system_forces( std::span<const vec3> pos_in, 
								std::span<const vec3> vel_in);

	void system_derivative( std::span<const vec3> pos_in, 
							std::span<const vec3> vel_in, 
							std::vector<vec3>& acc_out);

	void integrate_system_radau2( float dt );

	RigStructureState m_s;
};

}
