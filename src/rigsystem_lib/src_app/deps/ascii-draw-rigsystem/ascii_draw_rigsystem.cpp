#include "ascii_draw_rigsystem.hpp"

//#include <memory>
#include <rigsystem_common.hpp>

//#include <iostream>
//#include <chrono>
#include <cstdlib>

// ascii-graphics lib
#include <Screen.hpp>
#include <agm.hpp>
#include <Camera.hpp>
#include <Lights.hpp>
#include <Mesh.hpp>

using namespace rigsystem;

namespace adr 
{

Mat4 translateXYZ(float x, float y, float z) 
{
	Mat4 m;

	m.m[0][0] = 1.0f;
	m.m[1][1] = 1.0f;
	m.m[2][2] = 1.0f;
	m.m[3][3] = 1.0f;

	m.m[0][3] = x;
	m.m[1][3] = y;
	m.m[2][3] = z;

	return m;
}

void RigWireMesh::update_wire(size_t id, const Vert& a, const Vert& b, bool broken)
{
    // resize the vectors if necessary - will reallocate a bit initially but impact should be minor
    if (static_cast<size_t>(id) + 1> wires.size()) wires.resize(id+1);
    if (static_cast<size_t>(id) + 1> wbroken.size()) wbroken.resize(id+1);
    wires[id] = { a, b };
    wbroken[id] = broken;
}


RigRendererAscii::RigRendererAscii(size_t width, size_t height, float cam_fov, float nearplane, float farplane) 
{
    p_camera = std::unique_ptr<Camera>(new Camera(0.0f, 0.0f, 0.0f, static_cast<float>(width) / static_cast<float>(height), cam_fov, nearplane, farplane));
	p_screen = std::unique_ptr<Screen>(new Screen(static_cast<int>(width), static_cast<int>(height), *p_camera, LightD()));
}


size_t RigRendererAscii::add_mesh(RigWireMesh mesh) 
{
	meshes.push_back(mesh);
	transforms.push_back(std::vector<Mat4>());
	return meshes.size() - 1; 
}


RigWireMesh& RigRendererAscii::get_mesh(size_t id) 
{ 
	return meshes[id];
}

size_t RigRendererAscii::add_transform(size_t mesh_id, Mat4 t)
{
	transforms[mesh_id].push_back(t);
	return transforms[mesh_id].size() - 1;
}

Mat4& RigRendererAscii::get_transform(size_t mesh_id, size_t transform_id) 
{
	return transforms[mesh_id][transform_id];
}

void RigRendererAscii::update(size_t mesh_id, const rigsystem::RigSystemCommon& rig)
{
	for (size_t i = 0; i < rig.get_conns_num(); ++i)
	{
		vec3 pa = rig.m_s.nodes_pos[rig.m_s.conns_node_a[i]];
		vec3 pb = rig.m_s.nodes_pos[rig.m_s.conns_node_b[i]];

		Vert va = { pa.x, pa.y, pa.z };
		Vert vb = { pb.x, pb.y, pb.z };

		for (Mat4& t : transforms[mesh_id])
		{
			va = mult4(va, t);
			vb = mult4(vb, t);
		}

		// updating wire mesh
		meshes[mesh_id].update_wire(i, va, vb, rig.m_s.conns_broken[i]);
	}
}

void RigRendererAscii::render()
{
	p_screen->start();
	for (const RigWireMesh& mesh : meshes) drawMeshWire(mesh, '#');
	p_screen->print(); // Print the entire screen
	p_screen->clear(); // Clear the screen
}

	
/// functions below were adapted from ascii-graphics lib to draw lines instead of triangles

void RigRendererAscii:: project(std::pair<Vert, Vert>& w, Mat4 m) 
{
	w.first  = mult4(w.first, m);
	w.second = mult4(w.second, m);
}


void RigRendererAscii::centerFlipY(std::pair<Vert, Vert>& w)
{
	// Flip triangle verts
	w.first.y *= -1.f;
	w.second.y *= -1.f;

	// Scale by aspect ratio
	w.first.x *= p_screen->camera.a*2.5f;
	w.second.x *= p_screen->camera.a*2.5f;

	// Move the very center of screen
	w.first.x  += static_cast<float>(p_screen->width)/2.0f;
	w.first.y  += static_cast<float>(p_screen->height)/2.0f;
	w.second.x += static_cast<float>(p_screen->width)/2.0f;
	w.second.y += static_cast<float>(p_screen->height)/2.0f;
}


void RigRendererAscii::drawWire(std::pair<Vert, Vert>& w, char c)
{
    p_screen->drawLine(w.first, w.second, c);
}


void RigRendererAscii::drawMeshWire(RigWireMesh m, char c) 
{
    p_screen->renderMode = 0;  // no z-buffer
	for (size_t i = 0; i < m.wires.size(); ++i) 
    { 
        //if (m.wbroken[i]) continue;

        auto &wire = m.wires[i];

		project(wire, p_screen->camera.projMat);
		centerFlipY(wire);

        // draw the wire (correct drawn with content of c, broken with '.')
		drawWire(wire, m.wbroken[i] ? '.' : c);
	}
}

}
