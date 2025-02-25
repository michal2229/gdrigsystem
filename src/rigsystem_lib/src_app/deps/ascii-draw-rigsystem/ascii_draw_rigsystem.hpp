#pragma once

// #include <iostream>
// #include <chrono>
#include <cstdlib>
#include <memory>

// ascii-graphics lib
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wc++98-compat"
#include <Screen.hpp>
#include <agm.hpp>
#include <Camera.hpp>
#include <Lights.hpp>
#include <Mesh.hpp>
#pragma GCC diagnostic pop

namespace rigsystem
{

class RigSystemCommon;

}

namespace adr 
{

Mat4 translateXYZ(float x, float y, float z);

struct RigWireMesh
{
    void update_wire(size_t id, const Vert& a, const Vert& b, bool broken);
    std::vector<std::pair<Vert, Vert>> wires;
    std::vector<char> wbroken;
};

class RigRendererAscii
{
public:
	RigRendererAscii(size_t width=200, size_t height=42, float cam_fov=60.0f, float nearplane=0.1f, float farplane=1000.0f);

	size_t add_mesh(RigWireMesh);
	RigWireMesh& get_mesh(size_t id);
	size_t add_transform(size_t mesh_id, Mat4 t);
	Mat4& get_transform(size_t mesh_id, size_t transform_id);
	void update(size_t mesh_id, const rigsystem::RigSystemCommon& rig);
	void render();

	// ascii screen drawing functions - modified vs library to draw lines instread of triangles
	void project(std::pair<Vert, Vert>& w, Mat4 m);
	void centerFlipY(std::pair<Vert, Vert>& w);
	void drawWire(std::pair<Vert, Vert>& w, char c);
	void drawMeshWire(RigWireMesh m, char c);
	
	// objects to draw
	std::vector<RigWireMesh> meshes;
	std::vector<std::vector<Mat4>> transforms;

    // ascii drawing setup objects
	std::unique_ptr<Camera> p_camera;
	std::unique_ptr<Screen> p_screen;
};


}
