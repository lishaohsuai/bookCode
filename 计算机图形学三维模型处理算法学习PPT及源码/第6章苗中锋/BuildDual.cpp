// BuildDual.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include<cmath>
#include<map>
#define pi 3.1415926
using namespace std;
typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;
typedef OpenMesh::PolyMesh_ArrayKernelT<>MynewMesh;
int main()
{
	MyMesh mesh;
	MynewMesh newmesh;
	if (!OpenMesh::IO::read_mesh(mesh, "bunny.off")) {
		std::cerr << "Cannot write mesh to file:s.off " << std::endl;
		return 1;
	}
	map<MyMesh::FaceHandle, MynewMesh::VertexHandle>fv_map;
	map<MyMesh::EdgeHandle, MynewMesh::VertexHandle>ev_map;
	double fhandle[3][3];
	MynewMesh::VertexHandle v;
	std::vector<MynewMesh::VertexHandle>newface_vhandles;
	for (auto f_it = mesh.faces_begin();f_it != mesh.faces_end();++f_it) {
		//v = newmesh.add_vertex(mesh.calc_face_centroid(*f_it));
		int k = 0;
		for (auto it = mesh.fv_begin(*f_it);it != mesh.fv_end(*f_it);++it) {
			auto point = mesh.point(*it);
			fhandle[k][0] = point.data()[0];
			fhandle[k][1] = point.data()[1];
			fhandle[k][2] = point.data()[2];
			++k;
		}
		double x = (fhandle[0][0] + fhandle[1][0] + fhandle[2][0]) / 3;
		double y = (fhandle[0][1] + fhandle[1][1] + fhandle[2][1]) / 3;
		double z = (fhandle[0][2] + fhandle[1][2] + fhandle[2][2]) / 3;
		v=newmesh.add_vertex(MyMesh::Point(x, y, z));
		fv_map.insert(make_pair(*f_it, newmesh.add_vertex(MyMesh::Point(x, y, z))));
	}
	for (auto e_it = mesh.edges_begin();e_it != mesh.edges_end();++e_it) {
		 v=newmesh.add_vertex(mesh.calc_edge_midpoint(*e_it));
		 ev_map.insert(make_pair(*e_it, newmesh.add_vertex(mesh.calc_edge_midpoint(*e_it))));

	}

	for (auto v_it = mesh.vertices_begin();v_it != mesh.vertices_end();++v_it) {
		   newface_vhandles.clear();
		    for (auto vh_it = mesh.voh_begin(*v_it);vh_it != mesh.voh_end(*v_it);++vh_it) {
				if (ev_map.count(mesh.edge_handle(*vh_it)) && fv_map.count(mesh.face_handle(*vh_it)) ){
					newface_vhandles.push_back(fv_map[mesh.face_handle(*vh_it)]);
					newface_vhandles.push_back(ev_map[mesh.edge_handle(*vh_it)]);
				}
		
		    }
			newmesh.add_face(newface_vhandles);
	}
	try
	{
		if (!OpenMesh::IO::write_mesh(newmesh, "output.off")) {
			std::cerr << "Cannot write mesh to file ' output.off ' " << std::endl;
			return 1;
		}
	}
	catch (std::exception&x) {
		std::cerr << x.what() << std::endl;
		return 1;
	}
	return 0;
}
