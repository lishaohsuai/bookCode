 // RemoveCapWithSplit.cpp: 定义控制台应用程序的入口点。
#include "stdafx.h"
#include <iostream>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include<cmath>
#include<vector>
#define pi 3.1415926
using namespace std;
typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;
int main()
{
	MyMesh mesh;
	std:vector<MyMesh::VertexHandle>addface;
	MyMesh::Point point;
	int count = 0;
	if (!OpenMesh::IO::read_mesh(mesh, "block.off"))
		return 1;
	int he_count = mesh.n_halfedges();
	for (auto h_it = mesh.halfedges_begin();h_it != mesh.halfedges_end();++h_it) {
		    MyMesh::VertexHandle v, vl, vr,v1,v2;
			v = mesh.to_vertex_handle(mesh.next_halfedge_handle(*h_it));
			vl = mesh.from_vertex_handle(*h_it);
			vr = mesh.to_vertex_handle(*h_it);
			v1 = mesh.to_vertex_handle(mesh.next_halfedge_handle(mesh.opposite_halfedge_handle(*h_it)));
			v2 = mesh.to_vertex_handle(mesh.next_halfedge_handle(*h_it));
			double x1 = mesh.point(v).data()[0];
			double y1 = mesh.point(v).data()[1];
			double z1 = mesh.point(v).data()[2];

			double x2 = mesh.point(vl).data()[0];
			double y2 = mesh.point(vl).data()[1];
			double z2 = mesh.point(vl).data()[2];
	
			double x3 = mesh.point(vr).data()[0];
			double y3 = mesh.point(vr).data()[1];
			double z3 = mesh.point(vr).data()[2];
			double dx = x3 - x2;
			double dy = y3 - y2;
			double dz = z3 - z2;
			double k = -((x2 - x1)*(x3 - x2) + (y2 - y1)*(y3 - y2) + (z2 - z1)*(z3 - z2)) / ((dx*dx) + (dy*dy) + (dz*dz));
			double xf, yf, zf;
			xf = x2 + k * dx;
			yf = y2 + k * dy;
			zf = z2 + k * dz;
			if (!((xf > x2&&xf > x3) || (xf < x2&&xf < x3)))
				if(!mesh.is_boundary(mesh.opposite_halfedge_handle(*h_it)))
			       mesh.vertex_split(mesh.calc_edge_midpoint(mesh.edge_handle(*h_it)), mesh.from_vertex_handle(*h_it),v1,v2);
			count++;
		    if (count ==he_count)break;
			
	}
	if (!OpenMesh::IO::write_mesh(mesh, "result.off"))
		return 1;

    return 0;
}

