#include "stdafx.h"
#include <iostream>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
using namespace std;
typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;
MyMesh hh(MyMesh mesh, MyMesh::EdgeHandle edge);
MyMesh::VertexHandle findvertex(MyMesh mesh, MyMesh::Point point);
bool is_exist(MyMesh mesh, MyMesh::Point point);
bool EdgeCanCollapse(MyMesh mesh, MyMesh::HalfedgeHandle h);
int main()
{
	MyMesh mesh;
	int a;
	OpenMesh::IO::read_mesh(mesh, "output.off");
	mesh.request_edge_status();
	mesh.request_face_status();
	mesh.request_vertex_status();
	MyMesh::HalfedgeHandle h0, h1;
	MyMesh::VertexHandle v1, v2;
	MyMesh::EdgeHandle eh;

	double bianchang[100000];
	int i = 0, j;
	for (MyMesh::EdgeIter it = mesh.edges_begin(); it != mesh.edges_end(); ++it)
	{
		if (!mesh.is_boundary(*it))
		{
			bianchang[i]= mesh.calc_edge_length(*it);
			i++;
		}
	}
	double minlen = bianchang[0];
	for (j = 0; j < i; j++)//找到最短的边（内部的）
	{
		if (minlen > bianchang[j])
		{
			minlen = bianchang[j];
		}
	}

	/*cout << "输入您想要简化的次数："; 
	cin >> a;
	int num = 0;*/
	for (MyMesh::EdgeIter it = mesh.edges_begin(); it != mesh.edges_end(); ++it)
	{
		h0 = mesh.halfedge_handle(*it, 0);
		v1 = mesh.to_vertex_handle(h0);
		v2 = mesh.from_vertex_handle(h0);
		if (!EdgeCanCollapse(mesh, h0))
		{
			continue;
		}
		if (mesh.calc_edge_length(*it) == minlen)
		{
			mesh = hh(mesh, *it);
			/*num++;
			if (num == a)
				break;*/
		}
		/*if (mesh.n_faces() <= a)
		{
		break;
		}*/

	}
	OpenMesh::IO::write_mesh(mesh, "outputafter.off");
	return 0;
}

MyMesh hh(MyMesh mesh, MyMesh::EdgeHandle edge)
{
	mesh.request_vertex_status();
	MyMesh::VertexHandle newv;
	MyMesh::Point newpoint;
	MyMesh::HalfedgeHandle h0, h1;
	h0 = mesh.halfedge_handle(edge, 0);
	h1 = mesh.halfedge_handle(edge, 1);
	vector<MyMesh::VertexHandle> vhandle;
	vector<MyMesh::Point> point;
	for (MyMesh::HalfedgeHandle h = mesh.opposite_halfedge_handle(mesh.prev_halfedge_handle(h0));
	h != mesh.next_halfedge_handle(h1);
		h = mesh.opposite_halfedge_handle(mesh.prev_halfedge_handle(h)))
	{
		point.push_back(mesh.point(mesh.to_vertex_handle(h)));
	}
	for (MyMesh::HalfedgeHandle h = mesh.opposite_halfedge_handle(mesh.prev_halfedge_handle(h1));
	h != mesh.next_halfedge_handle(h0);
		h = mesh.opposite_halfedge_handle(mesh.prev_halfedge_handle(h)))
	{
		point.push_back(mesh.point(mesh.to_vertex_handle(h)));
	}
	newpoint = mesh.calc_edge_midpoint(edge);
	mesh.delete_vertex(mesh.to_vertex_handle(h0));
	mesh.delete_vertex(mesh.to_vertex_handle(h1));
	mesh.garbage_collection();
	newv = mesh.add_vertex(newpoint);
	for (int i = 0; i < point.size(); i++)
	{
		if (is_exist(mesh, point[i]))
			vhandle.push_back(findvertex(mesh, point[i]));
		else
			vhandle.push_back(mesh.add_vertex(point[i]));
	}
	for (int i = 0; i < vhandle.size() - 1; i++)
	{
		mesh.add_face(newv, vhandle[i], vhandle[i + 1]);
	}
	mesh.add_face(newv, vhandle[vhandle.size() - 1], vhandle[0]);
	return mesh;
}
MyMesh::VertexHandle findvertex(MyMesh mesh, MyMesh::Point point)
{
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		if (mesh.point(*v_it) == point)
		{
			return *v_it;
		}
	}
}
bool is_exist(MyMesh mesh, MyMesh::Point point)
{
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		if (mesh.point(*v_it) == point)
		{
			return 1;
		}
	}
	return 0;
}
bool EdgeCanCollapse(MyMesh mesh, MyMesh::HalfedgeHandle h)
{
	MyMesh::VertexHandle v1, v2, v3, v4;
	v1 = mesh.to_vertex_handle(h);
	v2 = mesh.from_vertex_handle(h);
	v3 = mesh.to_vertex_handle(mesh.next_halfedge_handle(h));
	v4 = mesh.to_vertex_handle(mesh.next_halfedge_handle(mesh.opposite_halfedge_handle(h)));
	if (mesh.is_boundary(v1) || mesh.is_boundary(v2))
	{
		return false;
	}
	/*int right = mesh.valence(v1);
	int left = mesh.valence(v2);
	int top = mesh.valence(v3);
	int buttom = mesh.valence(v4);
	return left + right < 13 && top>5 && buttom > 5;*/
}