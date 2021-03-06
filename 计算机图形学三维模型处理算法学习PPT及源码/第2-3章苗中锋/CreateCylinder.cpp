// CreateCylinder.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include<cmath>
#define pi 3.1415926
using namespace std;
typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;

int main()
{
	MyMesh mesh;
	int edge,height;
	cout << "edge:";
	cin >> edge;
	cout << "height:";
	cin >> height;
	MyMesh::VertexHandle *vhandle1 = new MyMesh::VertexHandle[edge+1];
	MyMesh::VertexHandle *vhandle2 = new MyMesh::VertexHandle[edge+1];
	for (int i = 0;i < edge;i++) {
		double x = 0.5*cos(pi * 2 / edge * i);
		double y = 0.5*sin(pi * 2 / edge * i);
		vhandle1[i] = mesh.add_vertex(MyMesh::Point(x, y, height/2));
	}
	for (int i = 0;i < edge;i++) {
		double x = 0.5*cos(pi * 2 / edge * i);
		double y = 0.5*sin(pi * 2 / edge * i);
		vhandle2[i] = mesh.add_vertex(MyMesh::Point(x, y, -height / 2));
	}
	vhandle1[edge] = mesh.add_vertex(MyMesh::Point(0, 0, height / 2));
	vhandle2[edge] = mesh.add_vertex(MyMesh::Point(0, 0, -height / 2));
	std::vector<MyMesh::VertexHandle>face_vhandles;
	switch (edge) {
	case 3:
		face_vhandles.clear();
		face_vhandles.push_back(vhandle1[0]);
		face_vhandles.push_back(vhandle1[2]);
		face_vhandles.push_back(vhandle1[1]);
		mesh.add_face(face_vhandles);
		face_vhandles.clear();
		face_vhandles.push_back(vhandle2[0]);
		face_vhandles.push_back(vhandle2[2]);
		face_vhandles.push_back(vhandle2[1]);
		mesh.add_face(face_vhandles);
		break;
	case 4:
		face_vhandles.clear();
		face_vhandles.push_back(vhandle1[0]);
		face_vhandles.push_back(vhandle1[2]);
		face_vhandles.push_back(vhandle1[1]);
		mesh.add_face(face_vhandles);
		face_vhandles.clear();
		face_vhandles.push_back(vhandle2[0]);
		face_vhandles.push_back(vhandle2[2]);
		face_vhandles.push_back(vhandle2[1]);
		mesh.add_face(face_vhandles);
		break;
	default:
		for (int i = 0;i < edge;i++) {
			int next1 = (i + 1) % edge;
			int next2 = (i + edge - 1) % edge;
			face_vhandles.clear();
			face_vhandles.push_back(vhandle1[edge]);
			face_vhandles.push_back(vhandle1[i]);
			face_vhandles.push_back(vhandle1[next1]);
			mesh.add_face(face_vhandles);
			face_vhandles.clear();
			face_vhandles.push_back(vhandle2[edge]);
			face_vhandles.push_back(vhandle2[i]);
			face_vhandles.push_back(vhandle2[next2]);
			mesh.add_face(face_vhandles);
		}
		break;
	}
	for (int i = 0;i < edge;i++) {
		int next = (i + 1) % edge;
		face_vhandles.clear();
		face_vhandles.push_back(vhandle1[i]);
		face_vhandles.push_back(vhandle2[i]);
		face_vhandles.push_back(vhandle1[next]);
		mesh.add_face(face_vhandles);
		face_vhandles.clear();
		face_vhandles.push_back(vhandle2[i]);
		face_vhandles.push_back(vhandle2[next]);
		face_vhandles.push_back(vhandle1[next]);
		mesh.add_face(face_vhandles);

	}
	try
	{
		if (!OpenMesh::IO::write_mesh(mesh, "output4 .off")) {
			std::cerr << "Cannot write mesh to file ' output4 .off ' " << std::endl;
			return 1;
		}
	}
	catch (std::exception&x) {
		std::cerr << x.what() << std::endl;
		return 1;
	}
    return 0;
}

